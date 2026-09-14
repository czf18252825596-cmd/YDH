#include "usvc_planner/apf_avoid.h"
#include "usvc_planner/util.h"
#include "usvc_planner/APFUpdate.h"

#include <limits>
#include <algorithm>
#include <cmath>

namespace usvc_planner {

namespace {
// 用"障碍物→船的径向方向"与"船→目标方向"做叉乘，决定绕行方向
// cross > 0 → +1（障碍在目标左侧 → 右绕）
// cross < 0 → -1（障碍在目标右侧 → 左绕）
// cross ≈ 0 → -1（共线，归入左绕，需配合逃逸机制脱困）
inline int vortexSign(const Eigen::Vector2f& force_dir, const Eigen::Vector2f* goal_dir) {
    if (!goal_dir) return 0;
    float cross = force_dir.x() * goal_dir->y() - force_dir.y() * goal_dir->x();
    return cross > 0.0f ? 1 : -1;
}
}  // namespace

ApfAvoider::ApfAvoider(const APFConfig& cfg) : config_(cfg) {}

void ApfAvoider::setConfig(const APFConfig& cfg) { config_ = cfg; }


APFOutput ApfAvoider::plan(
    float target_heading, 
    float current_heading,
    float target_speed,
    const std::vector<Eigen::Vector3f>& lidar,
    bool water_is_on_left,
    AvoidStrategy strategy,
    const Eigen::Vector2f* goal_dir)
{
    APFOutput out{target_heading, target_speed, false};

    float heading_diff   = normalizeAngle(target_heading - current_heading);
    float relative_angle = heading_diff;

    // 吸引力：指向目标方向
    Eigen::Vector2f attractive(std::cos(relative_angle), std::sin(relative_angle));
    attractive *= config_.attractive_gain;

    // 斥力累加
    Eigen::Vector2f repulsive(0.0f, 0.0f);
    float min_obs_dist = std::numeric_limits<float>::max();
    Eigen::Vector2f nearest_force_dir(0.0f, 0.0f);

    if (!lidar.empty()) {
        for (const auto& pt : lidar) {
            float d = pt.head<2>().norm();
            if (d < config_.influence_distance) {
                // 1. 径向斥力方向（障碍→船）
                Eigen::Vector2f force_dir(pt.x(), pt.y());
                force_dir = -force_dir.normalized();

                if (d < min_obs_dist) {
                    min_obs_dist = d;
                    nearest_force_dir = force_dir;
                }

                // 2. 涡旋场（切向绕行方向）
                Eigen::Vector2f vortex_dir;
                if (strategy == AvoidStrategy::OPEN_WATER && goal_dir != nullptr) {
                    // 自主导航：叉乘定边
                    vortex_dir = (vortexSign(force_dir, goal_dir) > 0)
                        ? Eigen::Vector2f(force_dir.y(), -force_dir.x())
                        : Eigen::Vector2f(-force_dir.y(), force_dir.x());
                } else {
                    // 沿边模式：由 water_is_on_left 决定固定绕行方向
                    if (water_is_on_left) {
                        vortex_dir = Eigen::Vector2f(force_dir.y(), -force_dir.x());
                    } else {
                        vortex_dir = Eigen::Vector2f(-force_dir.y(), force_dir.x());
                    }
                }

                // 3. 动态权重：距离越近切向权重越小（径向推开为主）
                float d_ratio = (config_.influence_distance - d)
                              / (config_.influence_distance - config_.safe_distance);
                d_ratio = std::max(0.0f, std::min(1.0f, d_ratio));
                float vortex_weight = 0.2f + 0.6f * (1.0f - d_ratio);
                Eigen::Vector2f final_dir = (force_dir + vortex_weight * vortex_dir).normalized();

                // 4. 高斯衰减斥力
                repulsive += config_.repulsive_gain *
                    std::exp(-std::pow(d - config_.safe_distance, 2) /
                            (2 * std::pow(config_.sigma, 2))) * final_dir;
            }
        }
    }

    // 极小值逃逸保持期
    if (escape_frames_ > 0) {
        escape_frames_--;
        if (escape_min_frames_ > 0) {
            escape_min_frames_--;
            out.heading_rad = escape_heading_;
            out.speed       = std::max(config_.min_speed * 2.0f, 0.6f);
            out.is_avoiding = true;
            return out;
        }
        if (!lidar.empty()) {
            Eigen::Vector2f total = attractive + repulsive;
            float mag  = total.norm();
            Eigen::Vector2f goal_dir_body(std::cos(relative_angle), std::sin(relative_angle));
            float proj = total.dot(goal_dir_body);

            if (mag > 0.5f && proj > 0.0f) {
                if (++recover_frame_ >= 5) {
                    escape_frames_ = 0;
                    recover_frame_ = 0;
                }
            } else {
                recover_frame_ = 0;
            }
        }
        if (escape_frames_ > 0) {
            out.heading_rad = escape_heading_;
            out.speed       = std::max(config_.min_speed * 2.0f, 0.6f);
            out.is_avoiding = true;
            return out;
        }
    }

    // 影响距离内有障碍
    if (min_obs_dist < config_.influence_distance) {
        Eigen::Vector2f total = attractive + repulsive;
        float force_angle = std::atan2(total.y(), total.x());
        out.heading_rad = normalizeAngle(current_heading + force_angle);

        float force_magnitude = total.norm();
        if (force_magnitude < 0.3f) {
            small_force_count_++;
            if (small_force_count_ > 3) {
                // 极小值触发：±45° 逃逸 3s
                float sign = 1.0f;
                if (strategy == AvoidStrategy::EDGE_FOLLOWING) {
                    sign = water_is_on_left ? 1.0f : -1.0f;
                } else if (goal_dir != nullptr && nearest_force_dir.squaredNorm() > 1e-6f) {
                    sign = static_cast<float>(vortexSign(nearest_force_dir, goal_dir));
                }
                escape_heading_ = normalizeAngle(out.heading_rad + sign * M_PI / 4.0f);
                escape_frames_   = 30;
                escape_min_frames_ = 10;
                recover_frame_     = 0;
                small_force_count_ = 0;
            }
        } else {
            small_force_count_ = 0;
            escape_frames_     = 0;
            escape_min_frames_ = 0;
            recover_frame_     = 0;
        }

        // 偏航限幅 ±30°（逃逸时不限）
        if (small_force_count_ == 0 && escape_frames_ == 0) {
            constexpr float kMaxDeflect = 30.0f * M_PI / 180.0f;
            float deflect = normalizeAngle(out.heading_rad - target_heading);
            if (deflect >  kMaxDeflect) out.heading_rad = normalizeAngle(target_heading + kMaxDeflect);
            if (deflect < -kMaxDeflect) out.heading_rad = normalizeAngle(target_heading - kMaxDeflect);
        }

        // 速度控制
        if (min_obs_dist <= config_.safe_distance) {
            out.speed = (small_force_count_ > 0) ? std::max(config_.min_speed * 2.0f, 0.6f) : 0.2f;
        } else {
            float raw = (min_obs_dist - config_.safe_distance)
                      / (config_.influence_distance - config_.safe_distance);
            float t = std::max(0.0f, std::min(1.0f, raw));
            float factor = 3 * t * t - 2 * t * t * t;
            out.speed = config_.min_speed + (target_speed - config_.min_speed) * factor;
            //   参数映射：
            //    distance_to_target = min_obs_dist - safe_distance （距安全边界的距离）
            //    max_speed          = target_speed - min_speed    （速度幅值）
            //    stop_distance      = influence_distance - safe_distance （影响范围）
            //    输出范围：[0, target_speed - min_speed] → 加 min_speed 偏移
            //    out.speed = config_.min_speed +
            //    GeoUtils::smooth_acceleration(
            //    min_obs_dist - config_.safe_distance,
            //    target_speed - config_.min_speed,
            //    config_.influence_distance - config_.safe_distance);
        
        
        }
    } else {
        // 无障碍：复位状态
        small_force_count_ = 0;
        escape_frames_     = 0;
        escape_min_frames_ = 0;
        recover_frame_     = 0;
    }

    return out;
}

}  // namespace usvc_planner
