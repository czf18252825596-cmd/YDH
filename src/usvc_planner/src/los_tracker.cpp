// LOS 沿边/路径跟踪库（改造自 YDH otter_planner/boundary_tracker）
// 改造点：
//   1. 闭环循环路径 → 单段路径（到达终点即停，不回起点）
//   2. 删除一圈停车逻辑（leave_stamp / has_left_start / arrived 回起点判定）
//   3. 去掉 ROS_INFO / ROS_INFO_THROTTLE（库层无 ROS 依赖）
//   4. advanceWaypoint 不再 safeMod 循环，到终点固定 waypoint_idx_ = n-1
// 保留核心算法：ALOS 自适应预瞄 + 弯道降速 + 越线推进 + 容差到达
#include "usvc_planner/los_tracker.h"
#include "usvc_planner/util.h"

#include <cmath>
#include <algorithm>

namespace usvc_planner {

void LosTracker::setBounds(double target_speed, double los_distance, double min_speed_turn,
                           double waypoint_tolerance, double adaptive_los_scale)
{
    target_speed_       = target_speed;
    los_                = los_distance;
    min_speed_turn_     = min_speed_turn;
    waypoint_tol_       = waypoint_tolerance;
    adaptive_los_scale_ = adaptive_los_scale;
}

void LosTracker::setPath(const std::vector<Eigen::Vector2d>& pts)
{
    pts_ = pts;
    reset();
}

void LosTracker::reset()
{
    waypoint_idx_ = 0;
    arrived_      = false;
    look_x_ = look_y_ = 0.0;
}

// 越线航点推进（单段路径版：到终点不再循环）
void LosTracker::advanceWaypoint(double x, double y)
{
    int n = (int)pts_.size();
    if (n < 2) return;

    // 已到终点航点：只判到达
    if (waypoint_idx_ >= n - 1) {
        double tx = pts_[n - 1].x();
        double ty = pts_[n - 1].y();
        if (dist2D(x, y, tx, ty) < waypoint_tol_) arrived_ = true;
        return;
    }

    int curr_idx = waypoint_idx_;
    double tx = pts_[curr_idx].x();
    double ty = pts_[curr_idx].y();
    double dist_to_target = dist2D(x, y, tx, ty);

    bool reached = (dist_to_target < waypoint_tol_);

    // 几何越线判据：船已越过当前目标点所在法平面也算到达
    if (!reached) {
        int prev_idx = std::max(curr_idx - 1, 0);  // 单段：不循环
        double seg_x = tx - pts_[prev_idx].x();
        double seg_y = ty - pts_[prev_idx].y();
        if (seg_x * (x - tx) + seg_y * (y - ty) > 0) reached = true;
    }

    if (reached) {
        // 单段路径：waypoint_idx_ 单调递增到 n-1，不循环
        while (waypoint_idx_ < n - 1) {
            waypoint_idx_++;
            int next = waypoint_idx_;
            double nx = pts_[next].x();
            double ny = pts_[next].y();

            // 跳过容差圆内的点
            if (dist2D(x, y, nx, ny) < waypoint_tol_) continue;

            // 检查是否已越过 next 的法平面
            int prev = std::max(next - 1, 0);
            double seg_x = nx - pts_[prev].x();
            double seg_y = ny - pts_[prev].y();
            if (seg_x * (x - nx) + seg_y * (y - ny) > 0) continue;

            break;  // 找到下一个有效跟踪航点
        }

        // 到终点航点判到达
        if (waypoint_idx_ >= n - 1) {
            double fx = pts_[n - 1].x();
            double fy = pts_[n - 1].y();
            if (dist2D(x, y, fx, fy) < waypoint_tol_) arrived_ = true;
        }
    }
}

// ALOS 自适应视线：船离路径越远，预瞄点自动拉长，平滑切入
void LosTracker::losLookAhead(double x, double y)
{
    int n = (int)pts_.size();
    if (n < 2) return;

    int idx = waypoint_idx_;
    look_x_ = pts_[idx].x();
    look_y_ = pts_[idx].y();

    double dist_to_path = dist2D(x, y, look_x_, look_y_);
    double adaptive_los = std::max(los_, dist_to_path * adaptive_los_scale_);

    double acc = 0.0;
    // 单段路径：不循环，到末尾止
    for (int i = 1; idx + i < n; i++) {
        int cur  = idx + i;
        int prev = idx + i - 1;
        double seg = dist2D(pts_[prev].x(), pts_[prev].y(), pts_[cur].x(), pts_[cur].y());
        acc += seg;
        if (acc >= adaptive_los) {
            double ratio = (adaptive_los - (acc - seg)) / seg;
            look_x_ = pts_[prev].x() + ratio * (pts_[cur].x() - pts_[prev].x());
            look_y_ = pts_[prev].y() + ratio * (pts_[cur].y() - pts_[prev].y());
            break;
        }
        look_x_ = pts_[cur].x();
        look_y_ = pts_[cur].y();
    }
}

// 弯道降速：当前航向偏了 或 前方有急弯 → 降速
double LosTracker::curveSpeed(double x, double y, double yaw) const
{
    int n = (int)pts_.size();
    if (n < 2) return 0.0;

    double desired_yaw_los = std::atan2(look_y_ - y, look_x_ - x);
    double final_speed = target_speed_;

    double abs_yaw_err = std::abs(normalizeAngle(desired_yaw_los - yaw));

    // 单段路径：next_idx 不循环，到末尾固定
    int next_idx = std::min(waypoint_idx_ + 1, n - 1);
    double next_path_yaw = std::atan2(pts_[next_idx].y() - pts_[waypoint_idx_].y(),
                                      pts_[next_idx].x() - pts_[waypoint_idx_].x());
    double path_angle_diff = std::abs(normalizeAngle(next_path_yaw - desired_yaw_los));

    if (abs_yaw_err > 30.0 * M_PI / 180.0 || path_angle_diff > 30.0 * M_PI / 180.0) {
        final_speed = min_speed_turn_;            // 急弯降到底速
    } else if (abs_yaw_err > 15.0 * M_PI / 180.0 || path_angle_diff > 20.0 * M_PI / 180.0) {
        final_speed = target_speed_ * 0.7;        // 缓弯降 30%
    }
    return final_speed;
}

LosTrackOutput LosTracker::update(double x, double y, double yaw, double t)
{
    LosTrackOutput out;
    int n = (int)pts_.size();
    if (n < 2) { out.desired_speed = 0.0; return out; }

    // 已到达终点：期望速度归零，保持当前航向
    if (arrived_) {
        out.desired_yaw   = yaw;
        out.desired_speed = 0.0;
        out.reached       = true;
        out.waypoint_idx  = waypoint_idx_;
        return out;
    }

    advanceWaypoint(x, y);
    losLookAhead(x, y);

    out.desired_yaw   = std::atan2(look_y_ - y, look_x_ - x);
    out.desired_speed = curveSpeed(x, y, yaw);
    out.reached       = arrived_;
    out.waypoint_idx  = waypoint_idx_;
    out.look_x       = look_x_;
    out.look_y       = look_y_;

    return out;
}

}  // namespace usvc_planner
