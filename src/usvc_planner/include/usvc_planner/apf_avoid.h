#ifndef USVC_PLANNER_APF_AVOIDER_H
#define USVC_PLANNER_APF_AVOIDER_H

// APF 人工势场局部避障（拷自 YDH otter_planner/local_planner，去掉 ROS 依赖）
// 输入: 目标航向/速度 + 船头航向 + 雷达点云(船体系) + 水面方向
// 输出: APFOutput { heading_rad, speed, is_avoiding }

#include <Eigen/Core>
#include <vector>

namespace usvc_planner {

struct APFConfig {
    float repulsive_gain     = 0.6f;
    float attractive_gain    = 2.0f;
    float safe_distance      = 2.0f;
    float influence_distance = 20.0f;
    float sigma              = 3.5f;
    float min_speed          = 0.2f;
    float max_speed          = 1.0f;
};

struct APFOutput {
    float heading_rad;
    float speed;
    bool  is_avoiding;
};

// 避障策略：沿边用固定水域方向绕行；自主导航用目标方位叉乘动态绕行
enum class AvoidStrategy {
    EDGE_FOLLOWING = 0,  // 沿边：water_is_on_left 决定左右
    OPEN_WATER     = 1   // 自主导航：goal_dir 叉乘决定左右
};

class ApfAvoider {
public:
    explicit ApfAvoider(const APFConfig& cfg = APFConfig{});

    void setConfig(const APFConfig& cfg);

    APFOutput plan(
        float target_heading,
        float current_heading,
        float target_speed,
        const std::vector<Eigen::Vector3f>& lidar,
        bool water_is_on_left,
        AvoidStrategy strategy = AvoidStrategy::EDGE_FOLLOWING,
        const Eigen::Vector2f* goal_dir = nullptr);

private:
    APFConfig config_;
    int small_force_count_ = 0;
    float escape_heading_   = 0.0f;
    int   escape_frames_    = 0;
    int   recover_frame_    = 0;
    int   escape_min_frames_ = 0;
};

}  // namespace usvc_planner

#endif
