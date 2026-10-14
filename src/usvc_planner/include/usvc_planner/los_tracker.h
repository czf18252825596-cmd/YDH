#ifndef USVC_PLANNER_LOS_TRACKER_H
#define USVC_PLANNER_LOS_TRACKER_H

// LOS 沿边/路径跟踪模块（改造自 YDH boundary_tracker）
// 改造点：闭环一圈 → 单段路径（到达即停），去 ROS 依赖
// 输入：船位(xy) + 航向 + 路径点列 + 时间
// 输出：LOS 期望航向 + 弯道降速 + 到达标志

#include <Eigen/Core>
#include <vector>

namespace usvc_planner {

struct LosTrackOutput {
    double desired_yaw   = 0.0;  // LOS 期望航向 (rad)
    double desired_speed = 0.0;  // 弯道降速后的目标速度 (m/s)
    bool   reached       = false; // 是否到达终点
    int    waypoint_idx  = 0;     // 当前跟踪航点索引
    double look_x        = 0.0;   // LOS 预瞄点 x（调试用）
    double look_y        = 0.0;   // LOS 预瞄点 y（调试用）
};

class LosTracker {
public:
    void setBounds(double target_speed, double los_distance, double min_speed_turn,
                   double waypoint_tolerance, double adaptive_los_scale);
    void setPath(const std::vector<Eigen::Vector2d>& pts);  // 单段路径航点
    void reset();

    LosTrackOutput update(double x, double y, double yaw, double t);

    int    waypointIdx() const { return waypoint_idx_; }
    bool   reached()     const { return arrived_; }
    double lookX()       const { return look_x_; }
    double lookY()       const { return look_y_; }
    size_t pathSize()    const { return pts_.size(); }

private:
    void advanceWaypoint(double x, double y);              // 越线航点推进
    void losLookAhead(double x, double y);                 // ALOS 自适应视线
    double curveSpeed(double x, double y, double yaw) const; // 弯道降速

    std::vector<Eigen::Vector2d> pts_;
    double target_speed_ = 1.0, los_ = 8.0, min_speed_turn_ = 0.4;
    double waypoint_tol_ = 2.5, adaptive_los_scale_ = 1.5;

    int    waypoint_idx_ = 0;
    bool   arrived_      = false;
    double look_x_       = 0.0, look_y_ = 0.0;
};

}  // namespace usvc_planner

#endif
