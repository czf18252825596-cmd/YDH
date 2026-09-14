// planner_node（集成层：LOS 沿边跟踪 + APF 避障 + PD/PI 控制）
// 改造自 YDH otter_planner/navigator_node：
//   1. 定位源：TF map→base_link → 订阅 "/sensor/navi_euler跟"/sensor/navi_gnss获取船的位置
//   2. 路径源：LosPath.path 仍为 nav_msgs/Path，但 Pose.position.x=lat, y=lon
//   3. 投影：boundary_tracker 原为 map 系 xy 投影 → 改用 GeoUtil 球面投影
//   4. 输出：cmd_vel (Twist) → GuidanceCmd (obj_speed + obj_heading 度)
//   5. 删除：TF 广播 / nav_tracker / global_planner / visualization / 初始位姿
//   6. 保留：mode_manager / boundary_tracker(改单段) / local_planner(改 apf_avoider)
//            + PD 航向 + PI 速度 + 雷达空间掩码 + 状态上报

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>    
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>

#include "usvc_planner/util.h"
#include "usvc_planner/geo_util.h"
#include "usvc_planner/apf_avoid.h"
#include "usvc_planner/mode_manager.h"
#include "usvc_planner/los_tracker.h"
#include "usvc_planner/GuidanceCmd.h"
#include "usvc_planner/LosPath.h"
#include "usvc_planner/OtterCtrl.h"
#include "usvc_planner/OtterStatus.h"
#include "usvc_planner/APFUpdate.h"
#include "usvc_planner/TunableParams.h"


#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace up = usvc_planner;

class PlannerNode {
public:
    PlannerNode(ros::NodeHandle& nh)
    {
        // ---------- 参数加载----------
        // 非tunable参数：从 config 读取
        los_distance_        = 5.0;
        waypoint_tolerance_  = 2.5;
        min_speed_turn_      = 0.4;
        adaptive_los_scale_  = 1.5;
        // tunable参数：硬编码默认值，等待 control_node 通过 /planner_node/tunable_update 话题更新
        target_speed_     = 1.0;   // 等待tunable_update更新
        nh.param<bool>  ("/planner_node/lidar_spatial_mask",  lidar_spatial_mask_,  true);
        nh.param<bool>  ("/planner_node/debug_print",         debug_print_,         true);
        //nh.param<std::string>("/planner_node/lidar_topic",    lidar_topic_,         "/livox/lidar");//原始雷达
        nh.param<std::string>("/planner_node/lidar_topic",    lidar_topic_,         "/sensor/lidar_filter");//将体素滤波点云
        nh.param<std::string>("/planner_node/lidar_type",     lidar_type_,          "pointcloud2");

        // APF 参数
        up::APFConfig apf_cfg;
        nh.param<float>("/planner_node/apf_repulsive_gain",      apf_cfg.repulsive_gain,      0.6f);//斥力
        nh.param<float>("/planner_node/apf_attractive_gain",     apf_cfg.attractive_gain,     2.0f);//吸力
        nh.param<float>("/planner_node/apf_safe_distance",       apf_cfg.safe_distance,       2.0f);//安全距离
        nh.param<float>("/planner_node/apf_influence_distance",  apf_cfg.influence_distance,  20.0f);//最大影响距离
        nh.param<float>("/planner_node/apf_sigma",               apf_cfg.sigma,               3.5f);//高斯参数
        nh.param<float>("/planner_node/apf_min_speed",           apf_cfg.min_speed,           0.2f);
        nh.param<float>("/planner_node/apf_max_speed",           apf_cfg.max_speed,           1.0f);
        apf_.setConfig(apf_cfg);

        // 启动初始状态（从 yaml 读取，默认 STOP 待命）
        int start_run_mode = 0, start_task_state = 0;
        nh.param<int>("/planner_node/start_run_mode",   start_run_mode,   0);
        nh.param<int>("/planner_node/start_task_state", start_task_state, 0);
        mode_.setStartState(start_run_mode, start_task_state);

        // LOS 模块参数注入s
        los_.setBounds(target_speed_, los_distance_, min_speed_turn_,
                       waypoint_tolerance_, adaptive_los_scale_);

        // ---------- 订阅 ----------
        euler_sub_  = nh.subscribe("/sensor/navi_euler", 1, &PlannerNode::onEuler, this);
        gnss_sub_   = nh.subscribe("/sensor/navi_gnss", 1, &PlannerNode::onGnss, this);
        path_sub_   = nh.subscribe("/usvc_planner/path", 1, &PlannerNode::onPath, this);
        ctrl_sub_   = nh.subscribe("/usvc_planner/ctrl", 1, &PlannerNode::onCtrl, this);
        apf_update_sub_ = nh.subscribe<up::APFUpdate>("/planner_node/apf_update", 1, &PlannerNode::onApfUpdate, this);
        tunable_sub_    = nh.subscribe<up::TunableParams>("/planner_node/tunable_update", 1, &PlannerNode::onTunableUpdate, this);

        if (lidar_type_ == "pointcloud2") {
            lidar_sub_ = nh.subscribe(lidar_topic_, 1, &PlannerNode::onPointCloud2, this);
        } else {
            lidar_sub_ = nh.subscribe(lidar_topic_, 1, &PlannerNode::onLaserScan, this);
        }

        // ---------- 发布 ----------
        cmd_pub_    = nh.advertise<up::GuidanceCmd>("/usvc_planner/cmd", 1);
        status_pub_ = nh.advertise<up::OtterStatus>("/usvc_planner/status", 1);

        // ---------- 定时器 (10Hz) ----------
        timer_ = nh.createTimer(ros::Duration(0.1), &PlannerNode::onTimer, this);

        ROS_INFO("[Planner] 启动完成。模式: %s, 任务: %s",
                 mode_.runMode() == up::RunMode::AUTO_NAV ? "自主导航" : "沿边",
                 mode_.running() ? "已开始" : "待命");
    }

private:
    // ---------- ROS 接口 ----------
    ros::Subscriber euler_sub_, gnss_sub_, path_sub_, ctrl_sub_, lidar_sub_, apf_update_sub_, tunable_sub_;
    ros::Publisher  cmd_pub_, status_pub_;
    ros::Timer      timer_;

    // ---------- 模块实例 ----------
    up::ModeManager  mode_;
    up::LosTracker   los_;
    up::ApfAvoider   apf_;

    // ---------- 配置参数 ----------
    double target_speed_ = 1.2, los_distance_ = 5.0, waypoint_tolerance_ = 2.5;
    double min_speed_turn_ = 0.4; 
    double adaptive_los_scale_ = 1.5;
    double safe_margin_ = 1.5, boom_x_max_ = 7.0;   // 掩码可调参数（由上位机 tunable 覆盖 isMasked）
    bool   lidar_spatial_mask_ = true, debug_print_ = true;

    std::string lidar_topic_, lidar_type_;

    // ---------- 定位状态（来自 RTK） ----------
    double cur_lat_ = 0.0, cur_lon_ = 0.0, cur_heading_rad_ = 0.0;
    bool   has_euler_ = false;
    bool   has_gnss_  = false;


    // ---------- 路径状态 ----------
    std::vector<Eigen::Vector2d> path_latlon_;  // 路径点（经纬度: x=lat, y=lon）
    bool path_received_ = false;
    bool water_is_on_left_ = true;
    bool edge_active_ = false;

    // ---------- 雷达点云 ----------
    std::vector<Eigen::Vector3f> current_lidar_;

    // 定位回调
    void onEuler(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        // USVC heading: 0=北, 顺时针
        cur_heading_rad_ = (90.0 - msg->z)* M_PI / 180.0;
        has_euler_ = true;
    }

    void onGnss(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        cur_lat_  = msg->latitude;
        cur_lon_  = msg->longitude;
        has_gnss_ = true;
    }

    // 路径回调（LosPath.msg，path 字段经纬度点列）
    void onPath(const up::LosPath::ConstPtr& msg)
    {
        path_latlon_.clear();
        path_latlon_.reserve(msg->path.poses.size());
        for (const auto& p : msg->path.poses) {
            // 约定：x=lat, y=lon
            path_latlon_.emplace_back(p.pose.position.x, p.pose.position.y);
        }
        water_is_on_left_ = msg->water_is_on_left;
        path_received_ = true;

        // 注入 LOS tracker（单段路径，已去闭环）
        los_.setPath(path_latlon_);
        edge_active_ = true;

        ROS_INFO("[Planner] 收到路径 %zu 点, water_on_left=%d",
                 path_latlon_.size(), water_is_on_left_ ? 1 : 0);
    }

    // 控制指令回调（usvc 风格）
    void onCtrl(const up::OtterCtrl::ConstPtr& msg)
    {
        mode_.onCtrl(msg->run_mode, msg->task_state);

        if (mode_.consumeModeChanged()) {
            los_.reset();
            edge_active_ = false;
            ROS_INFO("[Planner] 模式切换: %s",
                     mode_.runMode() == up::RunMode::AUTO_NAV ? "自主导航" : "沿边");
        }
        if (mode_.consumeStopEvent()) {
            los_.reset();
            edge_active_ = false;
            ROS_INFO("[Planner] 任务停止");
        }
    }

    // LaserScan 回调（2D 雷达）
    void onLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        current_lidar_.clear();
        current_lidar_.reserve(msg->ranges.size());

        float angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float r = msg->ranges[i];
            if (r > msg->range_min && r < msg->range_max) {
                float x = r * std::cos(angle);
                float y = r * std::sin(angle);
                if (!isMasked(x, y)) {
                    current_lidar_.emplace_back(x, y, 0.0f);
                }
            }
            angle += msg->angle_increment;
        }
    }

    // PointCloud2 回调（Livox MID360 等 3D 雷达）
    void onPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        current_lidar_.clear();
        current_lidar_.reserve(cloud.points.size());
        for (const auto& pt : cloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
            // 船体系：x 前, y 左
            if (!isMasked(pt.x, pt.y)) {
                current_lidar_.emplace_back(pt.x, pt.y, pt.z);
            }
        }
    }

    // 雷达空间掩码：沿边模式屏蔽岸侧，保留前方近距离
    bool isMasked(float x, float y) const
    {
        if (!lidar_spatial_mask_) return false;//空间掩码没有启用
        if (mode_.runMode() != up::RunMode::ALONG_EDGE) return false;//仅沿边模式
        if (!edge_active_) return false;//沿边未激活

        // 喇叭口扇区屏蔽（沿边收集）：机械臂/喇叭口前伸区域不作为障碍，防机械臂被当障碍
        // boom_x_max_：喇叭口最前端离雷达距离(m)；safe_margin_：水侧掩码阈值
        if (x > 0.0f && x < boom_x_max_) return true;

        // 水侧掩码（沿边岸侧）
        bool masked = false;
        if (water_is_on_left_ && y < -safe_margin_)       masked = true;
        else if (!water_is_on_left_ && y > safe_margin_)  masked = true;
        return masked;
    }

    // ========================================================
    // 主控制循环 (10Hz)
    // ========================================================
    void onTimer(const ros::TimerEvent&)
    {
        // 任务状态守卫：STOP 或未开始 → 停船
        if (!mode_.running()) { publishStop(); publishStatus(); return; }

        // 定位数据未就绪
        if (!(has_gnss_ && has_euler_)) {
            ROS_WARN_THROTTLE(2.0, "[Planner] 等待 RTK 定位数据...");
            publishStop(); publishStatus();
            return;
        }

        // 路径未注入
        if (!path_received_ || !edge_active_) {
            ROS_WARN_THROTTLE(2.0, "[Planner] 等待路径注入...");
            publishStop(); publishStatus(); return;
        }

        // ---------- 1. LOS 制导（在经纬度域内做） ----------
        // LosTracker 原版是 map 系 xy，我们改造为"船位=0,0，目标在 ENU 投影后的 xy"
        // 但更简单：直接把经纬度喂进 los_（内部 dist2D/atan2 不变，数值上等价 ENU 局域）
        // 严格做法是把路径投到以船为原点的 ENU，这里用近似（短距离 < 1km 精度足够）
        // 当前船位在经纬度域：
        double ship_lat = cur_lat_, ship_lon = cur_lon_;
        double ship_yaw = cur_heading_rad_;

        // 把路径平移到以船为原点（局部经纬度差），并旋转到船体系
        // 这样 los_ 内部的 atan2 / dist2D 仍然是几何意义上的 LOS
        // 注意：经纬度差转米要乘 EARTH_RADIUS，但 los_ 内部用相对值，比例不变
        // 为简化，直接传"以船为原点的经纬度差"（los_ 输出 atan2 在 ENU 域）
        /*
        * ========== 坐标系与单位约定 ==========
        * 
        * 经纬度:   (lat, lon)  度       —— RTK 输入、路径话题
        * ENU:      (east, north) 米     —— 局部平面，几何运算
        * 船体系:   (x_body, y_body) 米  —— x前 y左，雷达/APF
        * 
        * 航向三种约定:
        *   bearing_deg:  度, 0=北, 顺时针  —— RTK heading, GuidanceCmd
        *   enu_yaw_rad:  弧度, 0=东, 逆时针 —— cur_heading_rad_, PD 控制器
        *   body_rad:     弧度, 0=船头, 左正 —— APF 内部
        * 
        * 转换关系:
        *   enu_yaw_rad = (90 - bearing_deg) * π/180
        *   bearing_deg = 90 - enu_yaw_rad * 180/π
        *   body_rad    = enu_yaw_rad - ship_yaw_enu_rad
        * 
        * 数据流:
        *   RTK(经纬度) → ENU(米) → 船体系(米) → LOS → APF
        *                                              ↓
        *    发布(bearing度) ← ENU(弧度) ← PD控制 ←─────┘
        */
        std::vector<Eigen::Vector2d> path_body;
        path_body.reserve(path_latlon_.size());
        for (const auto& p : path_latlon_) {
            double dlat = p.x() - ship_lat;
            double dlon = p.y() - ship_lon;
            // 经纬度差 → ENU 局部坐标（米）
            double east  = dlon * M_PI / 180.0 * up::EARTH_RADIUS * std::cos(ship_lat * M_PI / 180.0);
            double north = dlat * M_PI / 180.0 * up::EARTH_RADIUS;
            // 转船体系：x 前, y 左（ENU yaw=0 朝东时, east=x, north=y）
            double cos_y = std::cos(ship_yaw), sin_y = std::sin(ship_yaw);
            double x_body =  cos_y * east + sin_y * north;  // 前向
            double y_body = -sin_y * east + cos_y * north;  // 左向
            path_body.emplace_back(x_body, y_body);
        }
        los_.setPath(path_body);

        up::LosTrackOutput los_out = los_.update(0.0, 0.0, 0.0, ros::Time::now().toSec());
        // los_ 内部船位=(0,0), 航向=0（因为路径已转船体系，航向也相对）
        // 但 desired_yaw 在船体系，需要转回 ENU 再转回度
        double desired_yaw_body = los_out.desired_yaw;
        double desired_yaw_enu  = up::normalizeAngle(ship_yaw + desired_yaw_body);
        double final_speed = los_out.desired_speed;
        bool reached = los_out.reached;

        // ---------- 2. APF 避障 ----------
        bool apf_evading = false;
        if (!current_lidar_.empty() && !reached) {
            // 雷达点云已在船体系（x 前, y 左），直接喂 apf_
            // APF 的 target_heading / current_heading 都在船体系
            double apf_target_heading = desired_yaw_body;  // 船体系目标
            double apf_current_heading = 0.0;               // 船体系当前航向

            up::APFOutput apf_out = apf_.plan(
                apf_target_heading, apf_current_heading, final_speed,
                current_lidar_, water_is_on_left_,
                up::AvoidStrategy::EDGE_FOLLOWING, nullptr);

            if (std::abs(up::normalizeAngle(apf_out.heading_rad - apf_target_heading)) > 0.087) {
                apf_evading = true;
            }
            desired_yaw_body = apf_out.heading_rad;
            final_speed = apf_out.speed;
            desired_yaw_enu = up::normalizeAngle(ship_yaw + desired_yaw_body);
        }

        // ---------- 5. 发布 GuidanceCmd ----------
        // USVC 用 obj_heading 度（0=北, 顺时针）
        // ENU yaw 是 0=东, 逆时针；要转成 0=北顺时针：
        //   bearing_deg = 90 - yaw_deg  (mod 360)
        double heading_enu_deg = desired_yaw_enu * 180.0 / M_PI;
        double bearing_deg = std::fmod(90.0 - heading_enu_deg + 360.0, 360.0);//fmod取余

        up::GuidanceCmd cmd;
        // cmd.obj_speed   = static_cast<float>(current_cmd_speed_);  pid处理后的速度
        cmd.obj_speed   = static_cast<float>(final_speed);
        cmd.obj_heading = static_cast<float>(bearing_deg);
        cmd.is_avoiding = apf_evading;
        cmd.reached     = reached;
        cmd_pub_.publish(cmd);

        // ---------- 6. 状态上报 ----------
        publishStatus();

        // ---------- 7. 调试打印 ----------
        if (debug_print_) {
            ROS_INFO_THROTTLE(0.5,
                "[Planner] lat=%.6f lon=%.6f yaw=%.1f° bearing=%.1f° speed=%.2f wp=%d/%zu avoid=%d reached=%d",
                cur_lat_, cur_lon_, ship_yaw * 180.0 / M_PI, bearing_deg,
                final_speed, los_.waypointIdx(), path_latlon_.size(),
                apf_evading ? 1 : 0, reached ? 1 : 0);
        }
    }

    void publishStop()
    {
        up::GuidanceCmd cmd;
        cmd.obj_speed = 0.0f;
        double heading_enu_deg = cur_heading_rad_ * 180.0 / M_PI;
        double bearing_deg = std::fmod(90.0 - heading_enu_deg + 360.0, 360.0);//fmod 取余
        cmd.obj_heading = static_cast<float>(bearing_deg);
        cmd.is_avoiding = false;
        cmd.reached = false;
        cmd_pub_.publish(cmd);
    }

    void publishStatus()
    {
        up::OtterStatus s;
        s.run_mode = static_cast<uint16_t>(mode_.runMode());
        s.task_state = static_cast<uint8_t>(mode_.taskState());
        s.nav_state = 0;
        s.waypoint_idx = static_cast<uint16_t>(los_.waypointIdx());
        s.waypoint_num = static_cast<uint16_t>(path_latlon_.size());
        s.boundary_idx = 0;
        s.x = cur_lat_;  // 按约定 x=lat
        s.y = cur_lon_;
        s.yaw = cur_heading_rad_;
        status_pub_.publish(s);
    }

    // APF 参数更新回调
    void onApfUpdate(const usvc_planner::APFUpdate::ConstPtr& msg) {
        up::APFConfig cfg;
        cfg.repulsive_gain     = msg->repulsive_gain;
        cfg.attractive_gain    = msg->attractive_gain;
        cfg.safe_distance      = msg->safe_distance;
        cfg.influence_distance = msg->influence_distance;
        cfg.sigma              = msg->sigma;
        cfg.min_speed          = msg->min_speed;
        cfg.max_speed          = msg->max_speed;
        apf_.setConfig(cfg);
        ROS_INFO("[Planner] APF 参数已更新: repulsive=%.2f attractive=%.2f safe=%.2f influence=%.2f sigma=%.2f",
                 cfg.repulsive_gain, cfg.attractive_gain, cfg.safe_distance, cfg.influence_distance, cfg.sigma);
    }

    // 上位机可调参数更新回调（PID/LOS 组）
    void onTunableUpdate(const usvc_planner::TunableParams::ConstPtr& msg) {
        if (msg) {
            // ========== usvc_planner 参数（PID/LOS）==========
            target_speed_        = msg->target_speed;
            los_distance_        = msg->los_distance;
            waypoint_tolerance_  = msg->waypoint_tolerance;
            min_speed_turn_      = msg->min_speed_turn;
            adaptive_los_scale_  = msg->adaptive_los_scale;
            // --- 掩码参数 ---
            safe_margin_         = msg->safe_margin;
            boom_x_max_          = msg->boom_x_max;
            // 重新注入 LOS 边界（los_distance/min_speed_turn/waypoint_tolerance/adaptive_los_scale 需先更新）
            los_.setBounds(target_speed_, los_distance_, min_speed_turn_,
                           waypoint_tolerance_, adaptive_los_scale_);
            ROS_INFO("[Planner] 上位机调参更新:   target_speed=%.2f los_distance=%.2f waypoint_tolerance=%.2f min_speed_turn=%.2f adaptive_los_scale=%.2f safe_margin=%.2f boom_x_max=%.2f",
                     target_speed_,los_distance_, waypoint_tolerance_, min_speed_turn_, adaptive_los_scale_,
                     safe_margin_, boom_x_max_);
        }
    }

};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    PlannerNode node(nh);
    ros::spin();
    return 0;
}
