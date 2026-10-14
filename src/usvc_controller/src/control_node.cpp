#include <ros/ros.h>
#include "control/control.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <ros/package.h>  // 用于获取包路径
#include <unistd.h>       // 用于获取当前路径
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "usvc_controller/RunCtrl.h"
#include "usvc_controller/StateData.h"
#include "usvc_controller/EnableSwitches.h"

#include "usvc_controller/UpdateParams.h"
#include "config_manager_tool/config_manager_tool.h"

#include "usvc_planner/GuidanceCmd.h"
#include "usvc_planner/LosPath.h"
#include "usvc_planner/OtterCtrl.h"
#include "usvc_planner/APFUpdate.h"
#include "usvc_planner/TunableParams.h"

#include "visual_track/VisualTrack.h"
#include "std_msgs/Float32.h"

static Control m_Control;

static std::atomic<bool> new_runctrl_msg(false);
static std::atomic<bool> new_statedata_msg(false);

static ros::Subscriber upper_connect_sub;
static bool m_upper_connected = false;
static ros::Subscriber newTaskFile_sub;

static ros::Subscriber lidar_filter_sub;
static ros::Subscriber navi_euler_sub;
static ros::Subscriber navi_gnss_sub;
static ros::Subscriber runCtrl_sub;
static ros::Subscriber stateData_sub;

//沿边
static ros::Subscriber edge_cmd_sub;
static ros::Publisher edge_path_pub;
static ros::Publisher edge_ctrl_pub;

//视觉跟踪
static ros::Subscriber visual_cmd_sub;
static ros::Subscriber vt_has_target_sub;
static ros::Publisher  visual_ctrl_pub;
static ros::Publisher  visual_heading_pub;

static ros::Publisher vision_enable_pub;//目标识别+推流使能(并入 vision_en)
static ros::Publisher RunCtrl_pub; 
static ros::Publisher StateData_pub;
static ros::Publisher enableSwitches_bc_pub;   // 使能开关状态广播(latched)，供各功能包的订阅

static ros::ServiceServer sail_param_service;
static ros::ServiceServer back_param_service;
static ros::ServiceServer path_param_service;
static ros::ServiceServer enable_switches_service;
static ros::ServiceServer tunable_param_service;

static ros::ServiceServer charge_param_service;

static ros::Publisher apf_update_pub;//APF参数更新topic
static ros::Publisher tunable_update_pub;//上位机可调参数广播topic
static usvc_planner::TunableParams g_tunable_params;

// 显式赋非零安全初值，防止 static 零初始化或读取失败导致船速为 0
static void initDefaultTunableParams(usvc_planner::TunableParams& t) {
    t.target_speed        = 1.0;
    t.los_distance        = 8.0;
    t.waypoint_tolerance  = 2.5;
    t.min_speed_turn      = 0.4;
    t.adaptive_los_scale  = 1.5;
    t.safe_margin         = 1.5;
    t.boom_x_max          = 7.0;
    t.conf_thresh         = 0.5;
    t.max_heading_offset  = 15.0;
    t.approach_speed      = 0.6;
    t.heading_alpha       = 0.3;
    t.reached_area_thresh = 0.35;
    t.bottom_thresh       = 0.92;
    t.slow_area           = 0.03;
}

//出航参数
static bool sailParamsCallback(usvc_controller::UpdateParams::Request& req, usvc_controller::UpdateParams::Response& res) 
{
    if (req.node_name != "sail") 
    {
        res.success = false;
        res.message = "Not for sensor node";
        return true;
    }
    
    // 检查数组长度
    if (req.param_names.size() != req.param_values.size())
    {
        res.success = false;
        res.message = "Parameter arrays length mismatch";
        return true;
    }
    
    ROS_INFO("[sensor_node] Receiving batch update of %zu parameters", req.param_names.size());
    
    // ========== 1. 更新内存变量（立即生效）==========
    Param_Sail_t Param_Sail;
    for (size_t i = 0; i < req.param_names.size(); i++) 
    {
        const std::string& name = req.param_names[i];
        const std::string& value = req.param_values[i];
        
        ROS_DEBUG("  %s = %s", name.c_str(), value.c_str());

        if (name == "SPEED") 
        {
            Param_Sail.SAIL_SPEED = std::stod(value);
        }
        else if (name == "Y") 
        {
            Param_Sail.SAIL_Y = std::stod(value);
        }
        else if (name == "TOLERANCERADIUS0") 
        {
            Param_Sail.SAIL_TOLERANCERADIUS[0] = std::stod(value);
        }
        else if (name == "TOLERANCERADIUS1") 
        {
            Param_Sail.SAIL_TOLERANCERADIUS[1] = std::stod(value);
        }
        else if (name == "POS0") 
        {
            Param_Sail.DEFAULT_POS[0] = std::stod(value);
        }
        else if (name == "POS1") 
        {
            Param_Sail.DEFAULT_POS[1] = std::stod(value);
        }
        else 
        {
            ROS_WARN("[sail_node] Unknown parameter: %s", name.c_str());
        }

    }

    ROS_INFO("---[control_node.cpp] sailParams\r\n \
    SPEED:%lf\r\n \
    Y:%lf\r\n \
    TOLERANCERADIUS0:%lf\r\n \
    TOLERANCERADIUS1:%lf\r\n \
    POS0:%lf\r\n \
    POS1:%lf",
    Param_Sail.SAIL_SPEED, 
    Param_Sail.SAIL_Y, 
    Param_Sail.SAIL_TOLERANCERADIUS[0], 
    Param_Sail.SAIL_TOLERANCERADIUS[1], 
    Param_Sail.DEFAULT_POS[0], 
    Param_Sail.DEFAULT_POS[1]);


    m_Control.setParam_Sail(Param_Sail);
    
    res.success = true;
    res.message = "Parameters updated successfully";
    
    ROS_INFO("[sensor_node] Batch update complete, synced to param server");
    return true;
}

//返航参数
static bool backParamsCallback(usvc_controller::UpdateParams::Request& req, usvc_controller::UpdateParams::Response& res) 
{
    if (req.node_name != "back") 
    {
        res.success = false;
        res.message = "Not for sensor node";
        return true;
    }
    
    // 检查数组长度
    if (req.param_names.size() != req.param_values.size())
    {
        res.success = false;
        res.message = "Parameter arrays length mismatch";
        return true;
    }
    
    ROS_INFO("[sensor_node] Receiving batch update of %zu parameters", req.param_names.size());
    
    // ========== 1. 更新内存变量（立即生效）==========
    Param_Back_t Param_Back;
    for (size_t i = 0; i < req.param_names.size(); i++) 
    {
        const std::string& name = req.param_names[i];
        const std::string& value = req.param_values[i];
        
        ROS_DEBUG("  %s = %s", name.c_str(), value.c_str());
        
        if (name == "MAXSPEED") 
        {
            Param_Back.MAXSPEED = std::stod(value);
        }
        else if (name == "MINSPEED") 
        {
            Param_Back.MINSPEED = std::stod(value);
        }
        else if (name == "LOSDISTANCE") 
        {
            Param_Back.LOSDISTANCE = std::stod(value);
        }
        else if (name == "OBSTACLE_INFLUENCE_DISTANCE") 
        {
            Param_Back.OBSTACLE_INFLUENCE_DISTANCE = std::stod(value);
        }
        else if (name == "SAFE_DISTANCE") 
        {
            Param_Back.SAFE_DISTANCE = std::stod(value);
        }
        else if (name == "REPULSIVE_GAIN") 
        {
            Param_Back.REPULSIVE_GAIN = std::stod(value);
        }
        else if (name == "ATTRACTIVE_GAIN") 
        {
            Param_Back.ATTRACTIVE_GAIN = std::stod(value);
        }
        else if (name == "SIGMA") 
        {
            Param_Back.SIGMA = std::stod(value);
        }
        else if (name == "TOLERANCERADIUS") 
        {
            Param_Back.TOLERANCERADIUS = std::stod(value);
        }
        else if (name == "Y") 
        {
            Param_Back.Y = std::stod(value);
        }
        else if (name == "POS0") 
        {
            Param_Back.DEFAULT_POS[0] = std::stod(value);
        }
        else if (name == "POS1") 
        {
            Param_Back.DEFAULT_POS[1] = std::stod(value);
        }
        else
        {
            ROS_WARN("[back_node] Unknown parameter: %s", name.c_str());
        }
    }
    ROS_INFO("---[control_node.cpp] backParams\r\n \
    MAXSPEED:%lf\r\n \
    MINSPEED:%lf\r\n \
    LOSDISTANCE:%lf\r\n \
    OBSTACLE_INFLUENCE_DISTANCE:%lf\r\n \
    SAFE_DISTANCE:%lf\r\n \
    REPULSIVE_GAIN:%lf\r\n \
    ATTRACTIVE_GAIN:%lf\r\n \
    SIGMA:%lf\r\n \
    TOLERANCERADIUS:%lf\r\n \
    Y:%lf\r\n \
    POS0:%lf\r\n \
    POS1:%lf",
    Param_Back.MAXSPEED, 
    Param_Back.MINSPEED, 
    Param_Back.LOSDISTANCE, 
    Param_Back.OBSTACLE_INFLUENCE_DISTANCE, 
    Param_Back.SAFE_DISTANCE, 
    Param_Back.REPULSIVE_GAIN,
    Param_Back.ATTRACTIVE_GAIN,
    Param_Back.SIGMA,
    Param_Back.TOLERANCERADIUS,
    Param_Back.Y,
    Param_Back.DEFAULT_POS[0],
    Param_Back.DEFAULT_POS[1]);

    m_Control.setParam_Back(Param_Back);

    res.success = true;
    res.message = "Parameters updated successfully";
    
    ROS_INFO("[sensor_node] Batch update complete, synced to param server");
    return true;
}

// ===== 充电参数服务 =====
static bool chargeParamsCallback(usvc_controller::UpdateParams::Request& req, 
                                  usvc_controller::UpdateParams::Response& res) 
{
    if (req.node_name != "charge") 
    {
        res.success = false;
        res.message = "Not for charge node";
        return true;
    }
    
    if (req.param_names.size() != req.param_values.size())
    {
        res.success = false;
        res.message = "Parameter arrays length mismatch";
        return true;
    }
    
    Param_Charge_t param;
    for (size_t i = 0; i < req.param_names.size(); ++i)
    {
        const std::string& name = req.param_names[i];
        const std::string& value = req.param_values[i];

        if (name == "LOW_BATTERY")
        {
            param.LOW_BATTERY = std::stod(value);
        }
        else if (name == "CHARGE_START")
        {
            param.CHARGE_START = std::stod(value);
        }
        else if (name == "CHARGE_STOP")
        {
            param.CHARGE_STOP = std::stod(value);
        }
        else 
        {
            ROS_WARN("[control_node] Unknown charge param: %s", name.c_str());
        }
    }

    m_Control.setParam_Charge(param);

    res.success = true;
    res.message = "Charge parameters updated";
    ROS_INFO("[control_node] Charge params: low=%.1f start=%.1f stop=%.1f",
             param.LOW_BATTERY, param.CHARGE_START, param.CHARGE_STOP);
    return true;
}

//轨迹跟踪参数
static bool pathParamsCallback(usvc_controller::UpdateParams::Request& req, usvc_controller::UpdateParams::Response& res) 
{ 
    ROS_INFO("pathParamsCallback");
    if (req.node_name != "path") 
    {
         ROS_INFO("!=path");
        res.success = false;
        res.message = "Not for sensor node";
        return true;
    }
    
    // 检查数组长度
    if (req.param_names.size() != req.param_values.size())
    {
        ROS_INFO("req.param_names.size()!=req.param_values.size() : %zu != %zu", req.param_names.size(), req.param_values.size());
        res.success = false;
        res.message = "Parameter arrays length mismatch";
        return true;
    }
    
    ROS_INFO("[sensor_node] Receiving batch update of %zu parameters", req.param_names.size());
    
    // ========== 1. 更新内存变量（立即生效）==========
    Param_PathTracking_t Param_PathTracking;
    for (size_t i = 0; i < req.param_names.size(); i++) 
    {
        const std::string& name = req.param_names[i];
        const std::string& value = req.param_values[i];
        
        ROS_DEBUG("  %s = %s", name.c_str(), value.c_str());
        
        if (name == "MAXSPEED") 
        {
            Param_PathTracking.MAXSPEED = std::stod(value);
        }
        else if (name == "MINSPEED") 
        {
            Param_PathTracking.MINSPEED = std::stod(value);
        }
        else if (name == "LOSDISTANCE") 
        {
            Param_PathTracking.LOSDISTANCE = std::stod(value);
        }
        else if (name == "OBSTACLE_INFLUENCE_DISTANCE") 
        {
            Param_PathTracking.OBSTACLE_INFLUENCE_DISTANCE = std::stod(value);
        }
        else if (name == "SAFE_DISTANCE") 
        {
            Param_PathTracking.SAFE_DISTANCE = std::stod(value);
        }
        else if (name == "REPULSIVE_GAIN") 
        {
            Param_PathTracking.REPULSIVE_GAIN = std::stod(value);
        }
        else if (name == "ATTRACTIVE_GAIN") 
        {
            Param_PathTracking.ATTRACTIVE_GAIN = std::stod(value);
        }
        else if (name == "SIGMA") 
        {
            Param_PathTracking.SIGMA = std::stod(value);
        }
        else if (name == "TOLERANCERADIUS") 
        {
            Param_PathTracking.TOLERANCERADIUS = std::stod(value);
        }
        else 
        {
            ROS_WARN("[path_node] Unknown parameter: %s", name.c_str());
        }
    }
    ROS_INFO("---[control_node.cpp] pathParams\r\n \
    MAXSPEED:%lf\r\n \
    MINSPEED:%lf\r\n \
    LOSDISTANCE:%lf\r\n \
    OBSTACLE_INFLUENCE_DISTANCE:%lf\r\n \
    SAFE_DISTANCE:%lf\r\n \
    REPULSIVE_GAIN:%lf\r\n \
    ATTRACTIVE_GAIN:%lf\r\n \
    SIGMA:%lf\r\n \
    TOLERANCERADIUS:%lf",
    Param_PathTracking.MAXSPEED, 
    Param_PathTracking.MINSPEED, 
    Param_PathTracking.LOSDISTANCE, 
    Param_PathTracking.OBSTACLE_INFLUENCE_DISTANCE, 
    Param_PathTracking.SAFE_DISTANCE, 
    Param_PathTracking.REPULSIVE_GAIN,
    Param_PathTracking.ATTRACTIVE_GAIN,
    Param_PathTracking.SIGMA,
    Param_PathTracking.TOLERANCERADIUS);

    m_Control.setParam_PathTracking(Param_PathTracking);
    // 同步 APF 参数给 planner
    usvc_planner::APFUpdate apf_msg;
    apf_msg.repulsive_gain     = Param_PathTracking.REPULSIVE_GAIN;
    apf_msg.attractive_gain    = Param_PathTracking.ATTRACTIVE_GAIN;
    apf_msg.safe_distance      = Param_PathTracking.SAFE_DISTANCE;
    apf_msg.influence_distance = Param_PathTracking.OBSTACLE_INFLUENCE_DISTANCE;
    apf_msg.sigma              = Param_PathTracking.SIGMA;
    apf_msg.min_speed          = Param_PathTracking.MINSPEED;
    apf_msg.max_speed          = Param_PathTracking.MAXSPEED;
    apf_update_pub.publish(apf_msg);
    res.success = true;
    res.message = "Path parameters updated successfully";
    ROS_INFO("[control_node] APF 参数已同步给 planner");
    return true;
}

// 上位机可调参数（planner  + visual_track）转发服务回调
static bool tunableParamsCallback(usvc_controller::UpdateParams::Request& req,
                                  usvc_controller::UpdateParams::Response& res)
{
    ROS_INFO("tunableParamsCallback");
    if (req.node_name != "tunable") {
        res.success = false;
        res.message = "Not for tunable";
        return true;
    }
    if (req.param_names.size() != req.param_values.size()) {
        res.success = false;
        res.message = "Parameter arrays length mismatch";
        return true;
    }

    // 在当前内存真值 g_tunable_params 基础上增量更新，未传字段保持上次值
    for (size_t i = 0; i < req.param_names.size(); i++) {
        const std::string& name = req.param_names[i];
        double v = std::stod(req.param_values[i]);

        // ========== usvc_planner 参数 ==========
        if      (name == "target_speed")        g_tunable_params.target_speed = v;
        else if (name == "los_distance")        g_tunable_params.los_distance = v;
        else if (name == "waypoint_tolerance")  g_tunable_params.waypoint_tolerance = v;
        else if (name == "min_speed_turn")      g_tunable_params.min_speed_turn = v;
        else if (name == "adaptive_los_scale")  g_tunable_params.adaptive_los_scale = v;
        else if (name == "safe_margin")         g_tunable_params.safe_margin = v;
        else if (name == "boom_x_max")          g_tunable_params.boom_x_max = v;
        // ========== visual_track 参数 ==========
        else if (name == "conf_thresh")         g_tunable_params.conf_thresh = v;
        else if (name == "max_heading_offset")  g_tunable_params.max_heading_offset = v;
        else if (name == "approach_speed")      g_tunable_params.approach_speed = v;
        else if (name == "heading_alpha")       g_tunable_params.heading_alpha = v;
        else if (name == "reached_area_thresh") g_tunable_params.reached_area_thresh = v;
        else if (name == "bottom_thresh")       g_tunable_params.bottom_thresh = v;
        else if (name == "slow_area")           g_tunable_params.slow_area = v;
        else ROS_WARN("[tunable] Unknown parameter: %s", name.c_str());
    }

    // YAML 落盘已由 upper_node 统一完成，此处不再重复写文件，消除双写冲突
    // 广播给 planner 与 visual_track（latched）
    tunable_update_pub.publish(g_tunable_params);
    ROS_INFO("[control_node] 上位机可调参数已同步给 planner/visual_track");
    res.success = true;
    res.message = "tunable params updated";
    return true;
}

//EnableSwitches 数据转消息
static void enableSwitches_data_to_msg(const EnableSwitches_t &data, usvc_controller::EnableSwitches& msg)
{
    msg.mapping_en = data.mapping_en;
    msg.back_lowbatt_en = data.back_lowbatt_en;
    msg.back_linklost_en = data.back_linklost_en;
    msg.vision_en = data.vision_en;
}

//使能开关参数  
static bool enableSwitchesParamsCallback(usvc_controller::UpdateParams::Request& req,
                                         usvc_controller::UpdateParams::Response& res)
{
    ROS_INFO("enableSwitchesParamsCallback");
    
    if (req.node_name != "enable_switches")
    {
        ROS_INFO("!=enable_switches");
        res.success = false;
        res.message = "Not for enable_switches";
        return true;
    }

    // 检查数组长度
    if (req.param_names.size() != req.param_values.size())
    {
        ROS_INFO("req.param_names.size()!=req.param_values.size() : %zu != %zu",
                 req.param_names.size(), req.param_values.size());
        res.success = false;
        res.message = "Parameter arrays length mismatch";
        return true;
    }

    ROS_INFO("[control_node] Receiving batch update of %zu parameters", req.param_names.size());

    // ========== 1. 更新内存变量（立即生效）==========
    EnableSwitches_t param;  // 使用协议定义的结构体
    param.mapping_en       = 0;
    param.back_lowbatt_en  = 0;
    param.back_linklost_en = 0;
    param.vision_en        = 0;

    for (size_t i = 0; i < req.param_names.size(); i++)
    {
        const std::string& name  = req.param_names[i];
        const std::string& value = req.param_values[i];

        ROS_DEBUG("  %s = %s", name.c_str(), value.c_str());

        if      (name == "mapping_en")       param.mapping_en       = (uint8_t)std::stoi(value);
        else if (name == "back_lowbatt_en")  param.back_lowbatt_en  = (uint8_t)std::stoi(value);
        else if (name == "back_linklost_en") param.back_linklost_en = (uint8_t)std::stoi(value);
        else if (name == "vision_en")        param.vision_en        = (uint8_t)std::stoi(value);
        else
        {
            ROS_WARN("[enable_switches_node] Unknown parameter: %s", name.c_str());
        }
    }

    // 打印详细参数
    ROS_INFO("---[control_node.cpp] enableSwitchesParams\r\n \
    mapping_en:%d\r\n \
    back_lowbatt_en:%d\r\n \
    back_linklost_en:%d\r\n \
    vision_en:%d",
    param.mapping_en,
    param.back_lowbatt_en,
    param.back_linklost_en,
    param.vision_en);

    // ① 存内存（executeTask / 自动返航用）
    m_Control.setEnableSwitches(param);
    // ② 建图开关不再单独下发 /fast_lio/enable（已并入 ③ 总广播，fast_lio 直接订阅 /upper/EnableSwitches 取 mapping_en）
    // ③ 总线广播给各功能包（PathTrack 无此步，enable 为多消费者）
    usvc_controller::EnableSwitches msg;
    enableSwitches_data_to_msg(param, msg);
    enableSwitches_bc_pub.publish(msg);

    res.success = true;
    res.message = "EnableSwitches updated successfully";

    ROS_INFO("[control_node] Batch update complete, synced to param server");
    return true;
}

//RunCtrl 消息转数据
static void runctrl_msg_to_data(const usvc_controller::RunCtrl &msg, RunCtrl_t& data)
{
    data.runMode = msg.run_mode;
    data.Y = msg.Y;
    data.H = msg.H;
    data.objSpeed = msg.obj_speed;
    data.objHeading = msg.obj_heading;
    data.isAvoiding = msg.is_avoiding;
    data.has_dock = msg.has_dock;
    
    data.dock_ip[0] = msg.dock_ip[0];
    data.dock_ip[1] = msg.dock_ip[1];
    data.dock_ip[2] = msg.dock_ip[2];
    data.dock_ip[3] = msg.dock_ip[3];
    
    data.dock_port = msg.dock_port;
    data.task_state = msg.task_state;
    data.task_tepy = msg.task_tepy;
    data.ctrl_mode = msg.ctrl_mode;
}

//RunCtrl 数据转消息
static void runctrl_data_to_msg(const RunCtrl_t &data, usvc_controller::RunCtrl& msg)
{
    msg.run_mode = data.runMode;
    msg.Y = data.Y;
    msg.H = data.H;
    msg.obj_speed = data.objSpeed;
    msg.obj_heading = data.objHeading;
    msg.is_avoiding = data.isAvoiding;
    msg.has_dock = data.has_dock;
    
    msg.dock_ip[0] = data.dock_ip[0];
    msg.dock_ip[1] = data.dock_ip[1];
    msg.dock_ip[2] = data.dock_ip[2];
    msg.dock_ip[3] = data.dock_ip[3];
    
    msg.dock_port = data.dock_port;
    msg.task_state = data.task_state;
    msg.task_tepy = data.task_tepy;
    msg.ctrl_mode = data.ctrl_mode;
}

//State 消息转数据
static void statedata_msg_to_data(const usvc_controller::StateData &msg, StateData_t& data)
{
    // 1. 传感器数据
    data.sensor.humi = msg.humidity;
    data.sensor.temp = msg.temperature;
    data.sensor.leak[0] = msg.leak_detection[0];
    data.sensor.leak[1] = msg.leak_detection[1];
    
    // 2. 电源状态
    data.power.volage = msg.voltage;
    data.power.current = msg.current;
    
    // 3. 电源管理
    data.pms.Thruster = msg.Thruster;
    data.pms.LEDScre = msg.LEDScre;
    data.pms.unused2 = msg.unused2;
    data.pms.unused3 = msg.unused3;
    
    // 4. 电池状态
    data.battery.Q = msg.battery_q;
    data.battery.V = msg.battery_v;
    data.battery.A = msg.battery_a;
    data.battery.T = msg.battery_t;
    
    // 5. 运行状态
    data.run.runMode = msg.runMode;
    data.run.Y = msg.y;
    data.run.H = msg.h;
    data.run.objSpeed = msg.objSpeed;
    data.run.objHeading = msg.objHeading;
    data.run.objPos[0] = msg.objPos[0];
    data.run.objPos[1] = msg.objPos[1];
    data.run.task_tepy = msg.task_tepy;
    data.run.task_sta = msg.task_sta;
    data.run.vision_state = msg.vision_state;
    data.run.mapping_state = msg.mapping_state;
    
    // 6. 导航数据
    data.navi.ins_sta = msg.ins_status;
    data.navi.timeTick = msg.timestamp;
    
    data.navi.gyro_roll = msg.gyro_roll;
    data.navi.gyro_pitch = msg.gyro_pitch;
    data.navi.gyro_heading = msg.gyro_heading;
    
    data.navi.roll = msg.roll;
    data.navi.pitch = msg.pitch;
    data.navi.heading = msg.heading;
    
    data.navi.acc_x = msg.acc_x;
    data.navi.acc_y = msg.acc_y;
    data.navi.acc_z = msg.acc_z;
    
    data.navi.vel_x = msg.vel_x;
    data.navi.vel_y = msg.vel_y;
    data.navi.vel_z = msg.vel_z;
    
    data.navi.latitude = msg.latitude;
    data.navi.longitude = msg.longitude;
    data.navi.altitude = msg.altitude;
    
    data.navi.vel_n = msg.vel_n;
    data.navi.vel_e = msg.vel_e;
    data.navi.vel_d = msg.vel_d;
    
    data.navi.satellite_count = msg.satellite_count;
    data.navi.pos_sta = msg.pos_status;
    data.navi.heading_sta = msg.heading_status;
    
    data.navi.position_std_lat = msg.pos_std_lat;
    data.navi.position_std_lon = msg.pos_std_lon;
    
    // 7. 推进器状态
    data.thruster.left_Val = msg.left_thruster_val;
    data.thruster.left_sta = msg.left_thruster_status;
    data.thruster.left_duty_scaled = msg.left_duty_cycle;
    data.thruster.left_rpm = msg.left_rpm;
    data.thruster.left_MOST = msg.left_mos_temp;
    data.thruster.left_total_current = msg.left_total_current;
    data.thruster.left_current_in = msg.left_input_current;
    
    data.thruster.right_Val = msg.right_thruster_val;
    data.thruster.right_sta = msg.right_thruster_status;
    data.thruster.right_duty_scaled = msg.right_duty_cycle;
    data.thruster.right_rpm = msg.right_rpm;
    data.thruster.right_MOST = msg.right_mos_temp;
    data.thruster.right_total_current = msg.right_total_current;
    data.thruster.right_current_in = msg.right_input_current;
    
    // 8. 灯状态
    data.light.LED = msg.LED;
    data.light.CautionLED = msg.CautionLED;
    
    // 9. 码头信息
    data.dock.connect = msg.connect;
    data.dock.dock_ID = msg.dock_ID;
    data.dock.presetPos[0][0] = msg.preset_positions0[0];
    data.dock.presetPos[0][1] = msg.preset_positions0[1];
    data.dock.presetPos[1][0] = msg.preset_positions1[0];
    data.dock.presetPos[1][1] = msg.preset_positions1[1];
    data.dock.berth = msg.berth_status;
    data.dock.change = msg.charge_status;
}

//State 数据转消息
static void statedata_data_to_msg(const StateData_t &data, usvc_controller::StateData& msg)
{
    // 1. 传感器数据
    msg.humidity = data.sensor.humi;
    msg.temperature = data.sensor.temp;
    msg.leak_detection[0] = data.sensor.leak[0];
    msg.leak_detection[1] = data.sensor.leak[1];
    
    // 2. 电源状态
    msg.voltage = data.power.volage;
    msg.current = data.power.current;
    
    // 3. 电源管理
    msg.Thruster = data.pms.Thruster;
    msg.LEDScre = data.pms.LEDScre;
    msg.unused2 = data.pms.unused2;
    msg.unused3 = data.pms.unused3;
    
    // 4. 电池状态
    msg.battery_q = data.battery.Q;
    msg.battery_v = data.battery.V;
    msg.battery_a = data.battery.A;
    msg.battery_t = data.battery.T;
    
    // 5. 运行状态
    msg.runMode = data.run.runMode;
    msg.y = data.run.Y;
    msg.h = data.run.H;
    msg.objSpeed = data.run.objSpeed;
    msg.objHeading = data.run.objHeading;
    msg.objPos[0] = data.run.objPos[0];
    msg.objPos[1] = data.run.objPos[1];
    msg.task_tepy = data.run.task_tepy;
    msg.task_sta = data.run.task_sta;
    msg.vision_state = data.run.vision_state;
    msg.mapping_state = data.run.mapping_state;
    
    // 6. 导航数据
    msg.ins_status = data.navi.ins_sta;
    msg.timestamp = data.navi.timeTick;
    
    msg.gyro_roll = data.navi.gyro_roll;
    msg.gyro_pitch = data.navi.gyro_pitch;
    msg.gyro_heading = data.navi.gyro_heading;
    
    msg.roll = data.navi.roll;
    msg.pitch = data.navi.pitch;
    msg.heading = data.navi.heading;
    
    msg.acc_x = data.navi.acc_x;
    msg.acc_y = data.navi.acc_y;
    msg.acc_z = data.navi.acc_z;
    
    msg.vel_x = data.navi.vel_x;
    msg.vel_y = data.navi.vel_y;
    msg.vel_z = data.navi.vel_z;
    
    msg.latitude = data.navi.latitude;
    msg.longitude = data.navi.longitude;
    msg.altitude = data.navi.altitude;
    
    msg.vel_n = data.navi.vel_n;
    msg.vel_e = data.navi.vel_e;
    msg.vel_d = data.navi.vel_d;
    
    msg.satellite_count = data.navi.satellite_count;
    msg.pos_status = data.navi.pos_sta;
    msg.heading_status = data.navi.heading_sta;
    
    msg.pos_std_lat = data.navi.position_std_lat;
    msg.pos_std_lon = data.navi.position_std_lon;
    
    // 7. 推进器状态
    msg.left_thruster_val = data.thruster.left_Val;
    msg.left_thruster_status = data.thruster.left_sta;
    msg.left_duty_cycle = data.thruster.left_duty_scaled;
    msg.left_rpm = data.thruster.left_rpm;
    msg.left_mos_temp = data.thruster.left_MOST;
    msg.left_total_current = data.thruster.left_total_current;
    msg.left_input_current = data.thruster.left_current_in;
    
    msg.right_thruster_val = data.thruster.right_Val;
    msg.right_thruster_status = data.thruster.right_sta;
    msg.right_duty_cycle = data.thruster.right_duty_scaled;
    msg.right_rpm = data.thruster.right_rpm;
    msg.right_mos_temp = data.thruster.right_MOST;
    msg.right_total_current = data.thruster.right_total_current;
    msg.right_input_current = data.thruster.right_current_in;
    
    // 8. 灯状态
    msg.LED = data.light.LED;
    msg.CautionLED = data.light.CautionLED;
    
    // 9. 码头信息
    msg.connect = data.dock.connect;
    msg.dock_ID = data.dock.dock_ID;
    
    msg.preset_positions0[0] = data.dock.presetPos[0][0];
    msg.preset_positions0[1] = data.dock.presetPos[0][1];
    
    msg.preset_positions1[0] = data.dock.presetPos[1][0];
    msg.preset_positions1[1] = data.dock.presetPos[1][1];
    
    msg.berth_status = data.dock.berth;
    msg.charge_status = data.dock.change;
}

//新任务
static void newTaskFileCallback(const std_msgs::Bool& msg)
{
    if(msg.data)
    {
        std::string file_path = ros::package::getPath("usvc_controller") + "/task/task_file.json";
        m_Control.getNewTaskFile(file_path);//获取新任务
    }
} 


//雷达滤波数据
static void lidarFilterCallback(const sensor_msgs::PointCloud2& msg)
{
    std::vector<Eigen::Vector3f> lidar;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *cloud);
    for(const auto& point : cloud->points) 
    {
        lidar.emplace_back(point.x, point.y, point.z);
    }

    m_Control.setLidarData(lidar);
} 

//导航欧拉角数据
static void naviEulerCallback(const geometry_msgs::Vector3& msg)
{
    NAVIEulerData_t euler;

    euler.roll = msg.x;
    euler.pitch = msg.y;
    euler.heading = msg.z;

    m_Control.setNAVIEulerData(euler);
} 

//导航GNSS数据
static void naviGnssCallback(const sensor_msgs::NavSatFix& msg)
{
    NAVIGnssData_t gnss;

    gnss.latitude = msg.latitude;
    gnss.longitude = msg.longitude;

    m_Control.setNAVIGnssData(gnss);
}
// planner 输出回调
static void plannerCmdCallback(const usvc_planner::GuidanceCmd& msg)
{
    Edge_ctrl ec;
    ec.objSpeed   = msg.obj_speed;
    ec.objHeading = msg.obj_heading;
    ec.isAvoiding = msg.is_avoiding;
    ec.reached    = msg.reached;
    m_Control.setEdgeCtrl(ec);
}


// planner 路径装载 + 启停触发器
static bool planner_loaded = false;
//输出回调 
static void visualCmdCallback(const usvc_planner::GuidanceCmd& msg)
{
    m_Control.setVisualTrackSpeed(msg.obj_speed);        //visual_track 减速速度
    m_Control.setVisualTrackHeading(msg.obj_heading);    //visual_track 航向(含偏移)
    m_Control.setVisualTrackReached(msg.reached);
}

//visual_track 检测框判据（进 CLEANING 用，与执行速度/使能解耦）
static void vtHasTargetCallback(const std_msgs::Bool::ConstPtr& msg)
{
    m_Control.setVisualTrackHasTarget(msg->data);
}

// 清理垃圾后，找最近路径点 
static int findNearestWaypoint(double current_lat, double current_lon,
                               const std::vector<Eigen::Vector2d>& original_path) 
{   int nearest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < original_path.size(); ++i) {
        double dlat = original_path[i].x() - current_lat;
        double dlon = original_path[i].y() - current_lon;
        double dist = dlat * dlat + dlon * dlon;
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

// 清理垃圾后，在轨迹跟踪路径中找最近路径点
static int findNearestTrackWaypoint(double current_lat, double current_lon,
                                    const std::map<int, std::vector<double>>& track_path)
{
    int nearest_idx = 0;
    double min_dist = 1e9;
    for (const auto& kv : track_path) {
        if (kv.second.size() < 2) continue;
        double dlat = kv.second[0] - current_lat;
        double dlon = kv.second[1] - current_lon;
        double dist = dlat * dlat + dlon * dlon;
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = kv.first;
        }
    }
    return nearest_idx;
}
//沿边任务启动时，生产沿边任务路径发布到“/usvc_planner/path”
static void plannerFeedIfNeeded(const RunCtrl_t& runCtrl)
{
    bool wantRun = (runCtrl.runMode    == RUN_MODE_ENUM::TASK)
                && (runCtrl.task_state == TASK_STATUS_ENUM::BEGIN);
    if(wantRun)
    {
        auto tl = m_Control.getTaskFile();
        bool has_ef = false;
        for(auto& ti : tl) if(ti.task_type == TASK_TYPE_ENUM::EDGE_FOLLOWING) { has_ef = true; break; }
        wantRun = has_ef;
    }

    if(wantRun && !planner_loaded)
    {
        planner_loaded = true;

        // 从任务文件取当前 EDGE_FOLLOWING 的航点，构造 LosPath
        auto task_list = m_Control.getTaskFile();
        usvc_planner::LosPath lp;
        lp.header.stamp = ros::Time::now();
        lp.header.frame_id = "usv";
        
        // 保存原路径 
        std::vector<Eigen::Vector2d> resume_path;
        bool water_left = false;
        
        for(size_t i = 0; i < task_list.size(); ++i)
        {
            if(task_list[i].task_type != TASK_TYPE_ENUM::EDGE_FOLLOWING) continue;
            water_left = task_list[i].water_on_left;
            for(const auto& kv : task_list[i].task_val)
            {
                geometry_msgs::PoseStamped ps;
                ps.header = lp.header;
                ps.pose.position.x = kv.second[0];   // lat
                ps.pose.position.y = kv.second[1];   // lon
                lp.path.poses.push_back(ps);
                 
                // 保存路径点 
                resume_path.emplace_back(kv.second[0], kv.second[1]);
            }
            break;
        }
        lp.water_is_on_left = water_left;
        edge_path_pub.publish(lp);
        
        // 保存路径到 Control 类
        m_Control.saveEdgePath(resume_path, water_left);

        //  启动 planner（沿边模式）
        usvc_planner::OtterCtrl oc;
        oc.run_mode        = 0;    // ALONG_EDGE
        oc.task_state      = 1;    // 开始
        oc.boundary_select = 0;
        edge_ctrl_pub.publish(oc);

        // 重置清理状态 
        m_Control.setEdgeSubState(Control::EDGE_FOLLOW);
        m_Control.setVisualTrackHasTarget(false);
        m_Control.setVisualTrackReached(false);

        ROS_INFO("[control_node] planner 已装载，保存 %zu 个路径点", resume_path.size());
    }
    else if(!wantRun && runCtrl.runMode != RUN_MODE_ENUM::TASK)
    {
        planner_loaded = false;
        m_Control.saveEdgePath({}, false);
        
        usvc_planner::OtterCtrl oc;
        oc.run_mode = 0; oc.task_state = 0; oc.boundary_select = 0;
        edge_ctrl_pub.publish(oc);
        
        // 重置状态 
        m_Control.setEdgeSubState(Control::EDGE_FOLLOW);
    }
}

//运动控制数据
static void runCtrlCallback(const usvc_controller::RunCtrl& msg)
{
    RunCtrl_t runCtrl;
    runctrl_msg_to_data(msg, runCtrl);

    m_Control.setRunCtrl(runCtrl);
    m_Control.changeRUN_MODE(runCtrl);//调整运动模式
    m_Control.changeTASK_STATUS(runCtrl);//调整任务状态

    new_runctrl_msg = true;

    plannerFeedIfNeeded(runCtrl);   // [planner] 任务进入/退出时装载路径并启停
}

//上位机连接状态回调
static void upperConnectCallback(const std_msgs::Bool::ConstPtr& msg)
{
    m_upper_connected = msg->data;
}

//状态数据
static void stateDataCallback(const usvc_controller::StateData& msg)
{
    StateData_t stateData;
    statedata_msg_to_data(msg, stateData);

    m_Control.setStateData(stateData);

    new_statedata_msg = true;
} 


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "control_node");
    
    ros::NodeHandle nh;

    Param_Sail_t Param_Sail;
    nh.param<double>("control/sail/SPEED", Param_Sail.SAIL_SPEED, 1.8);
    nh.param<double>("control/sail/Y", Param_Sail.SAIL_Y, 1.0);
    nh.param<double>("control/sail/TOLERANCERADIUS1", Param_Sail.SAIL_TOLERANCERADIUS[0], 1.0);
    nh.param<double>("control/sail/TOLERANCERADIUS2", Param_Sail.SAIL_TOLERANCERADIUS[1], 1.0);
    nh.param<double>("control/sail/LAT", Param_Sail.DEFAULT_POS[0], 1.0);
    nh.param<double>("control/sail/LON", Param_Sail.DEFAULT_POS[1], 1.0);
    m_Control.setParam_Sail(Param_Sail);

    Param_Back_t Param_Back;
    nh.param<double>("control/back/MAXSPEED", Param_Back.MAXSPEED, 1.0);
    nh.param<double>("control/back/MINSPEED", Param_Back.MINSPEED, 1.0);
    nh.param<double>("control/back/LOSDISTANCE", Param_Back.LOSDISTANCE, 1.0);
    nh.param<double>("control/back/OBSTACLE_INFLUENCE_DISTANCE", Param_Back.OBSTACLE_INFLUENCE_DISTANCE, 1.0);
    nh.param<double>("control/back/SAFE_DISTANCE", Param_Back.SAFE_DISTANCE, 1.0);
    nh.param<double>("control/back/REPULSIVE_GAIN", Param_Back.REPULSIVE_GAIN, 1.0);
    nh.param<double>("control/back/ATTRACTIVE_GAIN", Param_Back.ATTRACTIVE_GAIN, 1.0);
    nh.param<double>("control/back/SIGMA", Param_Back.SIGMA, 1.0);
    nh.param<double>("control/back/TOLERANCERADIUS", Param_Back.TOLERANCERADIUS, 1.0);
    nh.param<double>("control/back/Y", Param_Back.Y, 1.0);
    nh.param<double>("control/back/LAT", Param_Back.DEFAULT_POS[0], 1.0);
    nh.param<double>("control/back/LON", Param_Back.DEFAULT_POS[1], 1.0);
    m_Control.setParam_Back(Param_Back);

    Param_PathTracking_t Param_PathTracking;
    nh.param<double>("control/path/MAXSPEED",                    Param_PathTracking.MAXSPEED,                    1.0);
    nh.param<double>("control/path/MINSPEED",                    Param_PathTracking.MINSPEED,                    0.2);
    nh.param<double>("control/path/LOSDISTANCE",                 Param_PathTracking.LOSDISTANCE,                 5.0);
    nh.param<double>("control/path/OBSTACLE_INFLUENCE_DISTANCE", Param_PathTracking.OBSTACLE_INFLUENCE_DISTANCE, 20.0);
    nh.param<double>("control/path/SAFE_DISTANCE",               Param_PathTracking.SAFE_DISTANCE,               2.0);
    nh.param<double>("control/path/REPULSIVE_GAIN",              Param_PathTracking.REPULSIVE_GAIN,              0.6);
    nh.param<double>("control/path/ATTRACTIVE_GAIN",             Param_PathTracking.ATTRACTIVE_GAIN,             2.0);
    nh.param<double>("control/path/SIGMA",                       Param_PathTracking.SIGMA,                       3.5);
    nh.param<double>("control/path/TOLERANCERADIUS",             Param_PathTracking.TOLERANCERADIUS,             2.0);
    m_Control.setParam_PathTracking(Param_PathTracking);

    // enable_switches 冷启动恢复（launch rosparam 已加载 config.yaml 的 enable_switches 节）
    EnableSwitches_t EnableSwitches;
    int es_mapping = 0, es_lowbatt = 0, es_linklost = 0, es_vision = 0;
    nh.param<int>("enable_switches/mapping_en",       es_mapping,  0);
    nh.param<int>("enable_switches/back_lowbatt_en",  es_lowbatt,  0);
    nh.param<int>("enable_switches/back_linklost_en", es_linklost, 0);
    nh.param<int>("enable_switches/vision_en",        es_vision,   0);
    EnableSwitches.mapping_en       = (uint8_t)es_mapping;
    EnableSwitches.back_lowbatt_en  = (uint8_t)es_lowbatt;
    EnableSwitches.back_linklost_en = (uint8_t)es_linklost;
    EnableSwitches.vision_en        = (uint8_t)es_vision;
    m_Control.setEnableSwitches(EnableSwitches);

    Param_Charge_t Param_Charge;
    nh.param<double>("charge/LOW_BATTERY", Param_Charge.LOW_BATTERY, 20.0);
    nh.param<double>("charge/CHARGE_START", Param_Charge.CHARGE_START, 15.0);
    nh.param<double>("charge/CHARGE_STOP",  Param_Charge.CHARGE_STOP,  95.0);
    m_Control.setParam_Charge(Param_Charge);
        
    newTaskFile_sub = nh.subscribe("/upper/NewTaskFile", 10, newTaskFileCallback);

    lidar_filter_sub = nh.subscribe("/sensor/lidar_filter", 10, lidarFilterCallback);
    navi_euler_sub = nh.subscribe("/sensor/navi_euler", 10, naviEulerCallback);
    navi_gnss_sub = nh.subscribe("/sensor/navi_gnss", 10, naviGnssCallback);
    
    runCtrl_sub = nh.subscribe("/upper/RunCtrl", 10, runCtrlCallback);
    stateData_sub = nh.subscribe("/ship/StateData", 10, stateDataCallback);
    // enableSwitches 数据改由 service 回调进入，无需再自订阅广播话题（保留备查）
    upper_connect_sub = nh.subscribe("/upper/connect", 1, upperConnectCallback);

    RunCtrl_pub = nh.advertise<usvc_controller::RunCtrl>("/control/RunCtrl", 1); //
    StateData_pub = nh.advertise<usvc_controller::StateData>("/control/StateData", 1); //
    // 总线广播：latched 保证晚订阅功能包补收最近一次开关状态
    enableSwitches_bc_pub = nh.advertise<usvc_controller::EnableSwitches>("/upper/EnableSwitches", 1, true);
    // 启动即广播一次冷启动恢复的开关状态，供各功能包订阅到初始值
    {
        usvc_controller::EnableSwitches es_init;
        enableSwitches_data_to_msg(EnableSwitches, es_init);
        enableSwitches_bc_pub.publish(es_init);
    }

    // planner 订阅和发布（关键参数话题开启 latch = true，解决晚启动节点丢包）
    edge_cmd_sub       = nh.subscribe("/usvc_planner/cmd", 10, plannerCmdCallback);
    edge_path_pub      = nh.advertise<usvc_planner::LosPath>("/usvc_planner/path", 1);
    apf_update_pub     = nh.advertise<usvc_planner::APFUpdate>("/planner_node/apf_update", 1, true);
    tunable_update_pub = nh.advertise<usvc_planner::TunableParams>("/planner_node/tunable_update", 1, true);
    edge_ctrl_pub      = nh.advertise<usvc_planner::OtterCtrl>("/usvc_planner/ctrl", 1, true);

    // visual_track 与 yolo 订阅和发布
    visual_cmd_sub     = nh.subscribe("/visual_track/cmd", 10, visualCmdCallback);
    vt_has_target_sub  = nh.subscribe("/visual_track/has_target", 10, vtHasTargetCallback);
    visual_ctrl_pub    = nh.advertise<visual_track::VisualTrack>("/visual_track/ctrl", 1, true);
    visual_heading_pub = nh.advertise<std_msgs::Float32>("/visual_track/ref_heading", 1);
    vision_enable_pub  = nh.advertise<std_msgs::Bool>("/yolo/enable", 1, true);

    // 冷启动 1：广播 APF 初始真值（Param_PathTracking 已在上方加载）
    {
        usvc_planner::APFUpdate apf_init;
        apf_init.repulsive_gain     = Param_PathTracking.REPULSIVE_GAIN;
        apf_init.attractive_gain    = Param_PathTracking.ATTRACTIVE_GAIN;
        apf_init.safe_distance      = Param_PathTracking.SAFE_DISTANCE;
        apf_init.influence_distance = Param_PathTracking.OBSTACLE_INFLUENCE_DISTANCE;
        apf_init.sigma              = Param_PathTracking.SIGMA;
        apf_init.min_speed          = Param_PathTracking.MINSPEED;
        apf_init.max_speed          = Param_PathTracking.MAXSPEED;
        apf_update_pub.publish(apf_init);
    }

    // 冷启动 2：先赋显式安全初值，再从 config 读取 tunable 段并锁存广播
    {
        initDefaultTunableParams(g_tunable_params);
        nh.param<double>("tunable/target_speed",        g_tunable_params.target_speed,        g_tunable_params.target_speed);
        nh.param<double>("tunable/los_distance",        g_tunable_params.los_distance,        g_tunable_params.los_distance);
        nh.param<double>("tunable/waypoint_tolerance",  g_tunable_params.waypoint_tolerance,  g_tunable_params.waypoint_tolerance);
        nh.param<double>("tunable/min_speed_turn",      g_tunable_params.min_speed_turn,      g_tunable_params.min_speed_turn);
        nh.param<double>("tunable/adaptive_los_scale",  g_tunable_params.adaptive_los_scale,  g_tunable_params.adaptive_los_scale);
        nh.param<double>("tunable/safe_margin",         g_tunable_params.safe_margin,         g_tunable_params.safe_margin);
        nh.param<double>("tunable/boom_x_max",          g_tunable_params.boom_x_max,          g_tunable_params.boom_x_max);
        nh.param<double>("tunable/conf_thresh",         g_tunable_params.conf_thresh,         g_tunable_params.conf_thresh);
        nh.param<double>("tunable/max_heading_offset",  g_tunable_params.max_heading_offset,  g_tunable_params.max_heading_offset);
        nh.param<double>("tunable/approach_speed",      g_tunable_params.approach_speed,      g_tunable_params.approach_speed);
        nh.param<double>("tunable/heading_alpha",       g_tunable_params.heading_alpha,       g_tunable_params.heading_alpha);
        nh.param<double>("tunable/reached_area_thresh", g_tunable_params.reached_area_thresh, g_tunable_params.reached_area_thresh);
        nh.param<double>("tunable/bottom_thresh",       g_tunable_params.bottom_thresh,       g_tunable_params.bottom_thresh);
        nh.param<double>("tunable/slow_area",           g_tunable_params.slow_area,           g_tunable_params.slow_area);
        tunable_update_pub.publish(g_tunable_params);
    }

    // 冷启动 3：广播初始停机/关闭状态，保证各子节点初始状态一致
    {
        usvc_planner::OtterCtrl oc_init;
        oc_init.run_mode = 0; oc_init.task_state = 0; oc_init.boundary_select = 0;
        edge_ctrl_pub.publish(oc_init);

        visual_track::VisualTrack vm_init;
        vm_init.run_mode = 0; vm_init.task_state = 0; vm_init.task_type = 0;
        visual_ctrl_pub.publish(vm_init);

        std_msgs::Bool yolo_init;
        yolo_init.data = false;
        vision_enable_pub.publish(yolo_init);
    }

    sail_param_service = nh.advertiseService("sail/update_params", sailParamsCallback);
    back_param_service = nh.advertiseService("back/update_params", backParamsCallback);
    path_param_service = nh.advertiseService("path/update_params", pathParamsCallback);
    enable_switches_service = nh.advertiseService("enable_switches/update_params", enableSwitchesParamsCallback);
    tunable_param_service  = nh.advertiseService("tunable/update_params", tunableParamsCallback);
    //电量服务
    charge_param_service = nh.advertiseService("charge/update_params", chargeParamsCallback);
    
    ros::Rate rate(100); // 100Hz
    while (ros::ok()) 
    {
        //===========================================================
        // 自动返航触发检测(低电量 / 断联)
        //===========================================================
        m_Control.checkAutoReturn(m_Control.getEnableSwitches(), m_upper_connected);
        //===========================================================

        RUN_MODE_ENUM nowRUN_MODE = m_Control.getRUN_MODE();
        RUN_MODE_ENUM nextRUN_MODE = nowRUN_MODE;
        switch(nowRUN_MODE)
        {
            case RUN_MODE_ENUM::NORMAL: //一般
            {
                nextRUN_MODE = m_Control.executeNormal();

                //控制数据发布
                if(new_runctrl_msg)
                {
                    new_runctrl_msg = false;

                    RunCtrl_t run_ctrl = m_Control.getRunCtrl();
                    usvc_controller::RunCtrl msg;
                    runctrl_data_to_msg(run_ctrl, msg);
                    RunCtrl_pub.publish(msg);
                }

                break;
            }

            case RUN_MODE_ENUM::KEEP: //keep
            {
                nextRUN_MODE = m_Control.executeKeep();
                
                //控制数据发布
                if(new_runctrl_msg)
                {
                    new_runctrl_msg = false;

                    RunCtrl_t run_ctrl = m_Control.getRunCtrl();
                    usvc_controller::RunCtrl msg;
                    runctrl_data_to_msg(run_ctrl, msg);
                    RunCtrl_pub.publish(msg);
                }
                break;
            }
            
            case RUN_MODE_ENUM::SAIL: //出航
            {
                ROS_INFO("SAIL");
                nextRUN_MODE = m_Control.executeSail();

                //控制数据发布
                RunCtrl_t run_ctrl = m_Control.getRunCtrl();
                usvc_controller::RunCtrl msg;
                runctrl_data_to_msg(run_ctrl, msg);
                RunCtrl_pub.publish(msg);

                break;
            }

            case RUN_MODE_ENUM::BACK: //返航
            {
                ROS_INFO("BACK");
                nextRUN_MODE = m_Control.executeBack();

                //控制数据发布
                RunCtrl_t run_ctrl = m_Control.getRunCtrl();
                usvc_controller::RunCtrl msg;
                runctrl_data_to_msg(run_ctrl, msg);
                RunCtrl_pub.publish(msg);

                break;
            }

            case RUN_MODE_ENUM::TASK: //任务
            {
                ROS_INFO("TASK");
                nextRUN_MODE = m_Control.executeTask();

                // =====================================================
                // [MODIFIED-01] 识别+推流(yolo) 与 跟踪(visual_track) 视为同一功能，
                // 由同一把锁 vt_want（PATH_TRACKING && vision_en）一块启动/关闭。
                // 开启时先放行【上游】yolo(识别+推流,产 /yolo/detections)，再放行【下游】跟踪。
                // 两话题分属两个独立节点，控制逻辑这里只留一个状态标记，避免啰嗦。
                // =====================================================
                bool vt_want = m_Control.getCurrentTaskType() == TASK_TYPE_ENUM::PATH_TRACKING
                               && m_Control.getEnableSwitches().vision_en;
                static bool vt_enabled_ = false;
                if (vt_want && !vt_enabled_) {
                    std_msgs::Bool  ymsg; ymsg.data = true;  vision_enable_pub.publish(ymsg);   // 识别+推流 开
                    visual_track::VisualTrack vm;
                    vm.run_mode = 1; vm.task_state = 1;
                    vm.task_type = TASK_TYPE_ENUM::PATH_TRACKING;
                    visual_ctrl_pub.publish(vm);                                              // 跟踪 开
                    vt_enabled_ = true;
                    ROS_INFO("[vservo] PATH_TRACKING+vision_en：识别+推流+跟踪 使能");
                } else if (!vt_want && vt_enabled_) {
                    std_msgs::Bool  ymsg; ymsg.data = false; vision_enable_pub.publish(ymsg);  // 识别 关
                    visual_track::VisualTrack vm;
                    vm.run_mode = 0; vm.task_state = 0;
                    vm.task_type = TASK_TYPE_ENUM::PATH_TRACKING;
                    visual_ctrl_pub.publish(vm);                                              // 跟踪 关
                    vt_enabled_ = false;
                    ROS_INFO("[vservo] 退出 PATH_TRACKING/vision_en：识别+推流+跟踪 关闭");
                }

                // 轨迹跟踪时，把目标航向作为 ref_heading 发给 visual_track
                if (m_Control.getCurrentTaskType() == TASK_TYPE_ENUM::PATH_TRACKING) {
                    std_msgs::Float32 heading_msg;
                    heading_msg.data = m_Control.getPathTrackingHeading();
                    visual_heading_pub.publish(heading_msg);
                }

                Control::EdgeSubState sub_state = m_Control.getEdgeSubState();
                static Control::EdgeSubState prev_sub_state = Control::EDGE_FOLLOW;

            
                int task_type = m_Control.getCurrentTaskType();

                if (sub_state == Control::CLEAN_DONE && prev_sub_state != Control::CLEAN_DONE){

                    if (task_type == TASK_TYPE_ENUM::PATH_TRACKING) {
                        // 轨迹跟踪：找最近航点，恢复 path_tracking_ctrl
                        NAVIGnssData_t gnss = m_Control.getNAVIGnssData();
                        const auto& track_path = m_Control.getTrackPath();
                        if (!track_path.empty()) {
                            int nearest_id = findNearestTrackWaypoint(gnss.latitude, gnss.longitude, track_path);
                            m_Control.setPathId(nearest_id);
                            m_Control.setEdgeSubState(Control::EDGE_FOLLOW);
                            ROS_INFO("[PATH_RESUME] 从路径点 %d 恢复轨迹跟踪", nearest_id);
                        } else {
                            ROS_WARN("[PATH_RESUME] 无保存路径，无法恢复");
                        }
                    }
                    else if (task_type == TASK_TYPE_ENUM::EDGE_FOLLOWING) {
                        // 沿边：重新发布路径给 planner
                        NAVIGnssData_t gnss = m_Control.getNAVIGnssData();
                        const auto& resume_path = m_Control.getResumePath();
                        bool water_left = m_Control.getResumeWaterLeft();
                        if (!resume_path.empty()) {
                            int nearest_idx = findNearestWaypoint(gnss.latitude, gnss.longitude, resume_path);
                            usvc_planner::LosPath lp;
                            lp.header.stamp = ros::Time::now();
                            lp.header.frame_id = "usv";
                            lp.water_is_on_left = water_left;
                            for (size_t i = nearest_idx; i < resume_path.size(); ++i) {
                                geometry_msgs::PoseStamped ps;
                                ps.header = lp.header;
                                ps.pose.position.x = resume_path[i].x();
                                ps.pose.position.y = resume_path[i].y();
                                lp.path.poses.push_back(ps);
                            }
                            edge_path_pub.publish(lp);
                            usvc_planner::OtterCtrl oc;
                            oc.run_mode = 0;
                            oc.task_state = 1;
                            oc.boundary_select = 0;
                            edge_ctrl_pub.publish(oc);
                            m_Control.setEdgeSubState(Control::EDGE_FOLLOW);
                            ROS_INFO("[EDGE_RESUME] 从航点 %d/%zu 恢复沿边", nearest_idx, resume_path.size());
                        } else {
                                ROS_WARN("[EDGE_RESUME] 无保存路径，无法恢复");
                            
                        }
                    }
                }
                prev_sub_state = sub_state;

                //控制数据发布
                RunCtrl_t run_ctrl = m_Control.getRunCtrl();
                usvc_controller::RunCtrl msg;
                runctrl_data_to_msg(run_ctrl, msg);
                RunCtrl_pub.publish(msg);
                //轨迹任务结束，关闭目标识别推流+目标跟踪
                if (nextRUN_MODE != RUN_MODE_ENUM::TASK && vt_enabled_) {
                    std_msgs::Bool ym;
                    ym.data = false;
                    vision_enable_pub.publish(ym);
                    visual_track::VisualTrack vm;
                    vm.run_mode = 0;
                    vm.task_state = 0;
                    vm.task_type = TASK_TYPE_ENUM::PATH_TRACKING;
                    visual_ctrl_pub.publish(vm);
                    vt_enabled_ = false;
                    ROS_INFO("[vservo] 任务结束/退出TASK：识别+推流+跟踪 关闭");
                }

                break;
            }
            
            case RUN_MODE_ENUM::USER: //用户自定义
            {
                nextRUN_MODE = m_Control.executeUser();
                
                //控制数据发布
                if(new_runctrl_msg)
                {
                    new_runctrl_msg = false;

                    RunCtrl_t run_ctrl = m_Control.getRunCtrl();
                    usvc_controller::RunCtrl msg;
                    runctrl_data_to_msg(run_ctrl, msg);
                    RunCtrl_pub.publish(msg);
                }

                break;
            }
            
            default:
                break;
        }
        
        m_Control.setRUN_MODE(nextRUN_MODE); //更新运动模式


        //状态数据发布
        if(new_statedata_msg)
        {
            new_statedata_msg = false;

            usvc_controller::StateData msg;
            StateData_t state_data = m_Control.getStateData();
            statedata_data_to_msg(state_data, msg);

            StateData_pub.publish(msg);
        }

        ros::spinOnce();    
        rate.sleep();
    }

    return 0;
}