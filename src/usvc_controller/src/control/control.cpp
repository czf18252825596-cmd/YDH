#include "control.h"

#include <ros/package.h>
#include <ros/ros.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/asio.hpp> // 用于IP地址解析
#include <iostream>
#include <cstdio>
#include "../control_calc_tool/control_calc_tool.h"


Control::Control()
{

}

// 析构函数
Control::~Control()
{

}

//任务文件
bool Control::getNewTaskFile(const std::string& file_path)
{
    try {
        std::cerr << "control.cpp 接收到新的任务文件完成" << std::endl;
        std::vector<CtlTask_t> task; //

        // 清空输出参数
        task.clear();
        
        // 读取 JSON 文件到 ptree
        boost::property_tree::ptree pt;
        boost::property_tree::read_json(file_path, pt);

        // 获取 tasks 数组节点
        boost::property_tree::ptree tasks_pt = pt.get_child("tasks");
        
        // 遍历 JSON 中的每个任务节点
        for (const auto &task_item : tasks_pt)
        {
            // 获取当前任务的 ptree
            const boost::property_tree::ptree &single_task_pt = task_item.second;

            // 获取任务类型
            std::string task_type_str = single_task_pt.get<std::string>("task_type");
            
            if (task_type_str.empty()) 
            {
                std::cerr << "control.cpp  任务类型为空" << std::endl;
                continue;
            }
            
            int task_type = std::stoi(task_type_str);

            // 获取任务值节点
            boost::property_tree::ptree task_val_pt = single_task_pt.get_child("task_val");

            // 创建当前任务的参数map
            std::map<int, std::vector<double>> current_task_values;

            // 遍历当前任务的参数
            for (const auto &param_item : task_val_pt)
            {
                // 处理键名，去除前导零
                std::string key_str = param_item.first;
                int id = std::stoi(key_str);  // "01" -> 1, "02" -> 2 等
                
                const boost::property_tree::ptree &param_pt = param_item.second;

                std::vector<double> values;
                
                switch(task_type) 
                {
                    case TASK_TYPE_ENUM::SAIL_T:
                    case TASK_TYPE_ENUM::BACK_T:
                    {
                        // 解析 has_dock
                        std::string has_dock_str = param_pt.get<std::string>("has_dock", "false");
                        bool has_dock = (has_dock_str == "true" || has_dock_str == "1" || has_dock_str == "yes");
                        values.push_back(has_dock ? 1.0 : 0.0);

                        // 解析 IP 地址 - 处理空字符串情况
                        std::string ip_str = param_pt.get<std::string>("dock_ip", "0.0.0.0");
                        
                        uint8_t ip_array[4] = {0, 0, 0, 0};
                        
                        if (!ip_str.empty() && ip_str != "0.0.0.0") {
                            try {
                                boost::asio::ip::address_v4 addr = boost::asio::ip::address_v4::from_string(ip_str);
                                uint32_t ip_int = addr.to_ulong();
                                
                                ip_array[0] = (ip_int >> 24) & 0xFF;
                                ip_array[1] = (ip_int >> 16) & 0xFF;
                                ip_array[2] = (ip_int >> 8) & 0xFF;
                                ip_array[3] = ip_int & 0xFF;
                            } catch (const std::exception& e) {
                                std::cerr << "control.cpp IP地址解析失败: " << ip_str << ", 使用默认值" << std::endl;
                                // 使用默认的 0.0.0.0
                            }
                        }
                        
                        // 将IP地址的4个字节作为double存入
                        for (int i = 0; i < 4; i++) 
                        {
                            values.push_back(static_cast<double>(ip_array[i]));
                        }

                        // 解析端口
                        std::string port_str = param_pt.get<std::string>("dock_port", "0");
                        double port = 0.0;
                        
                        if (!port_str.empty()) {
                            try {
                                port = static_cast<double>(std::stoi(port_str));
                            } catch (const std::exception& e) {
                                std::cerr << "control.cpp 端口解析失败: " << port_str << ", 使用默认值" << std::endl;
                            }
                        }
                        values.push_back(port);
                        break;
                    }
                    case TASK_TYPE_ENUM::PATH_TRACKING:
                    case TASK_TYPE_ENUM::EDGE_FOLLOWING:  // 轨迹和沿边的航点格式相同，均为 [lat, lon]。
                    {
                        std::string lat_str = param_pt.get<std::string>("lat", "0.0");
                        std::string lon_str = param_pt.get<std::string>("lon", "0.0");

                        double lat = 0.0, lon = 0.0;
                       try {
                            if (!lat_str.empty()) {
                                lat = std::stod(lat_str);
                            }
                        } catch (const std::exception& e) {
                            std::cerr << "control.cpp 纬度解析失败: " << lat_str << ", 错误: " << e.what() << std::endl;
                        }
                        
                        try {
                            if (!lon_str.empty()) {
                                lon = std::stod(lon_str);
                            }
                        } catch (const std::exception& e) {
                            std::cerr << "control.cpp 经度解析失败: " << lon_str << ", 错误: " << e.what() << std::endl;
                        }
                    
                        values.push_back(lat);   // [0] = lat
                        values.push_back(lon);   // [1] = lon
                        break;
                    }
                    default:
                    {
                        std::cerr << "control.cpp 未知的任务类型: " << task_type << std::endl;
                        break;
                    }
                }
                // 将参数添加到当前任务
                current_task_values[id] = values;

            }  
         
            // 沿边模式：水向是任务级属性，存到结构体字段，不混入航点 map
            bool water_on_left = false;
            if(task_type == TASK_TYPE_ENUM::EDGE_FOLLOWING)
            {
                std::string wl = single_task_pt.get<std::string>("water_on_left", "0");
                try { water_on_left = (std::stod(wl) != 0.0); } catch(...) {}
            }    
            // 将任务类型和对应的参数对添加到结果中
            CtlTask_t t;
            t.task_type = task_type;
            t.water_on_left = water_on_left;
            t.task_val = current_task_values;
            task.push_back(t); 
        }
        
        // // 打印最终解析结果
        // std::cout << "\n========== 解析完成 ==========" << std::endl;
        // std::cout << "总共解析了 " << task.size() << " 个任务" << std::endl;
        // for (size_t i = 0; i < task.size(); ++i) {
        //     std::cout << "任务 " << i+1 << ": 类型=" << task[i].task_type 
        //               << ", 参数组数=" << task[i].task_val.size() << std::endl;
        // }
        // std::cout << "==============================\n" << std::endl;
        
        std::cerr << "control.cpp 新任务解析完成" << std::endl;
        TASK_STATUS_ENUM status = TASK_STATUS_ENUM::STOP;
        setTASK_STATUS(status);
        setTaskFile(task);

        return true;
        
    } catch (const boost::property_tree::json_parser::json_parser_error& e) {
        std::cerr << "control.cpp JSON解析错误: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "control.cpp 读取任务文件错误: " << e.what() << std::endl;
        return false;
    }
}

void Control::setTaskFile(std::vector<CtlTask_t> task_file)
{
    std::lock_guard<std::mutex> lock(m_task_fileMutex);
    m_task_file = task_file;
}

std::vector<CtlTask_t>  Control::getTaskFile(void) const
{
    std::lock_guard<std::mutex> lock(m_task_fileMutex);
    return m_task_file;
}


//出航参数
void Control::setParam_Sail(Param_Sail_t param)
{
    std::lock_guard<std::mutex> lock(m_Param_SailMutex);
    m_Param_Sail = param;
}

Param_Sail_t Control::getParam_Sail()
{
    std::lock_guard<std::mutex> lock(m_Param_SailMutex);
    return m_Param_Sail;
}


//返航参数
void Control::setParam_Back(Param_Back_t param)
{
    std::lock_guard<std::mutex> lock(m_Param_BackMutex);
    m_Param_Back = param;
}

Param_Back_t Control::getParam_Back()
{
    std::lock_guard<std::mutex> lock(m_Param_BackMutex);
    return m_Param_Back;
}


//轨迹跟踪参数
void Control::setParam_PathTracking(Param_PathTracking_t param)
{
    std::lock_guard<std::mutex> lock(m_Param_PathTrackingMutex);
    m_Param_PathTracking = param;
}

Param_PathTracking_t Control::getParam_PathTracking()
{
    std::lock_guard<std::mutex> lock(m_Param_PathTrackingMutex);
    return m_Param_PathTracking;
}

void Control::setParam_Charge(Param_Charge_t param)
{
    std::lock_guard<std::mutex> lock(m_Param_ChargeMutex);
    m_Param_Charge = param;
}

Param_Charge_t Control::getParam_Charge()
{
    std::lock_guard<std::mutex> lock(m_Param_ChargeMutex);
    return m_Param_Charge;
}


//运动模式
void Control::changeRUN_MODE(RunCtrl_t ctrl)
{
    std::lock_guard<std::mutex> lock(m_RUN_MODEMutex);
    RUN_MODE_ENUM now_RUN_MODE = m_RUN_MODE;
    switch(ctrl.runMode)
    {
        case 0x00:
            m_RUN_MODE = RUN_MODE_ENUM::NORMAL;
            break;
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x04:
            m_RUN_MODE = RUN_MODE_ENUM::KEEP;
            break;
        case 0x05:
            m_RUN_MODE = RUN_MODE_ENUM::SAIL;
            break;
        case 0x06:
            m_RUN_MODE = RUN_MODE_ENUM::BACK;
            break;
        case 0x07:
            m_RUN_MODE = RUN_MODE_ENUM::TASK;
            break;
        case 0x08:
            m_RUN_MODE = RUN_MODE_ENUM::USER;
            break;
        default:
            m_RUN_MODE = RUN_MODE_ENUM::NORMAL;
            break;
    }

    if(now_RUN_MODE != m_RUN_MODE)
    {
        task_index = 0;
        sail_step = 0; //出航步骤
        back_step = 0; //返航步骤
        path_id = 0;   //路径id
    }
}

void Control::setRUN_MODE(RUN_MODE_ENUM run)
{
    std::lock_guard<std::mutex> lock(m_RUN_MODEMutex);
    m_RUN_MODE = run;
}

RUN_MODE_ENUM Control::getRUN_MODE()
{
    std::lock_guard<std::mutex> lock(m_RUN_MODEMutex);
    return m_RUN_MODE;
}


//任务状态
void Control::changeTASK_STATUS(RunCtrl_t ctrl)
{
    std::lock_guard<std::mutex> lock(m_TASK_STATUSMutex);
    switch(ctrl.task_state)
    {
        case 0x00:
            m_TASK_STATUS = TASK_STATUS_ENUM::STOP;
            break;
        case 0x01:
            m_TASK_STATUS = TASK_STATUS_ENUM::BEGIN;
            break;
        case 0x02:
            m_TASK_STATUS = TASK_STATUS_ENUM::PAUSE;
            break;
        default:
            m_TASK_STATUS = TASK_STATUS_ENUM::STOP;
            break;
    }
}

void Control::setTASK_STATUS(TASK_STATUS_ENUM sta)
{
    std::lock_guard<std::mutex> lock(m_TASK_STATUSMutex);
    m_TASK_STATUS = sta;
}

TASK_STATUS_ENUM Control::getTASK_STATUS()
{
    std::lock_guard<std::mutex> lock(m_TASK_STATUSMutex);
    return m_TASK_STATUS;
}


//运动控制数据
void Control::setRunCtrl(RunCtrl_t ctrl)
{
    std::lock_guard<std::mutex> lock(m_RunCtrlMutex);
    m_RunCtrl = ctrl;
}

RunCtrl_t Control::getRunCtrl(void)
{
    std::lock_guard<std::mutex> lock(m_RunCtrlMutex);
    return m_RunCtrl;
}


//使能开关数据
void Control::setEnableSwitches(EnableSwitches_t enables)
{
    std::lock_guard<std::mutex> lock(m_EnableSwitchesMutex);
    m_EnableSwitches = enables;
}

EnableSwitches_t Control::getEnableSwitches(void)
{
    std::lock_guard<std::mutex> lock(m_EnableSwitchesMutex);
    return m_EnableSwitches;
}

void Control::checkAutoReturn(const EnableSwitches_t& enables, bool upper_connected)
{
    // 低电量返航（3秒防抖）
    static ros::Time low_batt_since;
    static bool      low_batt_armed = false;
    bool low_batt_cond = (enables.back_lowbatt_en == 1) &&
                         (getStateData().battery.Q <= getParam_Charge().LOW_BATTERY);
    if (low_batt_cond)
    {
        if (!low_batt_armed) { low_batt_since = ros::Time::now(); low_batt_armed = true; }
        if ((ros::Time::now() - low_batt_since).toSec() >= 3.0)
        {
            if (getRUN_MODE() != RUN_MODE_ENUM::BACK)
            {
                ROS_WARN("[control] 低电量(%.1f%%<=%.1f%%) 自动返航",
                         getStateData().battery.Q, getParam_Charge().LOW_BATTERY);
                setRUN_MODE(RUN_MODE_ENUM::BACK);
            }
        }
    }
    else
    {
        low_batt_armed = false;
    }

    // 断联返航（TCP断开，立即触发）
    if (enables.back_linklost_en == 1 && !upper_connected)
    {
        if (getRUN_MODE() != RUN_MODE_ENUM::BACK)
        {
            ROS_WARN("[control] 上位机失联(TCP断开) 自动返航");
            setRUN_MODE(RUN_MODE_ENUM::BACK);
        }
    }
}


//状态数据
void Control::setStateData(StateData_t data)
{
    std::lock_guard<std::mutex> lock(m_StateDataMutex);
    m_StateData = data;
}

StateData_t Control::getStateData(void)
{
    std::lock_guard<std::mutex> lock(m_StateDataMutex);
    return m_StateData;
}


//导航数据
void Control::setNAVIEulerData(NAVIEulerData_t data)
{
    std::lock_guard<std::mutex> lock(m_NAVIEulerDataMutex);
    m_NAVIEulerData = data;
}
    
NAVIEulerData_t Control::getNAVIEulerData(void)
{
    std::lock_guard<std::mutex> lock(m_NAVIEulerDataMutex);
    return m_NAVIEulerData;
}

//导航数据
void Control::setNAVIGnssData(NAVIGnssData_t data)
{
    std::lock_guard<std::mutex> lock(m_NAVIGnssDataMutex);
    m_NAVIGnssData = data;
}
    
NAVIGnssData_t Control::getNAVIGnssData(void)
{
    std::lock_guard<std::mutex> lock(m_NAVIGnssDataMutex);
    return m_NAVIGnssData;
}


//雷达数据
void Control::setLidarData(std::vector<Eigen::Vector3f> data)
{
    std::lock_guard<std::mutex> lock(m_LidarDataMutex);
    m_LidarData = data;
}

std::vector<Eigen::Vector3f> Control::getLidarData(void)
{
    std::lock_guard<std::mutex> lock(m_LidarDataMutex);
    return m_LidarData;
}
//沿边
void Control::setEdgeCtrl(const Edge_ctrl& c)
{
    std::lock_guard<std::mutex> lock(m_EdgeCtrlMutex);
    m_Edge_ctrl = c;
}

Edge_ctrl Control::getEdgeCtrl(void)
{
    std::lock_guard<std::mutex> lock(m_EdgeCtrlMutex);
    return m_Edge_ctrl;
}

//一般控制
 Ctrl_Result_t Control::normal_ctrl()
 {
    Ctrl_Result_t Ctrl_Result = {0};

    Ctrl_Result.ctrl_finished = true;

    return Ctrl_Result;
 }

//keep控制
Ctrl_Result_t Control::keep_ctrl()
{
    Ctrl_Result_t Ctrl_Result = {0};

    Ctrl_Result.ctrl_finished = true;

    return Ctrl_Result;
}

//出航
Ctrl_Result_t Control::sail_ctrl(const Param_Sail_t& param, const NAVIEulerData_t& euler, const NAVIGnssData_t& gnss, const std::vector<Eigen::Vector3f>& lidar, const Dock_Info_t& dock_info)
{
    Ctrl_Result_t Ctrl_Result = {0};

    if(dock_info.has_dock)
    {
        Ctrl_Result.dock_ip[0] = dock_info.dock_ip[0];
        Ctrl_Result.dock_ip[1] = dock_info.dock_ip[1];
        Ctrl_Result.dock_ip[2] = dock_info.dock_ip[2];
        Ctrl_Result.dock_ip[3] = dock_info.dock_ip[3];

        Ctrl_Result.dock_port = dock_info.dock_port;

        if(dock_info.connect)
        {
            //有码头情况
            double current_lat = static_cast<double>(gnss.latitude);
            double current_lon = static_cast<double>(gnss.longitude);
            float  current_heading = static_cast<float>(euler.heading);

            //第一阶段等待船解锁后前推，前推到预设点1进入第二阶段
            if(sail_step == 0)
            {
                if(dock_info.berth == AWAITING_ARRIVAL || dock_info.berth == PREPARING_DEPART) //解锁状态
                {
                    double target_lat = static_cast<double>(dock_info.presetPos[0][0]);
                    double target_lon = static_cast<double>(dock_info.presetPos[0][1]);
                    double target_distance = ControlCalaTool::get_target_distance(current_lat, current_lon, target_lat, target_lon);

                    if(target_distance <= param.SAIL_TOLERANCERADIUS[0])
                    {
                        sail_step = 1;
                    }
                    else
                    {
                        float object_Y, object_H;

                        object_Y = abs(param.SAIL_Y);
                        object_H = 0;

                        Ctrl_Result.ctrl_finished = false;
                        Ctrl_Result.ctrl_mode = 0x00;
                        Ctrl_Result.Y = object_Y;
                        Ctrl_Result.H = object_H;
                        Ctrl_Result.objHeading = 0;
                        Ctrl_Result.objSpeed = 0;
                        Ctrl_Result.isAvoiding = false;
                    }   
                }
            }

            //第二阶段到达预设点2，完成出航流程
            if(sail_step == 1)
            {
                double target_lat = static_cast<double>(dock_info.presetPos[1][0]);
                double target_lon = static_cast<double>(dock_info.presetPos[1][1]);
                double target_distance = ControlCalaTool::get_target_distance(current_lat, current_lon, target_lat, target_lon);

                if(target_distance <= param.SAIL_TOLERANCERADIUS[1])
                {
                    sail_step = 0;

                    Ctrl_Result.ctrl_finished = true;
                    Ctrl_Result.ctrl_mode = 0x00;
                    Ctrl_Result.Y = 0;
                    Ctrl_Result.H = 0;
                    Ctrl_Result.objHeading = 0;
                    Ctrl_Result.objSpeed = 0;
                    Ctrl_Result.isAvoiding = false;
                }
                else
                {
                    float object_speed, object_heading;//期望速度与航向

                    object_speed = param.SAIL_SPEED;
                    object_heading = ControlCalaTool::get_target_azimuth(current_lat, current_lon, target_lat, target_lon);

                    Ctrl_Result.ctrl_finished = false;
                    Ctrl_Result.ctrl_mode = 0x01;
                    Ctrl_Result.Y = 0;
                    Ctrl_Result.H = 0;
                    Ctrl_Result.objHeading = object_heading;
                    Ctrl_Result.objSpeed = object_speed;
                    Ctrl_Result.isAvoiding = false;
                }   
            }
        }
    }
    else
    {
        //无码头情况
        Ctrl_Result.dock_ip[0] = 0;
        Ctrl_Result.dock_ip[1] = 0;
        Ctrl_Result.dock_ip[2] = 0;
        Ctrl_Result.dock_ip[3] = 0;

        Ctrl_Result.dock_port = 0;
        //直接出航结束
        Ctrl_Result.ctrl_finished = true;
        Ctrl_Result.ctrl_mode = 0x00;
        Ctrl_Result.Y = 0;
        Ctrl_Result.H = 0;
        Ctrl_Result.objHeading = 0;
        Ctrl_Result.objSpeed = 0;
        Ctrl_Result.isAvoiding = false;
    }

    return Ctrl_Result;
}

//返航
Ctrl_Result_t Control::back_ctrl(const Param_Back_t& param, const NAVIEulerData_t& euler, const NAVIGnssData_t& gnss, const std::vector<Eigen::Vector3f>& lidar, const Dock_Info_t& dock_info)
{
    Ctrl_Result_t Ctrl_Result = {0};

    bool isAvoiding = false; //避障标志位

    if(dock_info.has_dock)
    {
        Ctrl_Result.dock_ip[0] = dock_info.dock_ip[0];
        Ctrl_Result.dock_ip[1] = dock_info.dock_ip[1];
        Ctrl_Result.dock_ip[2] = dock_info.dock_ip[2];
        Ctrl_Result.dock_ip[3] = dock_info.dock_ip[3];

        Ctrl_Result.dock_port = dock_info.dock_port;
        if(dock_info.connect)
        {
            double current_lat = static_cast<double>(gnss.latitude);
            double current_lon = static_cast<double>(gnss.longitude);
            float  current_heading = static_cast<float>(euler.heading);

            //有码头情况
            //第一阶段，避障到预设点2，进入第二阶段
            if(back_step == 0)
            {
                double target_lat = static_cast<double>(dock_info.presetPos[1][0]);
                double target_lon = static_cast<double>(dock_info.presetPos[1][1]);
                double target_distance = ControlCalaTool::get_target_distance(current_lat, current_lon, target_lat, target_lon);
            
                float object_speed, object_heading;//期望速度与航向

                //APF避障处理
                float target_speed   = ControlCalaTool::smooth_acceleration(target_distance, param.MAXSPEED, param.TOLERANCERADIUS*3);
                float target_heading = ControlCalaTool::get_target_azimuth(current_lat, current_lon, target_lat, target_lon);

                std::vector<Eigen::Vector3f> lidar_data = lidar;
                if(!lidar_data.empty())
                {
                    float heading_diff = target_heading - current_heading;
                    if (heading_diff > 180.0f) heading_diff -= 360.0f;
                    if (heading_diff < -180.0f) heading_diff += 360.0f;

                    float relative_angle = -heading_diff;

                    //处理引力
                    Eigen::Vector2f attractive_force(std::cos(ControlCalaTool::to_radians(relative_angle)), std::sin(ControlCalaTool::to_radians(relative_angle)));//引力
                    attractive_force = attractive_force * param.ATTRACTIVE_GAIN;

                    //处理斥力
                    Eigen::Vector2f repulsive_force(0.0f, 0.0f);//斥力
                    float min_obstacle_distance = std::numeric_limits<float>::max(); //最小距离，初始为最大值
                    for(const auto & obs_point : lidar_data)
                    {
                        float obs_dis = obs_point.head<2>().norm();
                        if(obs_dis < param.OBSTACLE_INFLUENCE_DISTANCE) //处理在影响范围内的障碍
                        {
                            isAvoiding = true; //正在避障

                            min_obstacle_distance = std::min(min_obstacle_distance, obs_dis);
                            Eigen::Vector2f dir(obs_point.x(), obs_point.y());
                            dir = -dir.normalized(); //障碍物的方向
                            repulsive_force += param.REPULSIVE_GAIN * exp(-pow(obs_dis - param.SAFE_DISTANCE, 2) / (2 * pow(param.SIGMA, 2))) * dir;
                        }
                    }

                    //计算合力,并计算目标速度与航向
                    if(min_obstacle_distance < param.OBSTACLE_INFLUENCE_DISTANCE && min_obstacle_distance > param.SAFE_DISTANCE) //处理影响范围内并在安全范围外的障碍
                    {
                        Eigen::Vector2f total_force = attractive_force + repulsive_force;

                        float force_angle  = std::atan2(total_force.y(), total_force.x()) * 180.0 / M_PI;
                        target_heading  = std::fmod(current_heading - force_angle + 360.0f, 360.0f);

                        //极小值处理
                        float force_magnitude = total_force.norm();
                        static int small_force_count = 0;
                        if(force_magnitude < 0.3f)
                        {
                            small_force_count++;
                            printf("[极小值警告] 合力=%.2f, 连续次数=%d, 障碍距离=%.2f\n", 
                                force_magnitude, small_force_count, min_obstacle_distance);
                            
                            if(small_force_count > 10)
                            {
                                printf("[紧急] 可能陷入极小值！需要干预！\n");
                                // 这里可以添加临时处理：添加随机扰动
                                target_heading = std::fmod(target_heading + 30.0f, 360.0f);
                                small_force_count = 0;
                            }
                        }
                        else
                        {
                            if(small_force_count > 0)
                            {
                                printf("[恢复正常] 合力=%.2f\n", force_magnitude);
                            }
                            small_force_count = 0;
                        }
                        
                        float speed_factor = (min_obstacle_distance - param.SAFE_DISTANCE) / (param.OBSTACLE_INFLUENCE_DISTANCE - param.SAFE_DISTANCE);
                        target_speed = param.MINSPEED + (target_speed - param.MINSPEED) * speed_factor;
                    }
                }

                //期望速度与航向
                object_speed   = target_speed;
                object_heading = target_heading;

                if(target_distance <= param.TOLERANCERADIUS)
                {
                    back_step = 1;
                }
                else
                {
                    Ctrl_Result.ctrl_finished = false;
                    Ctrl_Result.ctrl_mode = 0x01;
                    Ctrl_Result.Y = 0;
                    Ctrl_Result.H = 0;
                    Ctrl_Result.objHeading = object_heading;
                    Ctrl_Result.objSpeed = object_heading;
                    Ctrl_Result.isAvoiding = isAvoiding;
                }
            }

            //第二阶段，头尾转换（避障）到预设点1，进入第三阶段
            if(back_step == 1)
            {
                double target_lat = static_cast<double>(dock_info.presetPos[0][0]);
                double target_lon = static_cast<double>(dock_info.presetPos[0][1]);
                double target_distance = ControlCalaTool::get_target_distance(current_lat, current_lon, target_lat, target_lon);
                
                float object_speed, object_heading;//期望速度与航向

                //APF避障处理 - 第二阶段需要倒车，所以速度和航向计算需要调整
                // 倒车时目标航向是当前位置到目标点的方向加上180度（船尾方向）
                float target_heading_to_point = ControlCalaTool::get_target_azimuth(current_lat, current_lon, target_lat, target_lon);
                
                // 倒车时船尾作为"船头"，所以目标航向需要旋转180度
                float target_heading_reverse = fmod(target_heading_to_point + 180.0f, 360.0f);
                
                // 倒车速度需要取负值
                float target_speed = -ControlCalaTool::smooth_acceleration(target_distance, 
                                                                        param.MAXSPEED, 
                                                                        param.TOLERANCERADIUS*3);

                std::vector<Eigen::Vector3f> lidar_data = lidar;
                if(!lidar_data.empty())
                {
                    // 倒车时需要考虑激光雷达数据的坐标系转换
                    // 因为船尾作为船头，所以激光雷达数据需要旋转180度
                    std::vector<Eigen::Vector3f> rotated_lidar_data;
                    for(const auto & point : lidar_data)
                    {
                        //将激光雷达点旋转180度（绕Z轴旋转180度）
                        float rotated_x = -point.x();
                        float rotated_y = -point.y();
                        rotated_lidar_data.push_back(Eigen::Vector3f(rotated_x, rotated_y, point.z()));
                    }

                    // 计算航向差 - 使用旋转后的视角
                    float heading_diff = target_heading_reverse - current_heading;
                    if (heading_diff > 180.0f) heading_diff -= 360.0f;
                    if (heading_diff < -180.0f) heading_diff += 360.0f;

                    float relative_angle = -heading_diff;

                    // 处理引力 - 目标方向是旋转后的目标航向
                    Eigen::Vector2f attractive_force(std::cos(ControlCalaTool::to_radians(relative_angle)), 
                                                    std::sin(ControlCalaTool::to_radians(relative_angle)));
                    attractive_force = attractive_force * param.ATTRACTIVE_GAIN;

                    // 处理斥力 - 使用旋转后的激光雷达数据
                    Eigen::Vector2f repulsive_force(0.0f, 0.0f);
                    float min_obstacle_distance = std::numeric_limits<float>::max();
                    
                    for(const auto & obs_point : rotated_lidar_data)
                    {
                        float obs_dis = obs_point.head<2>().norm();
                        if(obs_dis < param.OBSTACLE_INFLUENCE_DISTANCE)
                        {
                            isAvoiding = true; //正在避障

                            min_obstacle_distance = std::min(min_obstacle_distance, obs_dis);
                            Eigen::Vector2f dir(obs_point.x(), obs_point.y());
                            dir = -dir.normalized();
                            repulsive_force += param.REPULSIVE_GAIN * exp(-pow(obs_dis - param.SAFE_DISTANCE, 2) / (2 * pow(param.SIGMA, 2))) * dir;
                        }
                    }

                    // 计算合力
                    if(min_obstacle_distance < param.OBSTACLE_INFLUENCE_DISTANCE && min_obstacle_distance > param.SAFE_DISTANCE)
                    {
                        Eigen::Vector2f total_force = attractive_force + repulsive_force;

                        float force_angle = std::atan2(total_force.y(), total_force.x()) * 180.0 / M_PI;
                        float adjusted_heading = std::fmod(current_heading - force_angle + 360.0f, 360.0f);
                        
                        // 航向调整为避障航向
                        object_heading = adjusted_heading;
                        
                        // 速度调整 - 倒车时保持负速度
                        float speed_factor = (min_obstacle_distance - param.SAFE_DISTANCE) / (param.OBSTACLE_INFLUENCE_DISTANCE - param.SAFE_DISTANCE);
                        target_speed = -param.MINSPEED + (target_speed + param.MINSPEED) * speed_factor;
                    }
                    else
                    {
                        object_heading = target_heading_reverse;
                    }
                }
                else
                {
                    object_heading = target_heading_reverse;
                }

                object_speed = target_speed;

                if(target_distance <= param.TOLERANCERADIUS)
                {
                    back_step = 2;
                }
                else
                {
                    Ctrl_Result.ctrl_finished = false;
                    Ctrl_Result.ctrl_mode = 0x01;
                    Ctrl_Result.Y = 0;
                    Ctrl_Result.H = 0;
                    Ctrl_Result.objHeading = object_heading;
                    Ctrl_Result.objSpeed = object_heading;
                    Ctrl_Result.isAvoiding = isAvoiding;
                }
            }

            //第三阶段，持续倒推，直到锁定后停止倒退，返航流程执行完
            if(back_step == 2)
            {
                if(dock_info.berth == PREPARING_DOCK)
                {
                    Ctrl_Result.ctrl_finished = false;
                    Ctrl_Result.ctrl_mode = 0x00;
                    Ctrl_Result.Y = -abs(param.Y);
                    Ctrl_Result.H = 0;
                    Ctrl_Result.objHeading = 0;
                    Ctrl_Result.objSpeed = 0;
                    Ctrl_Result.isAvoiding = false;
                }

                if(dock_info.berth == DOCKED_SECURED)
                {
                    back_step = 0;

                    Ctrl_Result.ctrl_finished = true;
                    Ctrl_Result.ctrl_mode = 0x00;
                    Ctrl_Result.Y = 0;
                    Ctrl_Result.H = 0;
                    Ctrl_Result.objHeading = 0;
                    Ctrl_Result.objSpeed = 0;
                    Ctrl_Result.isAvoiding = false;
                }
            }
        }
    }
    else
    {
        //无码头情况
        Ctrl_Result.dock_ip[0] = 0;
        Ctrl_Result.dock_ip[1] = 0;
        Ctrl_Result.dock_ip[2] = 0;
        Ctrl_Result.dock_ip[3] = 0;

        Ctrl_Result.dock_port = 0;
        
        //避障到达预存点
        double current_lat = static_cast<double>(gnss.latitude);
        double current_lon = static_cast<double>(gnss.longitude);
        float  current_heading = static_cast<float>(euler.heading);

        double target_lat = static_cast<double>(param.DEFAULT_POS[0]);
        double target_lon = static_cast<double>(param.DEFAULT_POS[1]);
        double target_distance = ControlCalaTool::get_target_distance(current_lat, current_lon, target_lat, target_lon);
        printf("-------------------------------\r\n \
        current_lat: %lf\r\n \
        current_lon: %lf\r\n \
        current_heading: %f\r\n \
        target_lat: %lf\r\n \
        target_lon: %lf\r\n \
        target_distance: %lf\r\n",
        current_lat,
        current_lon,
        current_heading,
        target_lat,
        target_lon,
        target_distance);
    
        float object_speed, object_heading;//期望速度与航向

        //APF避障处理
        float target_speed   = ControlCalaTool::smooth_acceleration(target_distance, param.MAXSPEED, param.TOLERANCERADIUS*3);
        float target_heading = ControlCalaTool::get_target_azimuth(current_lat, current_lon, target_lat, target_lon);

        std::vector<Eigen::Vector3f> lidar_data = lidar;
        if(!lidar_data.empty())
        {
            float heading_diff = target_heading - current_heading;
            if (heading_diff > 180.0f) heading_diff -= 360.0f;
            if (heading_diff < -180.0f) heading_diff += 360.0f;

            float relative_angle = -heading_diff;

            //处理引力
            Eigen::Vector2f attractive_force(std::cos(ControlCalaTool::to_radians(relative_angle)), std::sin(ControlCalaTool::to_radians(relative_angle)));//引力
            attractive_force = attractive_force * param.ATTRACTIVE_GAIN;

            //处理斥力
            Eigen::Vector2f repulsive_force(0.0f, 0.0f);//斥力
            float min_obstacle_distance = std::numeric_limits<float>::max(); //最小距离，初始为最大值
            for(const auto & obs_point : lidar_data)
            {
                float obs_dis = obs_point.head<2>().norm();
                if(obs_dis < param.OBSTACLE_INFLUENCE_DISTANCE) //处理在影响范围内的障碍
                {
                    isAvoiding = true; //正在避障 

                    min_obstacle_distance = std::min(min_obstacle_distance, obs_dis);
                    Eigen::Vector2f dir(obs_point.x(), obs_point.y());
                    dir = -dir.normalized(); //障碍物的方向
                    repulsive_force += param.REPULSIVE_GAIN * exp(-pow(obs_dis - param.SAFE_DISTANCE, 2) / (2 * pow(param.SIGMA, 2))) * dir;
                }
            }

            //计算合力,并计算目标速度与航向
            if(min_obstacle_distance < param.OBSTACLE_INFLUENCE_DISTANCE && min_obstacle_distance > param.SAFE_DISTANCE) //处理影响范围内并在安全范围外的障碍
            {
                Eigen::Vector2f total_force = attractive_force + repulsive_force;

                float force_angle  = std::atan2(total_force.y(), total_force.x()) * 180.0 / M_PI;
                printf("force_angle: %f\r\n", force_angle);
                target_heading  = std::fmod(current_heading - force_angle + 360.0f, 360.0f);

                //极小值处理
                float force_magnitude = total_force.norm();
                static int small_force_count = 0;
                if(force_magnitude < 0.3f)
                {
                    small_force_count++;
                    printf("[极小值警告] 合力=%.2f, 连续次数=%d, 障碍距离=%.2f\n", 
                        force_magnitude, small_force_count, min_obstacle_distance);
                    
                    if(small_force_count > 10)
                    {
                        printf("[紧急] 可能陷入极小值！需要干预！\n");
                        // 这里可以添加临时处理：添加随机扰动
                        target_heading = std::fmod(target_heading + 30.0f, 360.0f);
                        small_force_count = 0;
                    }
                }
                else
                {
                    if(small_force_count > 0)
                    {
                        printf("[恢复正常] 合力=%.2f\n", force_magnitude);
                    }
                    small_force_count = 0;
                }

                float speed_factor = (min_obstacle_distance - param.SAFE_DISTANCE) / (param.OBSTACLE_INFLUENCE_DISTANCE - param.SAFE_DISTANCE);
                target_speed = param.MINSPEED + (target_speed - param.MINSPEED) * speed_factor;
            }
        }

        //期望速度与航向
        object_speed   = target_speed;
        object_heading = target_heading;

        printf("object_speed: %f\r\n \
                object_heading: %f\r\n",
        object_speed,
        object_heading);

        if(target_distance <= param.TOLERANCERADIUS)
        {
            Ctrl_Result.ctrl_finished = true;
            Ctrl_Result.ctrl_mode = 0x00;
            Ctrl_Result.Y = 0;
            Ctrl_Result.H = 0;
            Ctrl_Result.objHeading = 0;
            Ctrl_Result.objSpeed = 0;
            Ctrl_Result.isAvoiding = false;
        }
        else
        {
            Ctrl_Result.ctrl_finished = false;
            Ctrl_Result.ctrl_mode = 0x01;
            Ctrl_Result.Y = 0;
            Ctrl_Result.H = 0;
            Ctrl_Result.objHeading = object_heading;
            Ctrl_Result.objSpeed = object_speed;
            Ctrl_Result.isAvoiding = isAvoiding;
        }
    }

    return Ctrl_Result;
}


//轨迹跟踪
Ctrl_Result_t Control::path_tracking_ctrl(const Param_PathTracking_t& param, const NAVIEulerData_t& euler, const NAVIGnssData_t& gnss, const std::vector<Eigen::Vector3f>& lidar, const std::map<int, std::vector<double>>& path)
{
    Ctrl_Result_t Ctrl_Result = {0};

    bool isAvoiding = false; //避障标志位

    Ctrl_Result.dock_ip[0] = 0;
    Ctrl_Result.dock_ip[1] = 0;
    Ctrl_Result.dock_ip[2] = 0;
    Ctrl_Result.dock_ip[3] = 0;

    Ctrl_Result.dock_port = 0;

    //路径规划及避障处理
    double current_lat = static_cast<double>(gnss.latitude);
    double current_lon = static_cast<double>(gnss.longitude);
    float  current_heading = static_cast<float>(euler.heading);
    double target_lat = static_cast<double>(path.at(path_id+1)[0]);
    double target_lon = static_cast<double>(path.at(path_id+1)[1]);
    double target_distance = ControlCalaTool::get_target_distance(current_lat, current_lon, target_lat, target_lon);

    printf("-------------------------------\r\n \
        current_lat: %lf\r\n \
        current_lon: %lf\r\n \
        current_heading: %f\r\n \
        target_lat: %lf\r\n \
        target_lon: %lf\r\n \
        target_distance: %lf\r\n \
        path_id: %d\r\n",
        current_lat,
        current_lon,
        current_heading,
        target_lat,
        target_lon,
        target_distance,
        path_id);

    float object_speed, object_heading;//期望速度与航向

    if(path_id == 0) //第一个点
    {
        //APF避障处理
        float target_speed   = ControlCalaTool::smooth_acceleration(target_distance, param.MAXSPEED, param.TOLERANCERADIUS*3);
        float target_heading = ControlCalaTool::get_target_azimuth(current_lat, current_lon, target_lat, target_lon);

        std::vector<Eigen::Vector3f> lidar_data = lidar;
        if(!lidar_data.empty())
        {
            float heading_diff = target_heading - current_heading;
            if (heading_diff > 180.0f) heading_diff -= 360.0f;
            if (heading_diff < -180.0f) heading_diff += 360.0f;

            float relative_angle = -heading_diff;

            //处理引力
            Eigen::Vector2f attractive_force(std::cos(ControlCalaTool::to_radians(relative_angle)), std::sin(ControlCalaTool::to_radians(relative_angle)));//引力
            attractive_force = attractive_force * param.ATTRACTIVE_GAIN;

            //处理斥力
            Eigen::Vector2f repulsive_force(0.0f, 0.0f);//斥力
            float min_obstacle_distance = std::numeric_limits<float>::max(); //最小距离，初始为最大值
            for(const auto & obs_point : lidar_data)
            {
                float obs_dis = obs_point.head<2>().norm();
                if(obs_dis < param.OBSTACLE_INFLUENCE_DISTANCE) //处理在影响范围内的障碍
                {
                    isAvoiding = true; //正在避障 

                    min_obstacle_distance = std::min(min_obstacle_distance, obs_dis);
                    Eigen::Vector2f dir(obs_point.x(), obs_point.y());
                    dir = -dir.normalized(); //障碍物的方向
                    repulsive_force += param.REPULSIVE_GAIN * exp(-pow(obs_dis - param.SAFE_DISTANCE, 2) / (2 * pow(param.SIGMA, 2))) * dir;
                }
            }

            //计算合力,并计算目标速度与航向
            if(min_obstacle_distance < param.OBSTACLE_INFLUENCE_DISTANCE && min_obstacle_distance > param.SAFE_DISTANCE) //处理影响范围内并在安全范围外的障碍
            {
                Eigen::Vector2f total_force = attractive_force + repulsive_force;
                printf("total_force: %f\r\n", total_force.norm());
                float force_angle  = std::atan2(total_force.y(), total_force.x()) * 180.0 / M_PI;
                printf("force_angle: %f\r\n", force_angle);
                target_heading  = std::fmod(current_heading - force_angle + 360.0f, 360.0f);

                //极小值处理
                float force_magnitude = total_force.norm();
                static int small_force_count = 0;
                if(force_magnitude < 0.3f)
                {
                    small_force_count++;
                    printf("[极小值警告] 合力=%.2f, 连续次数=%d, 障碍距离=%.2f\n", 
                        force_magnitude, small_force_count, min_obstacle_distance);
                    
                    if(small_force_count > 10)
                    {
                        printf("[紧急] 可能陷入极小值！需要干预！\n");
                        // 这里可以添加临时处理：添加随机扰动
                        target_heading = std::fmod(target_heading + 30.0f, 360.0f);
                        small_force_count = 0;
                    }
                }
                else
                {
                    if(small_force_count > 0)
                    {
                        printf("[恢复正常] 合力=%.2f\n", force_magnitude);
                    }
                    small_force_count = 0;
                }


                float speed_factor = (min_obstacle_distance - param.SAFE_DISTANCE) / (param.OBSTACLE_INFLUENCE_DISTANCE - param.SAFE_DISTANCE);
                target_speed = param.MINSPEED + (target_speed - param.MINSPEED) * speed_factor;
            }
        }

        //期望速度与航向
        object_speed   = target_speed;
        object_heading = target_heading;
        
        // //期望速度与航向
        // object_speed = ControlCalaTool::smooth_acceleration(target_distance, param.MAXSPEED, 10.0);
        // object_heading = ControlCalaTool::get_target_azimuth(current_lat, current_lon, target_lat, target_lon);
    }
    else
    {
        //上一个点
        double prev_lat = static_cast<double>(path.at(path_id)[0]);
        double prev_lon = static_cast<double>(path.at(path_id)[1]);

        //计算LOS引导航向
        float los_heading; //引导航向

        float heading_AB = ControlCalaTool::get_target_azimuth(prev_lat, prev_lon, target_lat, target_lon); 
        double proj_lat, proj_lon; //投影位置
        ControlCalaTool::project_point(prev_lat, prev_lon, target_lat, target_lon, current_lat, current_lon, proj_lat, proj_lon);
        double los_lat, los_lon; //引导位置
        ControlCalaTool::calculateDestination(proj_lat, proj_lon, heading_AB, param.LOSDISTANCE, los_lat, los_lon);

        los_heading = ControlCalaTool::get_target_azimuth(current_lat, current_lon, los_lat, los_lon);

        //APF避障处理
        float target_speed   = ControlCalaTool::smooth_acceleration(target_distance, param.MAXSPEED, param.TOLERANCERADIUS*3);
        float target_heading = los_heading;

            
        std::vector<Eigen::Vector3f> lidar_data = lidar;
        if(!lidar_data.empty())
        {
            float heading_diff = target_heading - current_heading;
            if (heading_diff > 180.0f) heading_diff -= 360.0f;
            if (heading_diff < -180.0f) heading_diff += 360.0f;

            float relative_angle = -heading_diff;

            //处理引力
            Eigen::Vector2f attractive_force(std::cos(ControlCalaTool::to_radians(relative_angle)), std::sin(ControlCalaTool::to_radians(relative_angle)));//引力
            attractive_force = attractive_force * param.ATTRACTIVE_GAIN;

            //处理斥力
            Eigen::Vector2f repulsive_force(0.0f, 0.0f);//斥力
            float min_obstacle_distance = std::numeric_limits<float>::max(); //最小距离，初始为最大值
            for(const auto & obs_point : lidar_data)
            {
                float obs_dis = obs_point.head<2>().norm();
                if(obs_dis < param.OBSTACLE_INFLUENCE_DISTANCE) //处理在影响范围内的障碍
                {
                    isAvoiding = true;

                    min_obstacle_distance = std::min(min_obstacle_distance, obs_dis);
                    Eigen::Vector2f dir(obs_point.x(), obs_point.y());
                    dir = -dir.normalized(); //障碍物的方向
                    repulsive_force += param.REPULSIVE_GAIN * exp(-pow(obs_dis - param.SAFE_DISTANCE, 2) / (2 * pow(param.SIGMA, 2))) * dir;
                }
            }

            //计算合力,并计算目标速度与航向
            if(min_obstacle_distance < param.OBSTACLE_INFLUENCE_DISTANCE && min_obstacle_distance > param.SAFE_DISTANCE) //处理影响范围内并在安全范围外的障碍
            {
                Eigen::Vector2f total_force = attractive_force + repulsive_force;
                printf("total_force: %f\r\n", total_force.norm());
                float force_angle  = std::atan2(total_force.y(), total_force.x()) * 180.0 / M_PI;
                printf("force_angle: %f\r\n", force_angle);
                target_heading  = std::fmod(current_heading - force_angle + 360.0f, 360.0f);

                //极小值处理
                float force_magnitude = total_force.norm();
                static int small_force_count = 0;
                if(force_magnitude < 0.3f)
                {
                    small_force_count++;
                    printf("[极小值警告] 合力=%.2f, 连续次数=%d, 障碍距离=%.2f\n", 
                        force_magnitude, small_force_count, min_obstacle_distance);
                    
                    if(small_force_count > 10)
                    {
                        printf("[紧急] 可能陷入极小值！需要干预！\n");
                        // 这里可以添加临时处理：添加随机扰动
                        target_heading = std::fmod(target_heading + 30.0f, 360.0f);
                        small_force_count = 0;
                    }
                }
                else
                {
                    if(small_force_count > 0)
                    {
                        printf("[恢复正常] 合力=%.2f\n", force_magnitude);
                    }
                    small_force_count = 0;
                }

                float speed_factor = (min_obstacle_distance - param.SAFE_DISTANCE) / (param.OBSTACLE_INFLUENCE_DISTANCE - param.SAFE_DISTANCE);
                target_speed = param.MINSPEED + (target_speed - param.MINSPEED) * speed_factor;
            }
        }

        //期望速度与航向
        object_speed   = target_speed;
        object_heading = target_heading;
    }

    printf("object_speed: %f\r\n \
        object_heading: %f\r\n",
        object_speed,
        object_heading);


    //调整部分控制数据
    Ctrl_Result.ctrl_finished = false;
    Ctrl_Result.ctrl_mode = 0x01;
    Ctrl_Result.Y = 0;
    Ctrl_Result.H = 0;
    Ctrl_Result.objHeading = object_heading;
    Ctrl_Result.objSpeed = object_speed;
    Ctrl_Result.isAvoiding = isAvoiding;
    
    if(target_distance <= param.TOLERANCERADIUS)
    {
        path_id++;

        if(path_id >= path.size()) //完成所有路径点，任务执行完成
        {
            path_id = 0;

            //调整部分控制数据
            Ctrl_Result.ctrl_finished = true;
            Ctrl_Result.ctrl_mode = 0x00;
            Ctrl_Result.Y = 0;
            Ctrl_Result.H = 0;
            Ctrl_Result.objHeading = 0;
            Ctrl_Result.objSpeed = 0;
            Ctrl_Result.isAvoiding = false;
        }
    }

    return Ctrl_Result;
}

//用户控制
Ctrl_Result_t Control::user_ctrl()
{
    Ctrl_Result_t Ctrl_Result = {0};

    Ctrl_Result.ctrl_finished = true;

    return Ctrl_Result;
}


//一般
RUN_MODE_ENUM Control::executeNormal()
{
    RUN_MODE_ENUM mode = RUN_MODE_ENUM::NORMAL;

    return mode;
}

//keep
RUN_MODE_ENUM Control::executeKeep()
{
    RUN_MODE_ENUM mode = RUN_MODE_ENUM::KEEP;

    return mode;
}

//出航
RUN_MODE_ENUM Control::executeSail()
{
    RUN_MODE_ENUM mode = RUN_MODE_ENUM::NORMAL;

    Param_Sail_t param = getParam_Sail();

    NAVIEulerData_t euler = getNAVIEulerData();
    NAVIGnssData_t gnss = getNAVIGnssData();
    std::vector<Eigen::Vector3f> lidar = getLidarData();

    RunCtrl_t ctrl = getRunCtrl();
    StateData_t data = getStateData();

    Dock_Info_t dock_info;
    dock_info.has_dock = ctrl.has_dock;
    dock_info.dock_ip[0] = ctrl.dock_ip[0];
    dock_info.dock_ip[1] = ctrl.dock_ip[1];
    dock_info.dock_ip[2] = ctrl.dock_ip[2];
    dock_info.dock_ip[3] = ctrl.dock_ip[3];
    dock_info.dock_port = ctrl.dock_port;

    dock_info.connect = data.dock.connect;
    dock_info.dock_ID = data.dock.dock_ID;
    dock_info.presetPos[0][0] = data.dock.presetPos[0][0];
    dock_info.presetPos[0][1] = data.dock.presetPos[0][1];
    dock_info.presetPos[1][0] = data.dock.presetPos[1][0];
    dock_info.presetPos[1][1] = data.dock.presetPos[1][1];
    dock_info.berth = data.dock.berth;
    dock_info.change = data.dock.change;

    Ctrl_Result_t Ctrl_Result = sail_ctrl(param, euler, gnss, lidar, dock_info);
    if(Ctrl_Result.ctrl_finished)
    {
        mode = RUN_MODE_ENUM::NORMAL;
        ctrl.runMode = 0;

        ctrl.ctrl_mode = Ctrl_Result.ctrl_mode;
        ctrl.Y = Ctrl_Result.Y;
        ctrl.H = Ctrl_Result.H;
        ctrl.objHeading = Ctrl_Result.objHeading;
        ctrl.objSpeed = Ctrl_Result.objSpeed;

        ctrl.isAvoiding = Ctrl_Result.isAvoiding;

        ctrl.dock_ip[0] = Ctrl_Result.dock_ip[0];
        ctrl.dock_ip[1] = Ctrl_Result.dock_ip[1];
        ctrl.dock_ip[2] = Ctrl_Result.dock_ip[2];
        ctrl.dock_ip[3] = Ctrl_Result.dock_ip[3];

        ctrl.dock_port = Ctrl_Result.dock_port;
        
    }
    else
    {
        mode = RUN_MODE_ENUM::SAIL;
        ctrl.runMode = 5;

        ctrl.ctrl_mode = Ctrl_Result.ctrl_mode;
        ctrl.Y = Ctrl_Result.Y;
        ctrl.H = Ctrl_Result.H;
        ctrl.objHeading = Ctrl_Result.objHeading;
        ctrl.objSpeed = Ctrl_Result.objSpeed;
        
        ctrl.isAvoiding = Ctrl_Result.isAvoiding;

        ctrl.dock_ip[0] = Ctrl_Result.dock_ip[0];
        ctrl.dock_ip[1] = Ctrl_Result.dock_ip[1];
        ctrl.dock_ip[2] = Ctrl_Result.dock_ip[2];
        ctrl.dock_ip[3] = Ctrl_Result.dock_ip[3];

        ctrl.dock_port = Ctrl_Result.dock_port;
    }

    setRunCtrl(ctrl);
    setStateData(data);

    return mode;
}
    
//返航
RUN_MODE_ENUM Control::executeBack()
{
    RUN_MODE_ENUM mode = RUN_MODE_ENUM::NORMAL;

    Param_Back_t param = getParam_Back();

    NAVIEulerData_t euler = getNAVIEulerData();
    NAVIGnssData_t gnss = getNAVIGnssData();
    std::vector<Eigen::Vector3f> lidar = getLidarData();

    RunCtrl_t ctrl = getRunCtrl();
    StateData_t data = getStateData();

    Dock_Info_t dock_info;
    dock_info.has_dock = ctrl.has_dock;
    dock_info.dock_ip[0] = ctrl.dock_ip[0];
    dock_info.dock_ip[1] = ctrl.dock_ip[1];
    dock_info.dock_ip[2] = ctrl.dock_ip[2];
    dock_info.dock_ip[3] = ctrl.dock_ip[3];
    dock_info.dock_port = ctrl.dock_port;

    dock_info.connect = data.dock.connect;
    dock_info.dock_ID = data.dock.dock_ID;
    dock_info.presetPos[0][0] = data.dock.presetPos[0][0];
    dock_info.presetPos[0][1] = data.dock.presetPos[0][1];
    dock_info.presetPos[1][0] = data.dock.presetPos[1][0];
    dock_info.presetPos[1][1] = data.dock.presetPos[1][1];
    dock_info.berth = data.dock.berth;
    dock_info.change = data.dock.change;


    Ctrl_Result_t Ctrl_Result = back_ctrl(param, euler, gnss, lidar, dock_info);
    if(Ctrl_Result.ctrl_finished)
    {
        mode = RUN_MODE_ENUM::NORMAL;
        ctrl.runMode = 0;

        ctrl.ctrl_mode = Ctrl_Result.ctrl_mode;
        ctrl.Y = Ctrl_Result.Y;
        ctrl.H = Ctrl_Result.H;
        ctrl.objHeading = Ctrl_Result.objHeading;
        ctrl.objSpeed = Ctrl_Result.objSpeed;

        ctrl.isAvoiding = Ctrl_Result.isAvoiding;

        ctrl.dock_ip[0] = Ctrl_Result.dock_ip[0];
        ctrl.dock_ip[1] = Ctrl_Result.dock_ip[1];
        ctrl.dock_ip[2] = Ctrl_Result.dock_ip[2];
        ctrl.dock_ip[3] = Ctrl_Result.dock_ip[3];

        ctrl.dock_port = Ctrl_Result.dock_port;
        
    }
    else
    {
        mode = RUN_MODE_ENUM::BACK;
        ctrl.runMode = 6;

        ctrl.ctrl_mode = Ctrl_Result.ctrl_mode;
        ctrl.Y = Ctrl_Result.Y;
        ctrl.H = Ctrl_Result.H;
        ctrl.objHeading = Ctrl_Result.objHeading;
        ctrl.objSpeed = Ctrl_Result.objSpeed;
        
        ctrl.isAvoiding = Ctrl_Result.isAvoiding;

        ctrl.dock_ip[0] = Ctrl_Result.dock_ip[0];
        ctrl.dock_ip[1] = Ctrl_Result.dock_ip[1];
        ctrl.dock_ip[2] = Ctrl_Result.dock_ip[2];
        ctrl.dock_ip[3] = Ctrl_Result.dock_ip[3];

        ctrl.dock_port = Ctrl_Result.dock_port;
    }

    setRunCtrl(ctrl);
    setStateData(data);

    return mode;
}

//任务
RUN_MODE_ENUM Control::executeTask()
{
    RUN_MODE_ENUM mode = RUN_MODE_ENUM::NORMAL;

    NAVIEulerData_t euler = getNAVIEulerData();
    NAVIGnssData_t gnss = getNAVIGnssData();
    std::vector<Eigen::Vector3f> lidar = getLidarData();
    std::vector<CtlTask_t> task = getTaskFile();

    RunCtrl_t ctrl = getRunCtrl();
    StateData_t data = getStateData();

    TASK_STATUS_ENUM status = getTASK_STATUS();
    switch(status)
    {
        case TASK_STATUS_ENUM::BEGIN:
        {
            if(task.empty())
            {
                status = TASK_STATUS_ENUM::STOP;

                ctrl.runMode = 0x00;
                ctrl.H = 0;
                ctrl.Y = 0;

                data.run.task_tepy = TASK_TYPE_ENUM::EMPTY;
                data.run.task_sta = TASK_STATUS_ENUM::STOP;

                task_index = 0;
                sail_step = 0; //出航步骤
                back_step = 0; //返航步骤
                path_id = 0;   //路径id

                return RUN_MODE_ENUM::NORMAL;
            }

            
            auto& current_task = task[task_index];
            int task_type = current_task.task_type;
            auto& task_val = current_task.task_val; 

            Ctrl_Result_t Ctrl_Result = {0};
            switch(task_type)
            {
                case TASK_TYPE_ENUM::SAIL_T:
                {
                    Param_Sail_t param = getParam_Sail();

                    Dock_Info_t dock_info;
                    dock_info.has_dock = static_cast<bool>(task_val.at(1)[0]);
                    dock_info.dock_ip[0] = static_cast<uint8_t>(task_val.at(1)[1]);
                    dock_info.dock_ip[1] = static_cast<uint8_t>(task_val.at(1)[2]);
                    dock_info.dock_ip[2] = static_cast<uint8_t>(task_val.at(1)[3]);
                    dock_info.dock_ip[3] = static_cast<uint8_t>(task_val.at(1)[4]);
                    dock_info.dock_port = static_cast<uint16_t>(task_val.at(1)[5]);

                    dock_info.connect = data.dock.connect;
                    dock_info.dock_ID = data.dock.dock_ID;
                    dock_info.presetPos[0][0] = data.dock.presetPos[0][0];
                    dock_info.presetPos[0][1] = data.dock.presetPos[0][1];
                    dock_info.presetPos[1][0] = data.dock.presetPos[1][0];
                    dock_info.presetPos[1][1] = data.dock.presetPos[1][1];
                    dock_info.berth = data.dock.berth;
                    dock_info.change = data.dock.change;

                    Ctrl_Result = sail_ctrl(param, euler, gnss, lidar, dock_info);
                    if(Ctrl_Result.ctrl_finished)
                        task_index++; 

                    break;
                }

                case TASK_TYPE_ENUM::BACK_T:
                {
                    printf("task BACK_T\r\n");
                    Param_Back_t param = getParam_Back();

                    Dock_Info_t dock_info;
                    dock_info.has_dock = static_cast<bool>(task_val.at(1)[0]);
                    dock_info.dock_ip[0] = static_cast<uint8_t>(task_val.at(1)[1]);
                    dock_info.dock_ip[1] = static_cast<uint8_t>(task_val.at(1)[2]);
                    dock_info.dock_ip[2] = static_cast<uint8_t>(task_val.at(1)[3]);
                    dock_info.dock_ip[3] = static_cast<uint8_t>(task_val.at(1)[4]);
                    dock_info.dock_port = static_cast<uint16_t>(task_val.at(1)[5]);

                    dock_info.connect = data.dock.connect;
                    dock_info.dock_ID = data.dock.dock_ID;
                    dock_info.presetPos[0][0] = data.dock.presetPos[0][0];
                    dock_info.presetPos[0][1] = data.dock.presetPos[0][1];
                    dock_info.presetPos[1][0] = data.dock.presetPos[1][0];
                    dock_info.presetPos[1][1] = data.dock.presetPos[1][1];
                    dock_info.berth = data.dock.berth;
                    dock_info.change = data.dock.change;

                    Ctrl_Result = back_ctrl(param, euler, gnss, lidar, dock_info);
                    if(Ctrl_Result.ctrl_finished)
                        task_index++; 

                    break;
                }

                case TASK_TYPE_ENUM::PATH_TRACKING:
                {
                    Param_PathTracking_t param = getParam_PathTracking();
                    std::map<int, std::vector<double>> path = task_val;
                    saveTrackPath(path);  // 保存路径供恢复时使用

                    Ctrl_Result.ctrl_mode = 1;
                    Ctrl_Result.Y = 0;
                    Ctrl_Result.H = 0;
                    //目标识别+推流+跟踪的小状态机
                    switch (m_EdgeSubState) {
                        case EDGE_FOLLOW:
                        {
                            Ctrl_Result = path_tracking_ctrl(param, euler, gnss, lidar, path);
                            setPathTrackingHeading(Ctrl_Result.objHeading);  // 保存目标航向供 visual_track 使用
                            if (Ctrl_Result.ctrl_finished)
                                task_index++;
                            if (m_VisualTrackHasTarget
                                && getEnableSwitches().vision_en) {
                                m_EdgeSubState = CLEANING;
                                ROS_INFO("[PATH] 发现目标，切换到视觉跟踪");
                            }
                            break;
                        }
                        case CLEANING:
                            Ctrl_Result.objSpeed   = getVisualTrackSpeed();
                            Ctrl_Result.objHeading = getVisualTrackHeading();
                            Ctrl_Result.isAvoiding = false;
                            Ctrl_Result.ctrl_finished = false;
                            if (m_VisualTrackReached) {
                                m_EdgeSubState = CLEAN_DONE;
                                ROS_INFO("[PATH] 到达垃圾位置，开始减速");
                            }
                            break;
                        case CLEAN_DONE:
                        {
                            Ctrl_Result.objSpeed   = 0.f;
                            Ctrl_Result.isAvoiding = false;
                            Ctrl_Result.ctrl_finished = false;
                            Ctrl_Result.objHeading = getVisualTrackHeading();
                            // 等待 control_node 恢复路径 → EDGE_FOLLOW
                            break;
                        }
                    }
                    break;
                }
                //沿边
                case TASK_TYPE_ENUM::EDGE_FOLLOWING:
                {
                    Edge_ctrl ec = getEdgeCtrl();
                    Ctrl_Result.ctrl_mode       = 1;
                    Ctrl_Result.Y               = 0;
                    Ctrl_Result.H               = 0;
                    Ctrl_Result.objSpeed        = ec.objSpeed;
                    Ctrl_Result.objHeading      = ec.objHeading;
                    Ctrl_Result.isAvoiding      = ec.isAvoiding;
                    Ctrl_Result.ctrl_finished   = ec.reached;
                    if (Ctrl_Result.ctrl_finished)
                        task_index++;
                    break;
                }
            }

            if(task_index >= task.size())
            {
                status = TASK_STATUS_ENUM::STOP;
                mode = RUN_MODE_ENUM::NORMAL;
                ctrl.runMode = 0;

                ctrl.ctrl_mode = 0;
                ctrl.Y = 0;
                ctrl.H = 0;
                ctrl.objHeading = 0;
                ctrl.objSpeed = 0;
                
                ctrl.isAvoiding = 0;

                ctrl.dock_ip[0] = 0;
                ctrl.dock_ip[1] = 0;
                ctrl.dock_ip[2] = 0;
                ctrl.dock_ip[3] = 0;

                ctrl.dock_port = 0;

                // 注意：最后一段任务完成时 task_index 已被++到==task.size()，
                // 不能再用 task[task_index]，必须取 task.back()，否则越界读到脏数据
                data.run.task_tepy = task.back().task_type;
                data.run.task_sta = status;

                 //任务结束时清零
                data.run.vision_state = 0;
                data.run.mapping_state = 0;

                task_index = 0;
                sail_step = 0; //出航步骤
                back_step = 0; //返航步骤
                path_id = 0;   //路径id  
            }
            else
            {
                mode = RUN_MODE_ENUM::TASK;
                ctrl.runMode = 0x07;

                ctrl.ctrl_mode = Ctrl_Result.ctrl_mode;
                ctrl.Y = Ctrl_Result.Y;
                ctrl.H = Ctrl_Result.H;
                ctrl.objHeading = Ctrl_Result.objHeading;
                ctrl.objSpeed = Ctrl_Result.objSpeed;
                
                ctrl.isAvoiding = Ctrl_Result.isAvoiding;

                ctrl.dock_ip[0] = Ctrl_Result.dock_ip[0];
                ctrl.dock_ip[1] = Ctrl_Result.dock_ip[1];
                ctrl.dock_ip[2] = Ctrl_Result.dock_ip[2];
                ctrl.dock_ip[3] = Ctrl_Result.dock_ip[3];

                ctrl.dock_port = Ctrl_Result.dock_port;

                data.run.task_tepy = task[task_index].task_type;
                data.run.task_sta = status;
                                // 识别+推流+跟踪 状态推断:
                //   vision_en关闭                               = 关
                //   沿边状态机处于 CLEANING        = 正在跟踪垃圾(跟踪中)
                //   处于 EDGE_FOLLOW              = 沿边进行中(识别+推流开启)
                //   其余(非沿边/已停)               = 关闭
                data.run.vision_state = (getEnableSwitches().vision_en
                    && getCurrentTaskType() == TASK_TYPE_ENUM::PATH_TRACKING)
                    ? ((m_EdgeSubState == EdgeSubState::CLEANING) ? 2
                      : ((m_EdgeSubState == EdgeSubState::EDGE_FOLLOW) ? 1 : 0))
                    : 0;
                // 建图状态:开关开→建图中
                data.run.mapping_state = getEnableSwitches().mapping_en ? 1 : 0;
                printf("data.run.task_tepy: %d   data.run.task_sta: %d\r\n",data.run.task_tepy, data.run.task_sta); 
            }
            break;
        }

        case TASK_STATUS_ENUM::PAUSE:
        {
            mode = RUN_MODE_ENUM::TASK;
            ctrl.runMode = 0x07; //任务模式
            ctrl.ctrl_mode = 0x00; //自由控制
            ctrl.objHeading = 0;
            ctrl.objSpeed = 0;
            
            ctrl.isAvoiding = 0;

            ctrl.dock_ip[0] = 0;
            ctrl.dock_ip[1] = 0;
            ctrl.dock_ip[2] = 0;
            ctrl.dock_ip[3] = 0;

            ctrl.dock_port = 0;

            data.run.task_tepy = (task_index < static_cast<int>(task.size()))
                                 ? task[task_index].task_type : task.back().task_type;
            data.run.task_sta = status;

            break;
        }

        case TASK_STATUS_ENUM::STOP:
        {
            mode = RUN_MODE_ENUM::NORMAL;
            ctrl.runMode = 0x00;
            ctrl.Y = 0;
            ctrl.H = 0;
            ctrl.objHeading = 0;
            ctrl.objSpeed = 0;
            
            ctrl.isAvoiding = 0;

            ctrl.dock_ip[0] = 0;
            ctrl.dock_ip[1] = 0;
            ctrl.dock_ip[2] = 0;
            ctrl.dock_ip[3] = 0;

            ctrl.dock_port = 0;

            data.run.task_tepy = (task_index < static_cast<int>(task.size()))
                                 ? task[task_index].task_type : task.back().task_type;
            data.run.task_sta = status;

            task_index = 0;
            sail_step = 0; //出航步骤
            back_step = 0; //返航步骤
            path_id = 0;   //路径id

            
            break;
        }
    }
    setTASK_STATUS(status);

    setRunCtrl(ctrl);
    setStateData(data);

    return mode;
}  

//用户自定义
RUN_MODE_ENUM Control::executeUser()
{
    RUN_MODE_ENUM mode = RUN_MODE_ENUM::USER;

    return mode;
}
// ================================================================
// 1：视觉数据槽（由 control_node 的 visualCmdCallback 写入）
//    供 executeTask 的 PATH_TRACKING(轨迹) 状态机读取
// ================================================================

// ---- 1.1 是否有目标（EDGE_FOLLOW → CLEANING 切换条件） ----
void Control::setVisualTrackHasTarget(bool has) {
    std::lock_guard<std::mutex> lock(m_VisualTrackMutex);
    m_VisualTrackHasTarget = has;
}
bool Control::getVisualTrackHasTarget() const {
    std::lock_guard<std::mutex> lock(m_VisualTrackMutex);
    return m_VisualTrackHasTarget;
}

// ---- 1.2 是否到达垃圾（CLEANING → CLEAN_DONE 切换条件） ----
void Control::setVisualTrackReached(bool reached) {
    std::lock_guard<std::mutex> lock(m_VisualTrackMutex);
    m_VisualTrackReached = reached;
}
bool Control::getVisualTrackReached() const {
    std::lock_guard<std::mutex> lock(m_VisualTrackMutex);
    return m_VisualTrackReached;
}

// ---- 1.3 目标速度（CLEANING 状态执行用） ----
void Control::setVisualTrackSpeed(float s) {
    std::lock_guard<std::mutex> lock(m_VisualTrackMutex);
    m_VisualTrackSpeed = s;
}
float Control::getVisualTrackSpeed() const {
    std::lock_guard<std::mutex> lock(m_VisualTrackMutex);
    return m_VisualTrackSpeed;
}

// ---- 1.4 目标航向（CLEANING 状态执行用） ----
void Control::setVisualTrackHeading(float h) {
    std::lock_guard<std::mutex> lock(m_VisualTrackMutex);
    m_VisualTrackHeading = h;
}
float Control::getVisualTrackHeading() const {
    std::lock_guard<std::mutex> lock(m_VisualTrackMutex);
    return m_VisualTrackHeading;
}

// ================================================================
// 2：状态机状态控制（executeTask 中读取/修改当前子状态）
// ================================================================

Control::EdgeSubState Control::getEdgeSubState() const {
    return m_EdgeSubState;
}
void Control::setEdgeSubState(EdgeSubState s) {
    m_EdgeSubState = s;
}

void Control::setPathId(int id) {
    path_id = id;
}
int Control::getPathId() const {
    return path_id;
}
int Control::getCurrentTaskType() const {
    auto task = getTaskFile();
    if (task_index < static_cast<int>(task.size()))
        return task[task_index].task_type;
    return TASK_TYPE_ENUM::EMPTY;
}


// ================================================================
// 3：沿边路径恢复（CLEAN_DONE 状态使用）
//        保存 planner 的沿边路径，恢复时重新发送
// ================================================================

// ---- 3.1 读取保存的路径 ----
const std::vector<Eigen::Vector2d>& Control::getResumePath() const {
    return m_ResumePath;
}

// ---- 3.2 读取水在船哪一侧（沿边方向） ----
bool Control::getResumeWaterLeft() const {
    return m_ResumeWaterLeft;
}

// ---- 3.3 保存沿边路径 + 方向（由 plannerFeedIfNeeded 调用） ----
void Control::saveEdgePath(const std::vector<Eigen::Vector2d>& path, bool water_left) {
    m_ResumePath = path;
    m_ResumeWaterLeft = water_left;
}

// ---- 4. 轨迹跟踪路径保存/读取 ----
void Control::saveTrackPath(const std::map<int, std::vector<double>>& path) {
    m_TrackPath = path;
}
const std::map<int, std::vector<double>>& Control::getTrackPath() const {
    return m_TrackPath;
}

void Control::setPathTrackingHeading(float h) {
    m_PathTrackingHeading = h;
}
float Control::getPathTrackingHeading() const {
    return m_PathTrackingHeading;
}

