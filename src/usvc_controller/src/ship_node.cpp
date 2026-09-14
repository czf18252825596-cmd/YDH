#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <ros/package.h>  // 用于获取包路径
#include <unistd.h>       // 用于获取当前路径

#include <thread>
#include <functional>

#include "config_manager_tool/config_manager_tool.h"

#include "usvc_controller/ID.h"
#include "usvc_controller/RunCtrl.h"
#include "usvc_controller/RawData.h"
#include "usvc_controller/StateData.h"


extern "C" {
#include "MTLink/MTLink.h"
}


using namespace boost::asio;
using namespace boost::asio::ip;
using boost::asio::buffer;

static int LISTEN_PORT;
static std::string DEVICE_SN;
static std::string SHIP_TYPE;

static MTLink_t g_MTLink_ship = {0};

static std::atomic<uint8_t> g_ship_connect_flag(0); //连接标志 0_无连接 1_连接 
// 全局socket
static std::mutex g_ship_socket_mutex;
static std::shared_ptr<tcp::socket> g_ship_socket;

static std::atomic<uint8_t> g_ship_server_ready(false); 
static std::string g_ship_server_ip;
static int g_ship_server_port;

static ros::Publisher ID_pub; 
static ros::Publisher RawData_pub; 
static ros::Publisher StateData_pub; 

static ros::Subscriber ID_sub;
static ros::Subscriber RawData_sub; 
static ros::Subscriber RunCtrl_sub;



// TCPC发送，用于注册MTLink
static bool shipTcpcSendBuffer(void* context, uint8_t* buf, uint16_t len) 
{
    bool state = false;

    std::lock_guard<std::mutex> lock(g_ship_socket_mutex);
    
    if (g_ship_socket && g_ship_socket->is_open()) 
    {
        boost::system::error_code ec;
        size_t bytes_sent = g_ship_socket->write_some(boost::asio::buffer(buf, len), ec);
        state |= (ec?false:true); 
    }

    return state;
}


// TCPC接收，用于注册MTLink
static bool shipTcpcRecvBuffer(void* context, uint8_t* buf, uint16_t* len) 
{
    bool state = false;

    std::shared_ptr<tcp::socket> ship_socket;
    {
        std::lock_guard<std::mutex> lock(g_ship_socket_mutex);
        ship_socket = g_ship_socket;  // 复制shared_ptr
    }


    if (!ship_socket || !ship_socket->is_open()) 
    {
        if(g_ship_connect_flag == 1)
        {
             g_ship_connect_flag = 0;  // 触发重连
        }
           

        return false;
    }
    
    boost::system::error_code ec;
    size_t bytes_read = ship_socket->read_some(boost::asio::buffer(buf, 2048), ec);
    *len = static_cast<uint16_t>(bytes_read);
    state |= (ec?false:true); 

    if(ec && ec != boost::asio::error::would_block)
    {
        if(g_ship_connect_flag == 1)
        {
            g_ship_connect_flag = 0;  // 触发重连
        }     
    }

    return state;
}


//UDP广播监听
static void udpListenThread(void)
{
    try {

        // 创建IO上下文和套接字
        io_context io_context_;
        udp::socket socket_(io_context_);
        
        // 创建监听端点
        udp::endpoint endpoint_(udp::v4(), LISTEN_PORT);
        
        // 配置套接字
        socket_.open(endpoint_.protocol());
        socket_.set_option(socket_base::reuse_address(true));
        socket_.set_option(socket_base::broadcast(true)); // 如果需要接收广播
        socket_.bind(endpoint_);
        socket_.non_blocking(true); // 设置非阻塞模式

        ROS_INFO("---[ship_node.cpp] udp listener started on port %d", LISTEN_PORT);
        
        // 接收缓冲区
        const size_t buf_size = 1024;
        char buf[buf_size];
        udp::endpoint sender_endpoint;
        
        while (ros::ok()) 
        {
            try {
                boost::system::error_code ec;
                size_t bytes_received = 0;
                
                // 尝试接收数据（非阻塞）
                bytes_received = socket_.receive_from(buffer(buf, buf_size), sender_endpoint, 0, ec);

                if (!ec && bytes_received > 0) 
                {
                    //解析JSON并获取船服务的IP和Port
                    try 
                    {
                        std::string json_str(buf, bytes_received);
                        std::istringstream iss(json_str);

                        boost::property_tree::ptree pt;
                        boost::property_tree::read_json(iss, pt);

                        int id = pt.get<int>("id");
                        std::string ip = pt.get<std::string>("ip");
                        int port = pt.get<int>("port");
                        std::string type = pt.get<std::string>("type");
                        std::string sn = pt.get<std::string>("sn");

                        if(type == SHIP_TYPE)
                        {
                            if(DEVICE_SN == "NULL")
                            {
                                ROS_INFO("---[ship_node.cpp] first get ship sn");
                                g_ship_server_ip = ip;
                                g_ship_server_port = port;

                                g_ship_server_ready = true;

                                //将yaml文件中的DEVICE_SN修改为本次接收到的sn并保存
                                std::string package_path = ros::package::getPath("usvc_controller");
                                std::string config_file  = package_path + "/config/config.yaml";

                                if (ConfigManagerTool::updateParamInFile(config_file, "DEVICE_SN", sn))
                                {
                                    // 更新内存中的DEVICE_SN值
                                    DEVICE_SN = sn;
                                    ROS_INFO("---[ship_node.cpp] saved ship sn is %s", DEVICE_SN.c_str());
                                }
                            }
                            else
                            {
                                if(sn == DEVICE_SN)
                                {
                                    g_ship_server_ip = ip;
                                    g_ship_server_port = port;

                                    // g_ship_server_ip = "192.168.1.9";
                                    // g_ship_server_port = 3000;

                                    g_ship_server_ready = true;
                                }
                            }
                        }
                    }catch (const std::exception& e) {
                        std::cerr<< "ship_node.cpp ==== failed to parse broadcast message: " << e.what() << std::endl;
                    }
                }

                // 如果没有休眠，给其他线程执行机会
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

            } catch (const std::exception& e) {
                std::cerr<< "ship_node.cpp ==== Exception in UDP listener: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }catch (const std::exception& e) {
        std::cerr<< "ship_node.cpp ==== Failed to initialize UDP listener on port " << LISTEN_PORT << ": " << e.what() << std::endl;
        return; // 初始化失败直接返回
    }
}


//TCP连接线程
static void tcpcShipMTLinkConnectThread(void)
{
    auto io_context = std::make_shared<boost::asio::io_context>();

    while (ros::ok()) 
    {  
        if(g_ship_connect_flag != 1)
        {
            if(g_ship_server_ready)
            {
                try {
                     // 创建新socket
                    auto new_socket = std::make_shared<tcp::socket>(*io_context);
                    new_socket->open(tcp::v4());
                    new_socket->non_blocking(true);  // 设置非阻塞

                    tcp::endpoint endpoint(address::from_string(g_ship_server_ip), g_ship_server_port);

                    boost::system::error_code ec;
                    new_socket->connect(endpoint, ec);

                    if (!ec) 
                    {
                        {
                            std::lock_guard<std::mutex> lock(g_ship_socket_mutex);
                            
                            // 关闭旧的socket
                            if(g_ship_socket && g_ship_socket->is_open())
                            {
                                g_ship_socket->close();
                            }
                            
                            // 设置新的socket
                            g_ship_socket = new_socket;
                        } 
                        // 立即连接成功
                        g_ship_connect_flag = 1;
                        ROS_INFO("---[ship_node.cpp] ship connected to server %s:%d", g_ship_server_ip.c_str(), g_ship_server_port);
                    } 
                    else if (ec == boost::asio::error::in_progress) 
                    {
                        // 连接进行中，等待一下再检查
                        ROS_INFO("---[ship_node.cpp] ship connection in progress...");
                        g_ship_connect_flag = 0;  // 保持连接标志，下次继续尝试
                    }
                    else
                    {
                        // 连接失败
                        ROS_WARN("---[ship_node.cpp] ship failed to connect: %s", ec.message().c_str());
                        std::this_thread::sleep_for(std::chrono::seconds(5));
                        g_ship_connect_flag = 0;  // 保持连接标志，下次继续尝试
                    }
                }catch (const boost::system::system_error& e) {
                    // 连接失败（服务没开、网络不通等）
                    std::cerr<< "ship_node.cpp ==== ship connect fail: " << e.what() << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    g_ship_connect_flag = 0;  // 保持连接标志，下次继续尝试
                }
            }
        }

        // 处理异步IO事件
        io_context->restart();
        io_context->poll();
        
        // 短的睡眠时间，提高响应速度
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



//ID
static void IDCallback(const usvc_controller::ID::ConstPtr& msg)
{
    MDATA_SHIP_ID_t SHIP_ID;

    SHIP_ID.flag = msg->flag;
    SHIP_ID.ID = msg->ID;

    MTLink_Send(&g_MTLink_ship, 0, 0, OBJ_SHIP_SetID, (uint8_t*)&SHIP_ID, sizeof(MDATA_SHIP_ID_t));
}


//RawData
static void RawDataCallback(const usvc_controller::RawData::ConstPtr& msg)
{
    uint16_t Obj = msg->obj;
	uint8_t* pBuf = const_cast<uint8_t*>(msg->data.data());
	uint16_t Len = msg->len;

    ROS_INFO("---[ship_node.cpp]  subscribe raw obj is 0x%04x:",Obj);

    MTLink_Send(&g_MTLink_ship, 0, 0, Obj, pBuf, Len);
}


//运动控制
static void RunCtrlCallback(const usvc_controller::RunCtrl::ConstPtr& msg)
{
    MDATA_SHIP_RunCtrl_t SHIP_RunCtrl;
    memset(&SHIP_RunCtrl, 0, sizeof(MDATA_SHIP_RunCtrl_t));

    // 运行模式赋值
    SHIP_RunCtrl.runMode = msg->run_mode;
    
    // 控制参数赋值
    SHIP_RunCtrl.Y = msg->Y;                    // Y轴摇杆值
    SHIP_RunCtrl.H = msg->H;                    // H轴摇杆值
    SHIP_RunCtrl.objSpeed = msg->obj_speed;     // 目标速度
    SHIP_RunCtrl.objHeading = msg->obj_heading; // 目标航向

    SHIP_RunCtrl.isAvoiding = msg->is_avoiding; // 避障状态

    // 码头信息赋值
    SHIP_RunCtrl.has_dock = msg->has_dock;      // 有无码头标志
    SHIP_RunCtrl.dock_ip[0] = msg->dock_ip[0];
    SHIP_RunCtrl.dock_ip[1] = msg->dock_ip[1];
    SHIP_RunCtrl.dock_ip[2] = msg->dock_ip[2];
    SHIP_RunCtrl.dock_ip[3] = msg->dock_ip[3];
    SHIP_RunCtrl.dock_port = msg->dock_port; // 码头端口

    // 任务和控制模式赋值
    SHIP_RunCtrl.task_state = msg->task_state;   // 任务状态
    SHIP_RunCtrl.task_teyp = msg->task_tepy;
    SHIP_RunCtrl.ctrl_mode = msg->ctrl_mode;     // 控制模式
    
    MTLink_Send(&g_MTLink_ship, 0, 0, OBJ_SHIP_RunCtrl, (uint8_t*)&SHIP_RunCtrl, sizeof(MDATA_SHIP_RunCtrl_t));
}



int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ship_node");
    
    ros::NodeHandle nh;

    // ROS_INFO("---[ship_node.cpp]  START ");

    nh.param<int>("ship/LISTEN_PORT", LISTEN_PORT, 5000);
    nh.param<std::string>("DEVICE_SN", DEVICE_SN, "NULL");
    nh.param<std::string>("SHIP_TYPE", SHIP_TYPE, "ship_main");

    // ROS_INFO("---[ship_node.cpp]  1 ");
    
    memset(&g_MTLink_ship, 0, sizeof(MTLink_t));
    MTLink_RegSendFunc(&g_MTLink_ship, shipTcpcSendBuffer, NULL);
    MTLink_RegReciveFunc(&g_MTLink_ship, shipTcpcRecvBuffer, NULL);
    
    // ROS_INFO("---[ship_node.cpp]  2 ");
    
    std::thread thread_udpListen(udpListenThread); //启动线程
    std::thread thread_tcpcShipMTLinkConnect(tcpcShipMTLinkConnectThread);
   
    // ROS_INFO("---[ship_node.cpp]  3 ");
    
    ID_pub         = nh.advertise<usvc_controller::ID>("/ship/ID", 10); //
    RawData_pub    = nh.advertise<usvc_controller::RawData>("/ship/RawData", 10); //
    StateData_pub  = nh.advertise<usvc_controller::StateData>("/ship/StateData", 10); //
    
    // ROS_INFO("---[ship_node.cpp]  4 ");
    
    ID_sub      = nh.subscribe("/upper/ID", 10, IDCallback);
    RawData_sub = nh.subscribe("/upper/RawData", 10, RawDataCallback);
    RunCtrl_sub = nh.subscribe("/control/RunCtrl", 10, RunCtrlCallback);
 

    ros::Rate rate(100); // 100Hz
    while (ros::ok()) 
    {
        // ROS_INFO("---[ship_node.cpp]  run ");
        uint8_t SID;
		uint8_t DID;
		uint16_t Obj;
		uint8_t *pBuf;
		uint16_t Len;

        if(MTLink_Receive(&g_MTLink_ship, &SID, &DID, &Obj, &pBuf, &Len))
        {
            switch(Obj)
			{
                case OBJ_SHIP_BoardReset:
                case OBJ_SHIP_BoardInfo:
                case OBJ_SHIP_BoardVersion:
                case OBJ_SHIP_BoardNetwork:
                {
                    ROS_INFO("---[ship_node.cpp]  publish raw obj is 0x%04x:",Obj);
                    //工控机不处理以上指令，不做数据解析，将原始数据直接发布给主控
                    usvc_controller::RawData msg;
                    msg.obj = Obj;
                    msg.len = Len;
                    msg.data.assign(pBuf, pBuf + Len);

                    RawData_pub.publish(msg);
                    break;
                }

                case OBJ_SHIP_SetID:
                {
                    //ID参数的处理，支持工控机端保存，并将指令同步转发给主板
                    MDATA_SHIP_ID_t SHIP_ID;
                    memcpy(&SHIP_ID, pBuf, Len);
                    
                    usvc_controller::ID msg;
                    msg.flag = SHIP_ID.flag;
                    msg.ID = SHIP_ID.ID;

                    ID_pub.publish(msg);

                    break;
                }

                case OBJ_SHIP_BoardPort:
                case OBJ_SHIP_YawPIDParam:
                case OBJ_SHIP_SpeedPIDParam:
                case OBJ_SHIP_DisPIDParam:
                {
                    //工控机不处理以上指令，不做数据解析，将原始数据直接发布给主控
                    usvc_controller::RawData msg;
                    msg.obj = Obj;
                    msg.len = Len;
                    msg.data.assign(pBuf, pBuf + Len);

                    RawData_pub.publish(msg);
                    break;
                }

                case OBJ_SHIP_ResetSN:
                case OBJ_SHIP_LidarParam:
                case OBJ_SHIP_SailParam:
                case OBJ_SHIP_BackParam:
                case OBJ_SHIP_PathTrackParam:
                case OBJ_SHIP_RunCtrl:
                case OBJ_SHIP_PMSCtrl:
                case OBJ_SHIP_MotorCtrl:
                case OBJ_SHIP_LightCtrl:
                case OBJ_SHIP_BerthCtrl:
                case OBJ_SHIP_ChangeCtrl:


                case OBJ_SHIP_StateData:
                {
                    MDATA_SHIP_StateData_t StateData;
                    memcpy(&StateData, pBuf, Len);

                    usvc_controller::StateData msg;
                    // 1. 传感器数据
                    msg.humidity = StateData.sensor.humi;
                    msg.temperature = StateData.sensor.temp;
                    msg.leak_detection[0] = StateData.sensor.leak[0];
                    msg.leak_detection[1] = StateData.sensor.leak[1];
                    // 2. 电源状态
                    msg.voltage = StateData.power.volage;
                    msg.current = StateData.power.current;
                    // 3. 电源管理
                    msg.Thruster = StateData.pms.Thruster;
                    msg.LEDScre = StateData.pms.LEDScre;
                    msg.unused2 = StateData.pms.unused2;
                    msg.unused3 = StateData.pms.unused3;
                    // 4. 电池状态
                    msg.battery_q = StateData.battery.Q;
                    msg.battery_v = StateData.battery.V;
                    msg.battery_a = StateData.battery.A;
                    msg.battery_t = StateData.battery.T;
                    // 5. 运行状态
                    msg.runMode = StateData.run.runMode;
                    msg.y = StateData.run.Y;
                    msg.h = StateData.run.H;
                    msg.objSpeed = StateData.run.objSpeed;
                    msg.objHeading = StateData.run.objHeading;
                    msg.objPos[0] = StateData.run.objPos[0];
                    msg.objPos[1] = StateData.run.objPos[1];
                    msg.task_tepy = StateData.run.task_tepy;
                    msg.task_sta = StateData.run.task_sta;
                    msg.vision_state = StateData.run.vision_state;
                    msg.mapping_state = StateData.run.mapping_state;
                    // 6. 导航数据
                    msg.ins_status = StateData.navi.ins_sta;
                    msg.timestamp = StateData.navi.timeTick;
                    msg.gyro_roll = StateData.navi.gyro_roll;
                    msg.gyro_pitch = StateData.navi.gyro_pitch;
                    msg.gyro_heading = StateData.navi.gyro_heading;
                    msg.roll = StateData.navi.roll;
                    msg.pitch = StateData.navi.pitch;
                    msg.heading = StateData.navi.heading;
                    msg.acc_x = StateData.navi.acc_x;
                    msg.acc_y = StateData.navi.acc_y;
                    msg.acc_z = StateData.navi.acc_z;
                    msg.vel_x = StateData.navi.vel_x;
                    msg.vel_y = StateData.navi.vel_y;
                    msg.vel_z = StateData.navi.vel_z;
                    msg.latitude = StateData.navi.latitude;
                    msg.longitude = StateData.navi.longitude;
                    msg.altitude = StateData.navi.altitude;
                    msg.vel_n = StateData.navi.vel_n;
                    msg.vel_e = StateData.navi.vel_e;
                    msg.vel_d = StateData.navi.vel_d;
                    msg.satellite_count = StateData.navi.satellite_count;
                    msg.pos_status = StateData.navi.pos_sta;
                    msg.heading_status = StateData.navi.heading_sta;
                    msg.pos_std_lat = StateData.navi.position_std_lat;
                    msg.pos_std_lon = StateData.navi.position_std_lon;
                    // 7. 推进器状态
                    msg.left_thruster_val = StateData.thruster.left_Val;
                    msg.left_thruster_status = StateData.thruster.left_sta;
                    msg.left_duty_cycle = StateData.thruster.left_duty_scaled;
                    msg.left_rpm = StateData.thruster.left_rpm;
                    msg.left_mos_temp = StateData.thruster.left_MOST;
                    msg.left_total_current = StateData.thruster.left_total_current;
                    msg.left_input_current = StateData.thruster.left_current_in;
                    
                    msg.right_thruster_val = StateData.thruster.right_Val;
                    msg.right_thruster_status = StateData.thruster.right_sta;
                    msg.right_duty_cycle = StateData.thruster.right_duty_scaled;
                    msg.right_rpm = StateData.thruster.right_rpm;
                    msg.right_mos_temp = StateData.thruster.right_MOST;
                    msg.right_total_current = StateData.thruster.right_total_current;
                    msg.right_input_current = StateData.thruster.right_current_in;
                    
                    // 8. 灯状态
                    msg.LED = StateData.light.LED;
                    msg.CautionLED = StateData.light.CautionLED;
                    
                    // 9. 码头信息
                    msg.connect = StateData.dock.connect;
                    msg.dock_ID = StateData.dock.dock_ID;
                    msg.preset_positions0[0] = StateData.dock.presetPos[0][0];
                    msg.preset_positions0[1] = StateData.dock.presetPos[0][1];
                    msg.preset_positions1[0] = StateData.dock.presetPos[1][0];
                    msg.preset_positions1[1] = StateData.dock.presetPos[1][1];
                    msg.berth_status = StateData.dock.berth;
                    msg.charge_status = StateData.dock.change;
                                    
                    StateData_pub.publish(msg);
                    break;
                }

                default:
                    break;
            }
        }

        ros::spinOnce();    
        rate.sleep();
    }

    if (thread_udpListen.joinable()) 
    {
        thread_udpListen.join();
    }
    
    if (thread_tcpcShipMTLinkConnect.joinable()) 
    {
        thread_tcpcShipMTLinkConnect.join();
    }
    
    return 0;
}