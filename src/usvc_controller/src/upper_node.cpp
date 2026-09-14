#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <boost/asio.hpp>
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
#include <yaml-cpp/yaml.h>  // 添加这个头文件
#include <fstream>           // 确保有 fstream

#include <thread>
#include <functional>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "usvc_controller/ACHData.h"
#include "usvc_controller/ID.h"
#include "usvc_controller/RawData.h"
#include "usvc_controller/RunCtrl.h"
#include "usvc_controller/StateData.h"
#include "usvc_controller/EnableSwitches.h"

#include "usvc_controller/UpdateParams.h"

#include "config_manager_tool/config_manager_tool.h"

extern "C" {
#include "MTLink/MTLink.h"
}

using boost::asio::io_context;
using boost::asio::ip::udp;
using boost::asio::ip::tcp;
using boost::asio::ip::address;
using boost::asio::buffer;
using boost::asio::socket_base;

static int BROAD_PORT;
static int DEVICE_ID;
static int DEVICE_PORT;
static std::string DEVICE_TYPE;
static std::string DEVICE_SN;

static MTLink_t g_MTLink_upper = {0};

static std::vector<uint8_t> g_json_buffer; // 全局缓冲区用于累积JSON数据

static std::mutex upper_clients_mutex; 
static std::vector<std::shared_ptr<tcp::socket>> upper_clients; //上位机客户端连接池
static std::mutex upper_clients_to_remove_mutex;
static std::vector<size_t> upper_clients_to_remove; //上位机客户端连接移除标志

static ros::Publisher UpperConect_pub;
static ros::Publisher NewTaskFile_pub; 

static ros::Publisher ID_pub; 
static ros::Publisher RawData_pub; 
static ros::Publisher RunCtrl_pub;

static ros::Subscriber lidar_filter_sub;
static ros::Subscriber ID_sub;
static ros::Subscriber RawData_sub; 
static ros::Subscriber StateData_sub;
static ros::Subscriber ACHData_sub;

static ros::ServiceClient lidar_param_client;
static ros::ServiceClient sail_param_client;
static ros::ServiceClient back_param_client;
static ros::ServiceClient path_param_client;
static ros::ServiceClient enable_switches_client;
static ros::ServiceClient tunable_param_client;
static ros::ServiceClient charge_param_client;


// TCPS发送，用于注册MTLink
static bool upperTcpsSendBuffer(void* context, uint8_t* buf, uint16_t len) 
{
    bool state = false;

    std::vector<std::shared_ptr<tcp::socket>> current_clients;
        
    // 复制客户端列表
    {
        std::lock_guard<std::mutex> lock(upper_clients_mutex);
        current_clients = upper_clients;
    }
        
        
    for (size_t i = 0; i < current_clients.size(); ++i) 
    {
        auto& socket = current_clients[i];

        if (!socket->is_open()) 
            continue;

        boost::system::error_code ec;
        size_t bytes_sent = socket->write_some(buffer(buf, len), ec);
        state |= (ec?false:true); 
    }

    return state;
}


// TCPS接收，用于注册MTLink
static bool upperTcpsRecvBuffer(void* context, uint8_t* buf, uint16_t* len) 
{
    bool state = false;

    std::vector<std::shared_ptr<tcp::socket>> current_clients;
        
    // 复制客户端列表
    {
        std::lock_guard<std::mutex> lock(upper_clients_mutex);
        current_clients = upper_clients;
    }
        
        
    for (size_t i = 0; i < current_clients.size(); ++i) 
    {
        auto& socket = current_clients[i];

        if (!socket->is_open()) 
        {
            //标记要移除的连接
            {
                std::lock_guard<std::mutex> lock(upper_clients_to_remove_mutex);
                upper_clients_to_remove.push_back(i);
            }

            continue;
        }
            

        boost::system::error_code ec;
        size_t bytes_read = socket->read_some(buffer(buf, 2048), ec);
        *len = static_cast<uint16_t>(bytes_read);
        state |= (ec?false:true); 

        if(ec && ec != boost::asio::error::would_block)
        {
            ROS_INFO("---[upper_node.cpp] upper Client disconnected");
            socket->close();
            std_msgs::Bool conn;
            conn.data = false;
            UpperConect_pub.publish(conn);
            //标记要移除的连接
            {
                std::lock_guard<std::mutex> lock(upper_clients_to_remove_mutex);
                upper_clients_to_remove.push_back(i);
            }
        }

        if(state && *len>0)
        {
            std_msgs::Bool conn;
            conn.data = true;
            UpperConect_pub.publish(conn);
            break;
        }    
    }
    
    return state;
}


//信息广播线程函数
static void udpBroadcastThread()
{
    try {
        io_context io_context_; 
        udp::socket socket_(io_context_); //创建套接字
        udp::endpoint endpoint_(udp::v4(), 0); 
        socket_.open(endpoint_.protocol()); //打开套接字

        socket_.set_option(socket_base::broadcast(true)); //允许广播
        socket_.set_option(socket_base::reuse_address(true)); //允许地址重用
        socket_.bind(endpoint_); //绑定套接字

        //获取本地IP
        std::string local_ip = "127.0.0.1";
        socket_.connect(boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::broadcast(), BROAD_PORT));
        auto local_endpoint = socket_.local_endpoint();
        local_ip = local_endpoint.address().to_string();   
        ROS_INFO("---[upper_node.cpp] udpBroadcast->local_ip is %s", local_ip.c_str());

        // 创建JSON格式的消息
        std::string json_str = "{\"id\":" + std::to_string(DEVICE_ID) + 
                       ",\"ip\":\"" + local_ip + 
                       "\",\"port\":" + std::to_string(DEVICE_PORT) + 
                       ",\"type\":\"" + DEVICE_TYPE + 
                       "\",\"sn\":\"" + DEVICE_SN + "\"}";
        // 创建广播地址
        while (ros::ok()) 
        {
            try {
                // 发送消息
                size_t bytes_sent = socket_.send(buffer(json_str));
                        
            } catch (const std::exception& e) {
                std::cerr << "upper_node.cpp ==== Error sending broadcast: " << e.what() << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (const std::exception& e) {
        std::cerr << "upper_node.cpp ==== Failed to initialize broadcast socket: " << e.what() << std::endl;
        return; // 初始化失败直接返回
    }
}


//TCP服务端接受客户端连接线程，支持多客户端连接
static void tcpsUpperMTLinkAcceptThread()
{
    try {
        io_context io_context_;
        tcp::acceptor acceptor_(io_context_, tcp::endpoint(tcp::v4(), DEVICE_PORT));

        // 开始异步接受的函数
        std::function<void()> do_accept;
        do_accept = [&]() 
        {
            auto new_socket = std::make_shared<tcp::socket>(io_context_);
            
            acceptor_.async_accept(*new_socket,
                [&, new_socket](const boost::system::error_code& error) {
                    if (!error) {
                        // 获取客户端信息
                        std::string client_ip = new_socket->remote_endpoint().address().to_string();
                        int client_port = new_socket->remote_endpoint().port();
                        
                        ROS_INFO("---[upper_node.cpp] upper new connection from %s:%d", client_ip.c_str(), client_port);
                        
                        // 设置为非阻塞，方便主循环读取
                        new_socket->non_blocking(true);
                        
                        // 添加到全局客户端列表
                        {
                            std::lock_guard<std::mutex> lock(upper_clients_mutex);
                            upper_clients.push_back(new_socket);
                        }
                    } 
                    else 
                    {
                        ROS_ERROR("---[upper_node.cpp] upper accept error: %s", error.message().c_str());
                    }
                    
                    // 继续接受下一个连接
                    do_accept();
                });
        };

        // 开始第一次异步接受
        do_accept();
        
        // 运行io_context处理异步操作
        auto work = std::make_shared<io_context::work>(io_context_);
        io_context_.run();
    } catch (const std::exception& e) {
        // 这里通常不会执行，除非初始化失败
        std::cerr << "upper_node.cpp ====  upper server fatal exception: " << e.what() << std::endl;
    }
}


//TCP服务端接受客户端连接，接收解析客户端发送的JSON数据并保存为文件
static void tcpsUpperJsonThread()
{
    try {
        io_context io_context_;
        tcp::acceptor acceptor_(io_context_, tcp::endpoint(tcp::v4(), DEVICE_PORT+1));
        acceptor_.listen(1);

        tcp::socket socket_(io_context_);

        auto do_accept_ptr = std::make_shared<std::function<void()>>();
        *do_accept_ptr = [&, do_accept_ptr]() 
        {
            acceptor_.async_accept(socket_,
                [&, do_accept_ptr](const boost::system::error_code& error) {
                    if (!error)
                    {
                        std::string client_ip = socket_.remote_endpoint().address().to_string();
                        int client_port = socket_.remote_endpoint().port();
                        ROS_INFO("---[upper_node.cpp] jsonPort connection from %s:%d", client_ip.c_str(), client_port);

                        // 关键：创建do_read的智能指针
                        auto do_read_ptr = std::make_shared<std::function<void()>>();
                        *do_read_ptr = [&, do_accept_ptr, do_read_ptr]() 
                        {
                            auto buf = std::make_shared<std::vector<uint8_t>>(65535);
                            socket_.async_read_some(boost::asio::buffer(*buf),
                                [&, buf, do_accept_ptr, do_read_ptr](const boost::system::error_code& ec, size_t bytes_read) {
                                    if (!ec && bytes_read > 0) 
                                    {
                                        // 处理数据
                                        std::string data(buf->begin(), buf->begin() + bytes_read);
                                        ROS_INFO("---[upper_node.cpp] received json data %zu bytes\r\njson data:  %s", bytes_read,  data.c_str());

                                        //包路径的task文件夹
                                        std::string task_file_dir = ros::package::getPath("usvc_controller") + "/task/";
                                        
                                        // 确保目录存在
                                        if (!boost::filesystem::exists(task_file_dir)) 
                                            boost::filesystem::create_directories(task_file_dir);

                                        
                                        // 1. 将新数据追加到缓冲区
                                        g_json_buffer.insert(g_json_buffer.end(), data.begin(), data.end());
                                        
                                        // 2. 转换为字符串进行处理
                                        std::string json_str(g_json_buffer.begin(), g_json_buffer.end());
                                        
                                        // 3. 使用更可靠的JSON完整性检测
                                        size_t open_brace = json_str.find('{');
                                        if (open_brace == std::string::npos) 
                                        {
                                            // 没有找到起始大括号，清空缓冲区
                                            g_json_buffer.clear();
                                            return;
                                        }
                                        
                                        // 从第一个大括号开始处理
                                        json_str = json_str.substr(open_brace);
                                        g_json_buffer.erase(g_json_buffer.begin(), g_json_buffer.begin() + open_brace);
                                        
                                        // 4. 使用栈来匹配大括号，检测完整的JSON
                                        int brace_count = 0;
                                        size_t complete_end = 0;
                                        
                                        for (size_t i = 0; i < json_str.size(); ++i) 
                                        {
                                            if (json_str[i] == '{') 
                                            {
                                                brace_count++;
                                            } 
                                            else if (json_str[i] == '}') 
                                            {
                                                brace_count--;
                                                if (brace_count == 0) 
                                                {
                                                    complete_end = i + 1;
                                                    break;
                                                }
                                            }
                                        }
                                        
                                        // 5. 如果找到完整的JSON对象
                                        if (brace_count == 0 && complete_end > 0) 
                                        {
                                            std::string complete_json = json_str.substr(0, complete_end);
                                            
                                            try 
                                            {
                                                // 验证JSON格式
                                                std::istringstream iss(complete_json);
                                                boost::property_tree::ptree pt;
                                                boost::property_tree::read_json(iss, pt);
                                                
                                                // 保存
                                                std::string filename = task_file_dir + "task_file" + ".json";
                                                std::ofstream file(filename);
                                                
                                                if (file.is_open()) 
                                                {
                                                    boost::property_tree::write_json(file, pt, true);
                                                    file.close();
                                                    ROS_INFO("---[upper_node.cpp] json file saved to %s", filename.c_str());

                                                    std_msgs::Bool flag;
                                                    flag.data = true;
                                                    NewTaskFile_pub.publish(flag); //新的任务文件
                                                    
                                                    g_json_buffer.clear();
                                                } 
                                                else 
                                                {
                                                    ROS_ERROR("---[upper_node.cpp] json file open failed: %s", filename.c_str());
                                                }
                                            }
                                            catch (const std::exception& e) 
                                            {
                                                std::cerr << "upper_node.cpp ====  json processing error: " << e.what() << std::endl;
                                            }
                                        }
                                        
                                        // 6. 防止缓冲区无限增长
                                        if (g_json_buffer.size() > 1024 * 1024)// 1MB限制
                                        { 
                                            ROS_WARN("---[upper_node.cpp] json buffer too large, clearing to prevent memory issues");
                                            g_json_buffer.clear();
                                        }
                                        
                                        // 继续读取下一包
                                        (*do_read_ptr)();
                                    }
                                    else
                                    {
                                        ROS_INFO("---[upper_node.cpp] jsonPort connection ended");
                                        g_json_buffer.clear();
                                        socket_.close();

                                        (*do_accept_ptr)();
                                    }
                                });
                        };

                        (*do_read_ptr)();
                    }
                    else 
                    {
                        ROS_ERROR("---[upper_node.cpp] jsonPort accept error: %s", error.message().c_str());
                        
                        socket_.close();
                        (*do_accept_ptr)();
                    }
                });
        };
        

        (*do_accept_ptr)();
        io_context_.run();

    } catch (const std::exception& e) {
        std::cerr << "upper_node.cpp ====  json server fatal error: " << e.what() << std::endl;
    }
}


//雷达滤波数据
static void lidarFilterCallback(const sensor_msgs::PointCloud2& msg)
{
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 10 != 0)  // 雷达频率是10Hz，每10帧处理一次就是1秒
        return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *cloud);

    // 存储所有点和对应的距离
    std::vector<std::pair<float, Eigen::Vector3f>> points_with_distance;
    points_with_distance.reserve(cloud->points.size());

    for(const auto& point : cloud->points) 
    {
        Eigen::Vector3f p(point.x, point.y, point.z);
        // 计算到原点的距离（欧氏距离）
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        points_with_distance.emplace_back(distance, p);
    }
    
    // 按照距离排序（从小到大）
    std::sort(points_with_distance.begin(), points_with_distance.end(), 
              [](const std::pair<float, Eigen::Vector3f>& a, const std::pair<float, Eigen::Vector3f>& b) {
                  return a.first < b.first;
              });


    MDATA_SHIP_LidarData_t LidarData;
    int num_points = points_with_distance.size();
    // ROS_INFO("lidar:  %d",num_points);
    LidarData.nums = (num_points > 100) ? 100 : static_cast<uint8_t>(num_points);
    for (int i = 0; i < LidarData.nums; ++i) 
    {
        LidarData.x[i] = points_with_distance[i].second.x();
        LidarData.y[i] = points_with_distance[i].second.y();
        // std::cout<<" x:"<<lidar[i].x() <<" y:"<<lidar[i].y();
    }

    MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_LidarData, (uint8_t*)&LidarData, sizeof(MDATA_SHIP_LidarData_t));
} 

static void ACHDataCallback(const usvc_controller::ACHData& msg)
{
    // AHC频率是1Hz，理一次是1秒
    MDATA_SHIP_ACHData_t ACHData;
    ACHData.total_distance = msg.total_distance;
    ACHData.total_operating_hours = msg.total_operating_hours;

    MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_ACHData, (uint8_t*)&ACHData, sizeof(MDATA_SHIP_ACHData_t));
}

//ID
static void IDCallback(const usvc_controller::ID::ConstPtr& msg)
{
    MDATA_SHIP_ID_t SHIP_ID;

    SHIP_ID.flag = msg->flag;
    SHIP_ID.ID = msg->ID;

    MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_SetID, (uint8_t*)&SHIP_ID, sizeof(MDATA_SHIP_ID_t));
}

//RawData
static void RawDataCallback(const usvc_controller::RawData::ConstPtr& msg)
{

    uint16_t Obj = msg->obj;
	uint8_t* pBuf = const_cast<uint8_t*>(msg->data.data());
	uint16_t Len = msg->len;

    ROS_INFO("---[upper_node.cpp]  subscribe raw obj is 0x%04x:",Obj);

    MTLink_Send(&g_MTLink_upper, 0, 0, Obj, pBuf, Len);
}

//状态数据
static void StateDataCallback(const usvc_controller::StateData::ConstPtr& msg)
{
    MDATA_SHIP_StateData_t SHIP_StateData;
    memset(&SHIP_StateData, 0, sizeof(MDATA_SHIP_StateData_t));

    // 1. 传感器数据
    SHIP_StateData.sensor.humi = msg->humidity;
    SHIP_StateData.sensor.temp = msg->temperature;
    SHIP_StateData.sensor.leak[0] = msg->leak_detection[0];
    SHIP_StateData.sensor.leak[1] = msg->leak_detection[1];

    // 2. 电源状态
    SHIP_StateData.power.volage = msg->voltage;
    SHIP_StateData.power.current = msg->current;

    // 3. 电源管理状态
    SHIP_StateData.pms.Thruster = msg->Thruster;
    SHIP_StateData.pms.LEDScre = msg->LEDScre;
    SHIP_StateData.pms.unused2 = msg->unused2;
    SHIP_StateData.pms.unused3 = msg->unused3;

    // 4. 电池状态
    SHIP_StateData.battery.Q = msg->battery_q;
    SHIP_StateData.battery.V = msg->battery_v;
    SHIP_StateData.battery.A = msg->battery_a;
    SHIP_StateData.battery.T = msg->battery_t;

    // 5. 运行状态
    SHIP_StateData.run.runMode = msg->runMode;
    SHIP_StateData.run.Y = msg->y;
    SHIP_StateData.run.H = msg->h;
    SHIP_StateData.run.objSpeed = msg->objSpeed;
    SHIP_StateData.run.objHeading = msg->objHeading;
    SHIP_StateData.run.objPos[0] = msg->objPos[0];
    SHIP_StateData.run.objPos[1] = msg->objPos[1];
    SHIP_StateData.run.task_tepy = msg->task_tepy;
    SHIP_StateData.run.task_sta = msg->task_sta;
    SHIP_StateData.run.vision_state = msg->vision_state;
    SHIP_StateData.run.mapping_state = msg->mapping_state;

    // 6. 导航数据
    SHIP_StateData.navi.ins_sta = msg->ins_status;
    SHIP_StateData.navi.timeTick = msg->timestamp;
    
    SHIP_StateData.navi.gyro_roll = msg->gyro_roll;
    SHIP_StateData.navi.gyro_pitch = msg->gyro_pitch;
    SHIP_StateData.navi.gyro_heading = msg->gyro_heading;
    
    SHIP_StateData.navi.roll = msg->roll;
    SHIP_StateData.navi.pitch = msg->pitch;
    SHIP_StateData.navi.heading = msg->heading;
    
    SHIP_StateData.navi.acc_x = msg->acc_x;
    SHIP_StateData.navi.acc_y = msg->acc_y;
    SHIP_StateData.navi.acc_z = msg->acc_z;
    
    SHIP_StateData.navi.vel_x = msg->vel_x;
    SHIP_StateData.navi.vel_y = msg->vel_y;
    SHIP_StateData.navi.vel_z = msg->vel_z;
    
    SHIP_StateData.navi.latitude = msg->latitude;
    SHIP_StateData.navi.longitude = msg->longitude;
    SHIP_StateData.navi.altitude = msg->altitude;
    
    SHIP_StateData.navi.vel_n = msg->vel_n;
    SHIP_StateData.navi.vel_e = msg->vel_e;
    SHIP_StateData.navi.vel_d = msg->vel_d;
    
    SHIP_StateData.navi.satellite_count = msg->satellite_count;
    SHIP_StateData.navi.pos_sta = msg->pos_status;
    SHIP_StateData.navi.heading_sta = msg->heading_status;
    
    SHIP_StateData.navi.position_std_lat = msg->pos_std_lat;
    SHIP_StateData.navi.position_std_lon = msg->pos_std_lon;

    // 7. 推进器状态
    // 左推进器
    SHIP_StateData.thruster.left_Val = msg->left_thruster_val;
    SHIP_StateData.thruster.left_sta = msg->left_thruster_status;
    SHIP_StateData.thruster.left_duty_scaled = msg->left_duty_cycle;
    SHIP_StateData.thruster.left_rpm = msg->left_rpm;
    SHIP_StateData.thruster.left_MOST = msg->left_mos_temp;
    SHIP_StateData.thruster.left_total_current = msg->left_total_current;
    SHIP_StateData.thruster.left_current_in = msg->left_input_current;
    
    // 右推进器
    SHIP_StateData.thruster.right_Val = msg->right_thruster_val;
    SHIP_StateData.thruster.right_sta = msg->right_thruster_status;
    SHIP_StateData.thruster.right_duty_scaled = msg->right_duty_cycle;
    SHIP_StateData.thruster.right_rpm = msg->right_rpm;
    SHIP_StateData.thruster.right_MOST = msg->right_mos_temp;
    SHIP_StateData.thruster.right_total_current = msg->right_total_current;
    SHIP_StateData.thruster.right_current_in = msg->right_input_current;

    // 8. 灯状态
    SHIP_StateData.light.LED = msg->LED;
    SHIP_StateData.light.CautionLED = msg->CautionLED;

    // 9. 码头信息
    SHIP_StateData.dock.connect = msg->connect;
    SHIP_StateData.dock.dock_ID = msg->dock_ID;
    
    SHIP_StateData.dock.presetPos[0][0] = msg->preset_positions0[0];
    SHIP_StateData.dock.presetPos[0][1] = msg->preset_positions0[1];
    SHIP_StateData.dock.presetPos[1][0] = msg->preset_positions1[0];
    SHIP_StateData.dock.presetPos[1][1] = msg->preset_positions1[1];
    
    SHIP_StateData.dock.berth = msg->berth_status;
    SHIP_StateData.dock.change = msg->charge_status;
    
    MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_StateData, (uint8_t*)&SHIP_StateData, sizeof(MDATA_SHIP_StateData_t));
}


// enable_switches 启动恢复已上移到 control_node 总线（nh.param 读 config），
// 旧版由 upper_node 发布话题的方式随 service 化改造移除。


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "upper_node");
    
    ros::NodeHandle nh;

    nh.param<int>("upper/BROAD_PORT", BROAD_PORT, 5000);
    nh.param<int>("DEVICE_ID", DEVICE_ID, 0);
    nh.param<int>("DEVICE_PORT", DEVICE_PORT, 5001);
    nh.param<std::string>("DEVICE_TYPE", DEVICE_TYPE, "ship_calc");
    nh.param<std::string>("DEVICE_SN", DEVICE_SN, "NULL");

    //UDP广播
    std::thread thread_udpBroadcast(udpBroadcastThread); //启动线程
    //TCP服务端MTLink
    std::thread thread_tcpsUpperMTLinkAccept(tcpsUpperMTLinkAcceptThread); //启动线程
    //TCP服务端TaskFile
    std::thread thread_tcpsUpperJson(tcpsUpperJsonThread); //启动线程

    memset(&g_MTLink_upper, 0, sizeof(MTLink_t));
    MTLink_RegSendFunc(&g_MTLink_upper, upperTcpsSendBuffer, NULL);
    MTLink_RegReciveFunc(&g_MTLink_upper, upperTcpsRecvBuffer, NULL);

    UpperConect_pub   = nh.advertise<std_msgs::Bool>("/upper/connect",1);
    NewTaskFile_pub   = nh.advertise<std_msgs::Bool>("/upper/NewTaskFile", 10);

    ID_pub            = nh.advertise<usvc_controller::ID>("/upper/ID", 10); //
    RawData_pub       = nh.advertise<usvc_controller::RawData>("/upper/RawData", 10); //
    RunCtrl_pub       = nh.advertise<usvc_controller::RunCtrl>("/upper/RunCtrl", 10); //

    lidar_filter_sub = nh.subscribe("/sensor/lidar_filter", 10, lidarFilterCallback);
    ACHData_sub     = nh.subscribe("/ach/ACHData", 10, ACHDataCallback);
    ID_sub          = nh.subscribe("/ship/ID", 10, IDCallback);
    RawData_sub     = nh.subscribe("/ship/RawData", 10, RawDataCallback);
    StateData_sub   = nh.subscribe("/control/StateData", 10, StateDataCallback);
    
    lidar_param_client = nh.serviceClient<usvc_controller::UpdateParams>("lidar/update_params");
    sail_param_client = nh.serviceClient<usvc_controller::UpdateParams>("sail/update_params");
    back_param_client = nh.serviceClient<usvc_controller::UpdateParams>("back/update_params");
    path_param_client = nh.serviceClient<usvc_controller::UpdateParams>("path/update_params");
    enable_switches_client = nh.serviceClient<usvc_controller::UpdateParams>("enable_switches/update_params");
    tunable_param_client  = nh.serviceClient<usvc_controller::UpdateParams>("tunable/update_params");
    charge_param_client = nh.serviceClient<usvc_controller::UpdateParams>("charge/update_params");

    ros::Rate rate(100); // 100Hz
    while (ros::ok()) 
    {
        uint8_t SID;
		uint8_t DID;
		uint16_t Obj;
		uint8_t *pBuf;
		uint16_t Len;

        if(MTLink_Receive(&g_MTLink_upper, &SID, &DID, &Obj, &pBuf, &Len))
        {
            switch(Obj)
			{
                case OBJ_SHIP_BoardReset:
                case OBJ_SHIP_BoardInfo:
                case OBJ_SHIP_BoardVersion:
                case OBJ_SHIP_BoardNetwork:
                {
                    ROS_INFO("---[upper_node.cpp]  publish raw obj is 0x%04x:",Obj);
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

                    if(SHIP_ID.flag == 1)
                    {
                        //将yaml文件中的DEVICE_SN修改为本次接收到的sn并保存
                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";
                        ConfigManagerTool::updateParamInFile(config_file, "DEVICE_ID", SHIP_ID.ID);
                    }
                    
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
                {
                    std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";

                    ConfigManagerTool::updateParamInFile(config_file, "DEVICE_SN", "NULL");
                    MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_ResetSN, NULL, 0);
                    break;
                }

                case OBJ_SHIP_LidarParam:
                {
                    MDATA_SHIP_LidarParam_t SHIP_Lidar;
                    memcpy(&SHIP_Lidar, pBuf, Len);

                    if(SHIP_Lidar.flag == 0x01)
                    {
                        usvc_controller::UpdateParams srv;
                        srv.request.node_name = "sensor";

                        srv.request.param_names = {
                            "X_DISTANCE", 
                            "Y_DISTANCE", 
                            "Z_DISTANCE",
                            "ClusterTolerance",
                            "MinClusterSize",
                            "MaxClusterSize"
                        };

                        srv.request.param_values = {
                            std::to_string(SHIP_Lidar.X_DISTANCE),
                            std::to_string(SHIP_Lidar.Y_DISTANCE),
                            std::to_string(SHIP_Lidar.Z_DISTANCE),
                            std::to_string(SHIP_Lidar.ClusterTolerance),
                            std::to_string(SHIP_Lidar.MinClusterSize),
                            std::to_string(SHIP_Lidar.MaxClusterSize)
                        };

                        lidar_param_client.call(srv);

                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";

                        ConfigManagerTool::updateParamInFile(config_file, "sensor/lidar/X_DISTANCE", SHIP_Lidar.X_DISTANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "sensor/lidar/Y_DISTANCE", SHIP_Lidar.Y_DISTANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "sensor/lidar/Z_DISTANCE", SHIP_Lidar.Z_DISTANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "sensor/lidar/ClusterTolerance", SHIP_Lidar.ClusterTolerance);
                        ConfigManagerTool::updateParamInFile(config_file, "sensor/lidar/MinClusterSize", SHIP_Lidar.MinClusterSize);
                        ConfigManagerTool::updateParamInFile(config_file, "sensor/lidar/MaxClusterSize", SHIP_Lidar.MaxClusterSize);

                        SHIP_Lidar.flag = 0x11;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_LidarParam, (uint8_t*)&SHIP_Lidar, sizeof(MDATA_SHIP_LidarParam_t));
                    }
                    else if(SHIP_Lidar.flag == 0x03)
                    {
                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";

                        ConfigManagerTool::getParamFromFile(config_file, "sensor/lidar/X_DISTANCE", SHIP_Lidar.X_DISTANCE);
                        ConfigManagerTool::getParamFromFile(config_file, "sensor/lidar/Y_DISTANCE", SHIP_Lidar.Y_DISTANCE);
                        ConfigManagerTool::getParamFromFile(config_file, "sensor/lidar/Z_DISTANCE", SHIP_Lidar.Z_DISTANCE);
                        ConfigManagerTool::getParamFromFile(config_file, "sensor/lidar/ClusterTolerance", SHIP_Lidar.ClusterTolerance);
                        ConfigManagerTool::getParamFromFile(config_file, "sensor/lidar/MinClusterSize", SHIP_Lidar.MinClusterSize);
                        ConfigManagerTool::getParamFromFile(config_file, "sensor/lidar/MaxClusterSize", SHIP_Lidar.MaxClusterSize);
                    
                        SHIP_Lidar.flag = 0x31;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_LidarParam, (uint8_t*)&SHIP_Lidar, sizeof(MDATA_SHIP_LidarParam_t));
                    }
                    else
                    {
                        ;
                    }

                    break;
                }

                case OBJ_SHIP_SailParam:
                {
                    MDATA_SHIP_SailParam_t SHIP_Sail;
                    memcpy(&SHIP_Sail, pBuf, Len);

                    if(SHIP_Sail.flag == 0x01)
                    {
                        usvc_controller::UpdateParams srv;
                        srv.request.node_name = "sail";

                        srv.request.param_names = {
                            "SPEED", 
                            "Y", 
                            "TOLERANCERADIUS0",
                            "TOLERANCERADIUS1",
                            "POS0",
                            "POS1"
                        };

                        srv.request.param_values = {
                            std::to_string(SHIP_Sail.SPEED),
                            std::to_string(SHIP_Sail.Y),
                            std::to_string(SHIP_Sail.TOLERANCERADIUS[0]),
                            std::to_string(SHIP_Sail.TOLERANCERADIUS[1]),
                            std::to_string(SHIP_Sail.POS[0]),
                            std::to_string(SHIP_Sail.POS[1])
                        };

                        sail_param_client.call(srv);

                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";
                    
                        ConfigManagerTool::updateParamInFile(config_file, "control/sail/SPEED", SHIP_Sail.SPEED);
                        ConfigManagerTool::updateParamInFile(config_file, "control/sail/Y", SHIP_Sail.Y);
                        ConfigManagerTool::updateParamInFile(config_file, "control/sail/TOLERANCERADIUS1", SHIP_Sail.TOLERANCERADIUS[0]);
                        ConfigManagerTool::updateParamInFile(config_file, "control/sail/TOLERANCERADIUS2", SHIP_Sail.TOLERANCERADIUS[1]);
                        ConfigManagerTool::updateParamInFile(config_file, "control/sail/LAT", SHIP_Sail.POS[0]);
                        ConfigManagerTool::updateParamInFile(config_file, "control/sail/LON", SHIP_Sail.POS[1]);

                        SHIP_Sail.flag = 0x11;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_SailParam, (uint8_t*)&SHIP_Sail, sizeof(MDATA_SHIP_SailParam_t));
                    }
                    else if(SHIP_Sail.flag == 0x03)
                    {
                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";

                        ConfigManagerTool::getParamFromFile(config_file, "control/sail/SPEED", SHIP_Sail.SPEED);
                        ConfigManagerTool::getParamFromFile(config_file, "control/sail/Y", SHIP_Sail.Y);
                        ConfigManagerTool::getParamFromFile(config_file, "control/sail/TOLERANCERADIUS1", SHIP_Sail.TOLERANCERADIUS[0]);
                        ConfigManagerTool::getParamFromFile(config_file, "control/sail/TOLERANCERADIUS2", SHIP_Sail.TOLERANCERADIUS[1]);
                        ConfigManagerTool::getParamFromFile(config_file, "control/sail/LAT", SHIP_Sail.POS[0]);
                        ConfigManagerTool::getParamFromFile(config_file, "control/sail/LON", SHIP_Sail.POS[1]);

                        SHIP_Sail.flag = 0x31;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_SailParam, (uint8_t*)&SHIP_Sail, sizeof(MDATA_SHIP_SailParam_t));
                    }
                    else
                    {
                        ;
                    }
                    break;
                }

                case OBJ_SHIP_BackParam:
                {
                    MDATA_SHIP_BackParam_t SHIP_Back;
                    memcpy(&SHIP_Back, pBuf, Len);

                    if(SHIP_Back.flag == 0x01)
                    {
                        usvc_controller::UpdateParams srv;
                        srv.request.node_name = "back";

                        srv.request.param_names = {
                            "MAXSPEED", 
                            "MINSPEED", 
                            "LOSDISTANCE",
                            "OBSTACLE_INFLUENCE_DISTANCE",
                            "SAFE_DISTANCE",
                            "REPULSIVE_GAIN",
                            "ATTRACTIVE_GAIN",
                            "SIGMA",
                            "TOLERANCERADIUS",
                            "Y",
                            "POS0",
                            "POS1",
                        };

                        srv.request.param_values = {
                            std::to_string(SHIP_Back.MAXSPEED),
                            std::to_string(SHIP_Back.MINSPEED),
                            std::to_string(SHIP_Back.LOSDISTANCE),
                            std::to_string(SHIP_Back.OBSTACLE_INFLUENCE_DISTANCE),
                            std::to_string(SHIP_Back.SAFE_DISTANCE),
                            std::to_string(SHIP_Back.REPULSIVE_GAIN),
                            std::to_string(SHIP_Back.ATTRACTIVE_GAIN),
                            std::to_string(SHIP_Back.SIGMA),
                            std::to_string(SHIP_Back.TOLERANCERADIUS),
                            std::to_string(SHIP_Back.Y),
                            std::to_string(SHIP_Back.POS[0]),
                            std::to_string(SHIP_Back.POS[1]),
                        };

                        back_param_client.call(srv);

                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";
                    
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/MAXSPEED", SHIP_Back.MAXSPEED);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/MINSPEED", SHIP_Back.MINSPEED);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/LOSDISTANCE", SHIP_Back.LOSDISTANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/OBSTACLE_INFLUENCE_DISTANCE", SHIP_Back.OBSTACLE_INFLUENCE_DISTANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/SAFE_DISTANCE", SHIP_Back.SAFE_DISTANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/REPULSIVE_GAIN", SHIP_Back.REPULSIVE_GAIN);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/ATTRACTIVE_GAIN", SHIP_Back.ATTRACTIVE_GAIN);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/SIGMA", SHIP_Back.SIGMA);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/TOLERANCERADIUS", SHIP_Back.TOLERANCERADIUS);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/Y", SHIP_Back.Y);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/LAT", SHIP_Back.POS[0]);
                        ConfigManagerTool::updateParamInFile(config_file, "control/back/LON", SHIP_Back.POS[1]);

                        SHIP_Back.flag = 0x11;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_BackParam, (uint8_t*)&SHIP_Back, sizeof(MDATA_SHIP_BackParam_t));
                    }
                    else if(SHIP_Back.flag == 0x03)
                    {
                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";

                        ConfigManagerTool::getParamFromFile(config_file, "control/back/MAXSPEED", SHIP_Back.MAXSPEED);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/MINSPEED", SHIP_Back.MINSPEED);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/LOSDISTANCE", SHIP_Back.LOSDISTANCE);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/OBSTACLE_INFLUENCE_DISTANCE", SHIP_Back.OBSTACLE_INFLUENCE_DISTANCE);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/SAFE_DISTANCE", SHIP_Back.SAFE_DISTANCE);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/REPULSIVE_GAIN", SHIP_Back.REPULSIVE_GAIN);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/ATTRACTIVE_GAIN", SHIP_Back.ATTRACTIVE_GAIN);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/SIGMA", SHIP_Back.SIGMA);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/TOLERANCERADIUS", SHIP_Back.TOLERANCERADIUS);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/Y", SHIP_Back.Y);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/LAT", SHIP_Back.POS[0]);
                        ConfigManagerTool::getParamFromFile(config_file, "control/back/LON", SHIP_Back.POS[1]);

                        SHIP_Back.flag = 0x31;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_BackParam, (uint8_t*)&SHIP_Back, sizeof(MDATA_SHIP_BackParam_t));
                    }
                    else
                    {
                        ;
                    }
                    break;
                }

                case OBJ_SHIP_PathTrackParam:
                {
                    MDATA_SHIP_PathTrackParam_t SHIP_PathTrack;
                    memcpy(&SHIP_PathTrack, pBuf, Len);

                    if(SHIP_PathTrack.flag == 0x01)
                    {
                        usvc_controller::UpdateParams srv;
                        srv.request.node_name = "path";

                        srv.request.param_names = {
                            "MAXSPEED", 
                            "MINSPEED", 
                            "LOSDISTANCE",
                            "OBSTACLE_INFLUENCE_DISTANCE",
                            "SAFE_DISTANCE",
                            "REPULSIVE_GAIN",
                            "ATTRACTIVE_GAIN",
                            "SIGMA",
                            "TOLERANCERADIUS"
                        };

                        srv.request.param_values = {
                            std::to_string(SHIP_PathTrack.MAXSPEED),
                            std::to_string(SHIP_PathTrack.MINSPEED),
                            std::to_string(SHIP_PathTrack.LOSDISTANCE),
                            std::to_string(SHIP_PathTrack.OBSTACLE_INFLUENCE_DISTANCE),
                            std::to_string(SHIP_PathTrack.SAFE_DISTANCE),
                            std::to_string(SHIP_PathTrack.REPULSIVE_GAIN),
                            std::to_string(SHIP_PathTrack.ATTRACTIVE_GAIN),
                            std::to_string(SHIP_PathTrack.SIGMA),
                            std::to_string(SHIP_PathTrack.TOLERANCERADIUS),
                        };

                        path_param_client.call(srv);

                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";
                    
                        ConfigManagerTool::updateParamInFile(config_file, "control/path/MAXSPEED", SHIP_PathTrack.MAXSPEED);
                        ConfigManagerTool::updateParamInFile(config_file, "control/path/MINSPEED", SHIP_PathTrack.MINSPEED);
                        ConfigManagerTool::updateParamInFile(config_file, "control/path/LOSDISTANCE", SHIP_PathTrack.LOSDISTANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "control/path/OBSTACLE_INFLUENCE_DISTANCE", SHIP_PathTrack.OBSTACLE_INFLUENCE_DISTANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "control/path/SAFE_DISTANCE", SHIP_PathTrack.SAFE_DISTANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "control/path/REPULSIVE_GAIN", SHIP_PathTrack.REPULSIVE_GAIN);
                        ConfigManagerTool::updateParamInFile(config_file, "control/path/ATTRACTIVE_GAIN", SHIP_PathTrack.ATTRACTIVE_GAIN);
                        ConfigManagerTool::updateParamInFile(config_file, "control/path/SIGMA", SHIP_PathTrack.SIGMA);
                        ConfigManagerTool::updateParamInFile(config_file, "control/path/TOLERANCERADIUS", SHIP_PathTrack.TOLERANCERADIUS);

                        SHIP_PathTrack.flag = 0x11;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_PathTrackParam, (uint8_t*)&SHIP_PathTrack, sizeof(MDATA_SHIP_PathTrackParam_t));
                    }
                    else if(SHIP_PathTrack.flag == 0x03)
                    {
                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";

                        ConfigManagerTool::getParamFromFile(config_file, "control/path/MAXSPEED", SHIP_PathTrack.MAXSPEED);
                        ConfigManagerTool::getParamFromFile(config_file, "control/path/MINSPEED", SHIP_PathTrack.MINSPEED);
                        ConfigManagerTool::getParamFromFile(config_file, "control/path/LOSDISTANCE", SHIP_PathTrack.LOSDISTANCE);
                        ConfigManagerTool::getParamFromFile(config_file, "control/path/OBSTACLE_INFLUENCE_DISTANCE", SHIP_PathTrack.OBSTACLE_INFLUENCE_DISTANCE);
                        ConfigManagerTool::getParamFromFile(config_file, "control/path/SAFE_DISTANCE", SHIP_PathTrack.SAFE_DISTANCE);
                        ConfigManagerTool::getParamFromFile(config_file, "control/path/REPULSIVE_GAIN", SHIP_PathTrack.REPULSIVE_GAIN);
                        ConfigManagerTool::getParamFromFile(config_file, "control/path/ATTRACTIVE_GAIN", SHIP_PathTrack.ATTRACTIVE_GAIN);
                        ConfigManagerTool::getParamFromFile(config_file, "control/path/SIGMA", SHIP_PathTrack.SIGMA);
                        ConfigManagerTool::getParamFromFile(config_file, "control/path/TOLERANCERADIUS", SHIP_PathTrack.TOLERANCERADIUS);


                        SHIP_PathTrack.flag = 0x31;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_PathTrackParam, (uint8_t*)&SHIP_PathTrack, sizeof(MDATA_SHIP_PathTrackParam_t));
                    }
                    else
                    {
                        ;
                    }
                    break;
                }

                case OBJ_SHIP_RunCtrl:
                {
                    MDATA_SHIP_RunCtrl_t SHIP_RunCtrl;
                    memcpy(&SHIP_RunCtrl, pBuf, Len);

                    usvc_controller::RunCtrl msg;
                    msg.run_mode = SHIP_RunCtrl.runMode;
                    ROS_INFO("UPPER msg.run_mode %d", msg.run_mode);
                    msg.Y = SHIP_RunCtrl.Y;
                    msg.H = SHIP_RunCtrl.H;
                    msg.obj_speed = SHIP_RunCtrl.objSpeed;
                    msg.obj_heading = SHIP_RunCtrl.objHeading;
                    msg.is_avoiding = SHIP_RunCtrl.isAvoiding;
                    msg.has_dock = SHIP_RunCtrl.has_dock;
                    msg.dock_ip[0] = SHIP_RunCtrl.dock_ip[0];
                    msg.dock_ip[1] = SHIP_RunCtrl.dock_ip[1];
                    msg.dock_ip[2] = SHIP_RunCtrl.dock_ip[2];
                    msg.dock_ip[3] = SHIP_RunCtrl.dock_ip[3];
                    msg.dock_port = SHIP_RunCtrl.dock_port;
                    msg.task_state = SHIP_RunCtrl.task_state;
                    msg.task_tepy = SHIP_RunCtrl.task_teyp;
                    msg.ctrl_mode = SHIP_RunCtrl.ctrl_mode;

                    RunCtrl_pub.publish(msg);
                    break;
                }
                //使能开关
                case OBJ_SHIP_EnableSwitches:
                {
                    MDATA_SHIP_EnableSwitches_t SHIP_enables;

                    // 长度校验：Len 过短会读到脏数据，过长会栈溢出
                    if (Len < sizeof(MDATA_SHIP_EnableSwitches_t)) {
                        ROS_ERROR("---[upper_node.cpp] EnableSwitches len too short: %d < %zu", Len, sizeof(MDATA_SHIP_EnableSwitches_t));
                        break;
                    }
                    memcpy(&SHIP_enables, pBuf, Len);

                    std::string config_file = ros::package::getPath("usvc_controller") + "/config/config.yaml";

                    if(SHIP_enables.flag == 0x01)  // 设置参数
                    {
                        // 1. 调用 control_node 总线 service，由总线落内存并广播给各功能包
                        usvc_controller::UpdateParams srv;
                        srv.request.node_name = "enable_switches";
                        srv.request.param_names = {
                            "mapping_en",
                            "back_lowbatt_en",
                            "back_linklost_en",
                            "vision_en"
                        };
                        srv.request.param_values = {
                            std::to_string(SHIP_enables.mapping_en),
                            std::to_string(SHIP_enables.back_lowbatt_en),
                            std::to_string(SHIP_enables.back_linklost_en),
                            std::to_string(SHIP_enables.vision_en)
                        };
                        enable_switches_client.call(srv);
                        //上位机发送的参数写入yaml文件
                        ConfigManagerTool::updateParamInFile(config_file, "enable_switches/mapping_en",       (int)SHIP_enables.mapping_en);
                        ConfigManagerTool::updateParamInFile(config_file, "enable_switches/back_lowbatt_en",  (int)SHIP_enables.back_lowbatt_en);
                        ConfigManagerTool::updateParamInFile(config_file, "enable_switches/back_linklost_en", (int)SHIP_enables.back_linklost_en);
                        ConfigManagerTool::updateParamInFile(config_file, "enable_switches/vision_en",        (int)SHIP_enables.vision_en);

                        SHIP_enables.flag = 0x11;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_EnableSwitches, (uint8_t*)&SHIP_enables, sizeof(MDATA_SHIP_EnableSwitches_t));
                    }
                    else if(SHIP_enables.flag == 0x03)  // 查询参数
                    {
                        int mapping_en = 0, back_lowbatt_en = 0, back_linklost_en = 0, vision_en = 0;
                        //上位机读取yaml文件
                        ConfigManagerTool::getParamFromFile(config_file, "enable_switches/mapping_en",       mapping_en);
                        ConfigManagerTool::getParamFromFile(config_file, "enable_switches/back_lowbatt_en",  back_lowbatt_en);
                        ConfigManagerTool::getParamFromFile(config_file, "enable_switches/back_linklost_en", back_linklost_en);
                        ConfigManagerTool::getParamFromFile(config_file, "enable_switches/vision_en",        vision_en);

                        SHIP_enables.mapping_en       = (uint8_t)mapping_en;
                        SHIP_enables.back_lowbatt_en  = (uint8_t)back_lowbatt_en;
                        SHIP_enables.back_linklost_en = (uint8_t)back_linklost_en;
                        SHIP_enables.vision_en        = (uint8_t)vision_en;

                        SHIP_enables.flag = 0x31;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_EnableSwitches, (uint8_t*)&SHIP_enables, sizeof(MDATA_SHIP_EnableSwitches_t));
                    }
                    else
                    {
                        ;
                    }
                    break;

                }

                case OBJ_SHIP_TunableParam:
                {
                    MDATA_SHIP_TunableParam_t SHIP_tunable;

                    if (Len < sizeof(MDATA_SHIP_TunableParam_t)) {
                        ROS_ERROR("---[upper_node.cpp] TunableParam len too short: %d < %zu", Len, sizeof(MDATA_SHIP_TunableParam_t));
                        break;
                    }
                    memcpy(&SHIP_tunable, pBuf, Len);

                    std::string config_file = ros::package::getPath("usvc_controller") + "/config/config.yaml";

                    if(SHIP_tunable.flag == 0x01)  // 设置参数
                    {
                        usvc_controller::UpdateParams srv;
                        srv.request.node_name = "tunable";
                        srv.request.param_names = {
                            // ========== usvc_planner 参数 ==========
                            // --- LOS 参数 ---
                             "target_speed",
                            "los_distance", "waypoint_tolerance",
                            "min_speed_turn", "adaptive_los_scale",
                            // --- 掩码参数 ---
                            "safe_margin", "boom_x_max",
                            // ========== visual_track 参数 ==========
                            "conf_thresh", "max_heading_offset", "approach_speed",
                            "heading_alpha", "reached_area_thresh",
                            "bottom_thresh", "slow_area"
                        };
                        srv.request.param_values = {
                            // ========== usvc_planner 参数 ==========
                            // --- LOS 参数 ---
                            std::to_string(SHIP_tunable.TARGET_SPEED),
                            std::to_string(SHIP_tunable.LOS_DISTANCE),
                            std::to_string(SHIP_tunable.WAYPOINT_TOLERANCE),
                            std::to_string(SHIP_tunable.MIN_SPEED_TURN),
                            std::to_string(SHIP_tunable.ADAPTIVE_LOS_SCALE),
                            // --- 掩码参数 ---
                            std::to_string(SHIP_tunable.SAFE_MARGIN),
                            std::to_string(SHIP_tunable.BOOM_X_MAX),
                            // ========== visual_track 参数 ==========
                            std::to_string(SHIP_tunable.CONF_THRESH),
                            std::to_string(SHIP_tunable.MAX_HEADING_OFFSET),
                            std::to_string(SHIP_tunable.APPROACH_SPEED),
                            std::to_string(SHIP_tunable.HEADING_ALPHA),
                            std::to_string(SHIP_tunable.REACHED_AREA_THRESH),
                            std::to_string(SHIP_tunable.BOTTOM_THRESH),
                            std::to_string(SHIP_tunable.SLOW_AREA)
                        };
                        tunable_param_client.call(srv);
                        // 上位机发送的参数写入yaml文件（冷启动恢复用）
                        // ========== usvc_planner 参数 ==========
                        // --- LOS 参数 ---
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/target_speed",      SHIP_tunable.TARGET_SPEED);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/los_distance",      SHIP_tunable.LOS_DISTANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/waypoint_tolerance",SHIP_tunable.WAYPOINT_TOLERANCE);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/min_speed_turn",    SHIP_tunable.MIN_SPEED_TURN);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/adaptive_los_scale",SHIP_tunable.ADAPTIVE_LOS_SCALE);
                        // --- 掩码参数 ---
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/safe_margin",       SHIP_tunable.SAFE_MARGIN);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/boom_x_max",        SHIP_tunable.BOOM_X_MAX);
                        // ========== visual_track 参数 ==========
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/conf_thresh",       SHIP_tunable.CONF_THRESH);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/max_heading_offset",SHIP_tunable.MAX_HEADING_OFFSET);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/approach_speed",    SHIP_tunable.APPROACH_SPEED);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/heading_alpha",     SHIP_tunable.HEADING_ALPHA);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/reached_area_thresh",SHIP_tunable.REACHED_AREA_THRESH);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/bottom_thresh",     SHIP_tunable.BOTTOM_THRESH);
                        ConfigManagerTool::updateParamInFile(config_file, "tunable/slow_area",         SHIP_tunable.SLOW_AREA);

                        SHIP_tunable.flag = 0x11;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_TunableParam, (uint8_t*)&SHIP_tunable, sizeof(MDATA_SHIP_TunableParam_t));
                    }
                    else if(SHIP_tunable.flag == 0x03)  // 查询参数
                    {
                        // ========== usvc_planner 参数 ==========
                        // --- LOS 参数 ---
                        double target_speed=0, los_distance=0, waypoint_tolerance=0, min_speed_turn=0, adaptive_los_scale=0;
                        // --- 掩码参数 ---
                        double safe_margin=0, boom_x_max=0;
                        // ========== visual_track 参数 ==========
                        double conf_thresh=0, max_heading_offset=0, approach_speed=0;
                        double heading_alpha=0, reached_area_thresh=0;
                        double bottom_thresh=0, slow_area=0;
                        
                        // ========== usvc_planner 参数 ==========
                        // --- LOS 参数 ---
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/target_speed",      target_speed);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/los_distance",      los_distance);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/waypoint_tolerance",waypoint_tolerance);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/min_speed_turn",    min_speed_turn);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/adaptive_los_scale",adaptive_los_scale);
                        // --- 掩码参数 ---
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/safe_margin",       safe_margin);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/boom_x_max",        boom_x_max);
                        // ========== visual_track 参数 ==========
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/conf_thresh",       conf_thresh);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/max_heading_offset",max_heading_offset);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/approach_speed",    approach_speed);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/heading_alpha",     heading_alpha);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/reached_area_thresh",reached_area_thresh);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/bottom_thresh",     bottom_thresh);
                        ConfigManagerTool::getParamFromFile(config_file, "tunable/slow_area",         slow_area);

                        // ========== usvc_planner 参数 ==========
                        // --- LOS 参数 ---
                        SHIP_tunable.TARGET_SPEED = target_speed;
                        SHIP_tunable.LOS_DISTANCE = los_distance;
                        SHIP_tunable.WAYPOINT_TOLERANCE = waypoint_tolerance;
                        SHIP_tunable.MIN_SPEED_TURN = min_speed_turn;
                        SHIP_tunable.ADAPTIVE_LOS_SCALE = adaptive_los_scale;
                        // --- 掩码参数 ---
                        SHIP_tunable.SAFE_MARGIN = safe_margin;
                        SHIP_tunable.BOOM_X_MAX = boom_x_max;
                        // ========== visual_track 参数 ==========
                        SHIP_tunable.CONF_THRESH = conf_thresh;
                        SHIP_tunable.MAX_HEADING_OFFSET = max_heading_offset;
                        SHIP_tunable.APPROACH_SPEED = approach_speed;
                        SHIP_tunable.HEADING_ALPHA = heading_alpha;
                        SHIP_tunable.REACHED_AREA_THRESH = reached_area_thresh;
                        SHIP_tunable.BOTTOM_THRESH = bottom_thresh;
                        SHIP_tunable.SLOW_AREA = slow_area;

                        SHIP_tunable.flag = 0x31;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_TunableParam, (uint8_t*)&SHIP_tunable, sizeof(MDATA_SHIP_TunableParam_t));
                    }
                    else
                    {
                        ;
                    }
                    break;
                }

                case OBJ_SHIP_ChargeParam:
                {
                    MDATA_SHIP_ChargeParam_t SHIP_Charge;
                    memcpy(&SHIP_Charge, pBuf, Len);

                    if(SHIP_Charge.flag == 0x01)
                    {
                        usvc_controller::UpdateParams srv;
                        srv.request.node_name = "charge";

                        srv.request.param_names = {
                            "LOW_BATTERY", 
                            "CHARGE_START", 
                            "CHARGE_STOP"
                        };

                        srv.request.param_values = {
                            std::to_string(SHIP_Charge.LOW_BATTERY),
                            std::to_string(SHIP_Charge.CHARGE_START),
                            std::to_string(SHIP_Charge.CHARGE_STOP)
                        };

                        charge_param_client.call(srv);

                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";

                        ConfigManagerTool::updateParamInFile(config_file, "charge/LOW_BATTERY", SHIP_Charge.LOW_BATTERY);
                        ConfigManagerTool::updateParamInFile(config_file, "charge/CHARGE_START", SHIP_Charge.CHARGE_START);
                        ConfigManagerTool::updateParamInFile(config_file, "charge/CHARGE_STOP",  SHIP_Charge.CHARGE_STOP);

                        SHIP_Charge.flag = 0x11;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_ChargeParam, (uint8_t*)&SHIP_Charge, sizeof(MDATA_SHIP_ChargeParam_t));
                    }
                    else if(SHIP_Charge.flag == 0x03)
                    {
                        std::string config_file  = ros::package::getPath("usvc_controller") + "/config/config.yaml";

                        ConfigManagerTool::getParamFromFile(config_file, "charge/LOW_BATTERY", SHIP_Charge.LOW_BATTERY);
                        ConfigManagerTool::getParamFromFile(config_file, "charge/CHARGE_START", SHIP_Charge.CHARGE_START);
                        ConfigManagerTool::getParamFromFile(config_file, "charge/CHARGE_STOP",  SHIP_Charge.CHARGE_STOP);

                        SHIP_Charge.flag = 0x31;
                        MTLink_Send(&g_MTLink_upper, 0, 0, OBJ_SHIP_ChargeParam, (uint8_t*)&SHIP_Charge, sizeof(MDATA_SHIP_ChargeParam_t));
                    }
                    else
                    {
                        ;
                    }
                    break;
                }
            

                case OBJ_SHIP_PMSCtrl:
                case OBJ_SHIP_MotorCtrl:
                case OBJ_SHIP_LightCtrl:
                case OBJ_SHIP_BerthCtrl:
                case OBJ_SHIP_ChangeCtrl:
                { 
                    //工控机不处理以上指令，不做数据解析，将原始数据直接发布给主控
                    ROS_INFO("---[upper_node.cpp]  publish raw obj is 0x%04x:",Obj);

                    usvc_controller::RawData msg;
                    msg.obj = Obj;
                    msg.len = Len;
                    msg.data.assign(pBuf, pBuf + Len);

                    RawData_pub.publish(msg);
                    break;
                }
                default:
                    break;
            }
        }


        /******************************************************************************************/
        /******************************************************************************************/
        // 清理断开连接的客户端
        std::vector<size_t> to_remove;
        {
            std::lock_guard<std::mutex> lock(upper_clients_to_remove_mutex);
            to_remove = upper_clients_to_remove;
            upper_clients_to_remove.clear();  // 清空
        }

        if (!to_remove.empty()) 
        {
            std::lock_guard<std::mutex> lock(upper_clients_mutex);
            
            // 从后往前删除
            for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) 
            {
                if (*it < upper_clients.size()) 
                {
                    upper_clients.erase(upper_clients.begin() + *it);
                }
            }
        }

        ros::spinOnce();    
        rate.sleep();
    }

    
    if (thread_udpBroadcast.joinable()) {
        thread_udpBroadcast.join();
    }
    
    if (thread_tcpsUpperMTLinkAccept.joinable()) {
        thread_tcpsUpperMTLinkAccept.detach();
    }
    
    if (thread_tcpsUpperJson.joinable()) {
        thread_tcpsUpperJson.detach();
    }
}
