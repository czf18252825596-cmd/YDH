#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <thread>
#include <functional>
#include <ros/package.h>  // 用于获取包路径
#include <unistd.h>       // 用于获取当前路径
#include <string>

#include "config_manager_tool/config_manager_tool.h"

using namespace boost::asio;
using namespace boost::asio::ip;
using boost::asio::buffer;

static std::atomic<uint8_t> listen_flag(0); 

static std::mutex listen_mutex;
static int listend_id;
static std::string listend_ip;
static int listend_port;
static std::string listend_type;
static std::string listend_sn;

static int SERVE_PORT;

static int PORT;
static std::string SN;

static std::atomic<uint8_t> tcpc_connect_flag(0); //连接标志 0_无连接 1_连接 
// 全局socket
static std::mutex tcpc_socket_mutex;
static std::shared_ptr<tcp::socket> tcpc_socket;

static std::atomic<uint8_t> tcpc_server_ready(false); 
static std::string tcpc_server_ip;
static int tcpc_server_port;

static std::mutex tcps_clients_mutex; 
static std::vector<std::shared_ptr<tcp::socket>> tcps_clients; //上位机客户端连接池
static std::mutex tcps_clients_to_remove_mutex;
static std::vector<size_t> tcps_clients_to_remove; //上位机客户端连接移除标志

// TCPS发送，用于注册MTLink
static bool tcpsSendBuffer(uint8_t* buf, uint16_t len) 
{
    bool state = false;

    std::vector<std::shared_ptr<tcp::socket>> current_clients;
        
    // 复制客户端列表
    {
        std::lock_guard<std::mutex> lock(tcps_clients_mutex);
        current_clients = tcps_clients;
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
static bool tcpsRecvBuffer(uint8_t* buf, uint16_t* len) 
{
    bool state = false;

    std::vector<std::shared_ptr<tcp::socket>> current_clients;
        
    // 复制客户端列表
    {
        std::lock_guard<std::mutex> lock(tcps_clients_mutex);
        current_clients = tcps_clients;
    }
        
        
    for (size_t i = 0; i < current_clients.size(); ++i) 
    {
        auto& socket = current_clients[i];

        if (!socket->is_open()) 
        {
            //标记要移除的连接
            {
                std::lock_guard<std::mutex> lock(tcps_clients_to_remove_mutex);
                tcps_clients_to_remove.push_back(i);
            }

            continue;
        }
            

        boost::system::error_code ec;
        size_t bytes_read = socket->read_some(buffer(buf, 2048), ec);
        *len = static_cast<uint16_t>(bytes_read);
        state |= (ec?false:true); 

        if(ec && ec != boost::asio::error::would_block)
        {
            ROS_INFO("---[peripheral_node.cpp] peripheral Client disconnected");
            socket->close();
            //标记要移除的连接
            {
                std::lock_guard<std::mutex> lock(tcps_clients_to_remove_mutex);
                tcps_clients_to_remove.push_back(i);
            }
        }
    }
    
    return state;
}

// TCPC发送
static bool tcpcSendBuffer(uint8_t* buf, uint16_t len) 
{
    bool state = false;

    std::lock_guard<std::mutex> lock(tcpc_socket_mutex);
    
    if (tcpc_socket && tcpc_socket->is_open()) 
    {
        boost::system::error_code ec;
        size_t bytes_sent = tcpc_socket->write_some(boost::asio::buffer(buf, len), ec);
        state |= (ec?false:true); 
    }

    return state;
}


// TCPC接收
static bool tcpcRecvBuffer(uint8_t* buf, uint16_t* len) 
{
    bool state = false;

    std::shared_ptr<tcp::socket> socket;
    {
        std::lock_guard<std::mutex> lock(tcpc_socket_mutex);
        socket = tcpc_socket;  // 复制shared_ptr
    }


    if (!socket || !socket->is_open()) 
    {
        if(tcpc_connect_flag == 1)
        {
             tcpc_connect_flag = 0;  // 触发重连
        }
           

        return false;
    }
    
    boost::system::error_code ec;
    size_t bytes_read = socket->read_some(boost::asio::buffer(buf, 2048), ec);
    *len = static_cast<uint16_t>(bytes_read);
    state |= (ec?false:true); 

    if(ec && ec != boost::asio::error::would_block)
    {
        if(tcpc_connect_flag == 1)
        {
            tcpc_connect_flag = 0;  // 触发重连
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
        udp::endpoint endpoint_(udp::v4(), PORT);
        
        // 配置套接字
        socket_.open(endpoint_.protocol());
        socket_.set_option(socket_base::reuse_address(true));
        socket_.set_option(socket_base::broadcast(true)); // 如果需要接收广播
        socket_.bind(endpoint_);
        socket_.non_blocking(true); // 设置非阻塞模式

        ROS_INFO("---[peripheral_node.cpp] udp listener started on port %d", PORT);
        
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

                        if(type.find("peripheral_ship_") != std::string::npos)
                        {
                            std::lock_guard<std::mutex> lock(tcpc_socket_mutex);
                            {
                                listend_id = id;
                                listend_ip = ip;
                                listend_port = port;
                                listend_type = type;
                                listend_sn = sn;
                            }

                            if(SN == "NULL")
                            {
                                ROS_INFO("---[peripheral_node.cpp] first get ship sn");
                                tcpc_server_ip = ip;
                                tcpc_server_port = port;

                                tcpc_server_ready = true;

                                //将yaml文件中的SN修改为本次接收到的sn并保存
                                std::string package_path = ros::package::getPath("usvc_controller");
                                std::string config_file  = package_path + "/config/config.yaml";

                                if (ConfigManagerTool::updateParamInFile(config_file, "peripheral/SN", sn))
                                {
                                    // 更新内存中的SN值
                                    SN = sn;
                                    ROS_INFO("---[peripheral_node.cpp] saved ship sn is %s", SN.c_str());
                                }

                                listen_flag = 1;
                            }
                            else
                            {
                                if(sn == SN)
                                {
                                    tcpc_server_ip = ip;
                                    tcpc_server_port = port;

                                    tcpc_server_ready = true;

                                    listen_flag = 1;
                                }
                            }
                        }
                    }catch (const std::exception& e) {
                        std::cerr<< "peripheral_node.cpp ==== failed to parse broadcast message: " << e.what() << std::endl;
                    }
                }

                // 如果没有休眠，给其他线程执行机会
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

            } catch (const std::exception& e) {
                std::cerr<< "peripheral_node.cpp ==== Exception in UDP listener: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }catch (const std::exception& e) {
        std::cerr<< "peripheral_node.cpp ==== Failed to initialize UDP listener on port " << PORT << ": " << e.what() << std::endl;
        return; // 初始化失败直接返回
    }
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
        socket_.connect(boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::broadcast(), PORT));
        auto local_endpoint = socket_.local_endpoint();
        local_ip = local_endpoint.address().to_string();   
        ROS_INFO("---[peripheral_node.cpp] udpBroadcast->local_ip is %s", local_ip.c_str());

        
        // 创建广播地址
        while (ros::ok()) 
        {
            try {
                if(listen_flag == 1)
                {
                    int id;
                    std::string ip;
                    int port;
                    std::string type;
                    std::string sn;
                    std::lock_guard<std::mutex> lock(tcpc_socket_mutex);
                    {
                        id = listend_id;
                        ip = local_ip;
                        port = SERVE_PORT;
                        type = listend_type.substr(std::string("peripheral_").length());
                        sn = listend_sn;
                    }

                    // 创建JSON格式的消息
                    std::string json_str = "{\"id\":" + std::to_string(id) + 
                                ",\"ip\":\"" + ip + 
                                "\",\"port\":" + std::to_string(port) + 
                                ",\"type\":\"" + type + 
                                "\",\"sn\":\"" + sn + "\"}";

                    // 发送消息
                    size_t bytes_sent = socket_.send(buffer(json_str));

                    listen_flag = 0;
                }
                        
            } catch (const std::exception& e) {
                std::cerr << "peripheral_node.cpp ==== Error sending broadcast: " << e.what() << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (const std::exception& e) {
        std::cerr << "peripheral_node.cpp ==== Failed to initialize broadcast socket: " << e.what() << std::endl;
        return; // 初始化失败直接返回
    }
}

//TCPc连接线程
static void tcpcConnectThread(void)
{
    auto io_context = std::make_shared<boost::asio::io_context>();

    while (ros::ok()) 
    {  
        if(tcpc_connect_flag != 1)
        {
            if(tcpc_server_ready)
            {
                try {
                     // 创建新socket
                    auto new_socket = std::make_shared<tcp::socket>(*io_context);
                    new_socket->open(tcp::v4());
                    new_socket->non_blocking(true);  // 设置非阻塞

                    tcp::endpoint endpoint(address::from_string(tcpc_server_ip), tcpc_server_port);

                    boost::system::error_code ec;
                    new_socket->connect(endpoint, ec);

                    if (!ec) 
                    {
                        {
                            std::lock_guard<std::mutex> lock(tcpc_socket_mutex);
                            
                            // 关闭旧的socket
                            if(tcpc_socket && tcpc_socket->is_open())
                            {
                                tcpc_socket->close();
                            }
                            
                            // 设置新的socket
                            tcpc_socket = new_socket;
                        } 
                        // 立即连接成功
                        tcpc_connect_flag = 1;
                        ROS_INFO("---[peripheral_node.cpp] peripheral connected to server %s:%d", tcpc_server_ip.c_str(), tcpc_server_port);
                    } 
                    else if (ec == boost::asio::error::in_progress) 
                    {
                        // 连接进行中，等待一下再检查
                        ROS_INFO("---[peripheral_node.cpp] peripheral connection in progress...");
                        tcpc_connect_flag = 0;  // 保持连接标志，下次继续尝试
                    }
                    else
                    {
                        // 连接失败
                        ROS_WARN("---[peripheral_node.cpp] peripheral failed to connect: %s", ec.message().c_str());
                        std::this_thread::sleep_for(std::chrono::seconds(5));
                        tcpc_connect_flag = 0;  // 保持连接标志，下次继续尝试
                    }
                }catch (const boost::system::system_error& e) {
                    // 连接失败（服务没开、网络不通等）
                    std::cerr<< "peripheral_node.cpp ==== peripheral connect fail: " << e.what() << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    tcpc_connect_flag = 0;  // 保持连接标志，下次继续尝试
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

//TCP服务端接受客户端连接线程，支持多客户端连接
static void tcpsAcceptThread()
{
    try {
        io_context io_context_;
        tcp::acceptor acceptor_(io_context_, tcp::endpoint(tcp::v4(), SERVE_PORT));

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
                        
                        ROS_INFO("---[peripheral_node.cpp] peripheral new connection from %s:%d", client_ip.c_str(), client_port);
                        
                        // 设置为非阻塞，方便主循环读取
                        new_socket->non_blocking(true);
                        
                        // 添加到全局客户端列表
                        {
                            std::lock_guard<std::mutex> lock(tcps_clients_mutex);
                            tcps_clients.push_back(new_socket);
                        }
                    } 
                    else 
                    {
                        ROS_ERROR("---[peripheral_node.cpp] peripheral accept error: %s", error.message().c_str());
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
        std::cerr << "peripheral_node.cpp ====  peripheral server fatal exception: " << e.what() << std::endl;
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "peripheral_node");
    ros::NodeHandle nh;

    int DEVICE_PORT;
    nh.param<int>("DEVICE_PORT", DEVICE_PORT, 5001);
    nh.param<int>("peripheral/PORT", PORT, 2000);
    nh.param<std::string>("peripheral/SN", SN, "NULL");

    SERVE_PORT = DEVICE_PORT + 2;

    std::thread thread_udpListen(udpListenThread); //启动线程
    std::thread thread_udpBroadcast(udpBroadcastThread); //启动线程
    std::thread thread_tcpsAccept(tcpsAcceptThread); //启动线程
    std::thread thread_tcpcConnect(tcpcConnectThread);

    ros::Rate rate(100); // 100Hz
    while (ros::ok()) 
    {
        uint8_t tcps_buf[1024];
        uint16_t tcps_len;
        if( tcpsRecvBuffer(tcps_buf,&tcps_len) )
        {
            tcpcSendBuffer(tcps_buf, tcps_len);
        }

        uint8_t tcpc_buf[1024];
        uint16_t tcpc_len;
        if( tcpcRecvBuffer(tcpc_buf,&tcpc_len) )
        {
            tcpsSendBuffer(tcpc_buf, tcpc_len);
        }

        /******************************************************************************************/
        /******************************************************************************************/
        // 清理断开连接的客户端
        std::vector<size_t> to_remove;
        {
            std::lock_guard<std::mutex> lock(tcps_clients_to_remove_mutex);
            to_remove = tcps_clients_to_remove;
            tcps_clients_to_remove.clear();  // 清空
        }

        if (!to_remove.empty()) 
        {
            std::lock_guard<std::mutex> lock(tcps_clients_mutex);
            
            // 从后往前删除
            for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) 
            {
                if (*it < tcps_clients.size()) 
                {
                    tcps_clients.erase(tcps_clients.begin() + *it);
                }
            }
        }

        ros::spinOnce();    

        ros::spinOnce();    
        rate.sleep();
    }
    
    return 0;
}