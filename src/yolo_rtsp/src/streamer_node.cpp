/**
 * streamer_node.cpp - ROS1主节点
 *
 * 功能：协调所有模块工作
 *   1. 读取360摄像机的RTSP流
 *   2. 启动Python子进程进行YOLO推理
 *   3. 推两路RTSP流（原始+标注）
 *   4. 发布ROS话题供其他节点使用
 *
 * 数据流：
 *   360摄像机 → C++读帧 → Python推理 → 标注画面
 *            ↘ 推raw流      ↘ 推annotated流
 *            ↘ 发ROS话题     ↘ 发检测结果话题
 *
 * 关键参数：
 *   - rtsp_url: 360摄像机的RTSP地址
 *   - rtsp_port: RTSP推流端口（8554是mediamtx默认端口）
 *   - model_path: YOLO模型文件路径
 *   - conf_thresh: 置信度阈值（低于此值的检测框被丢弃）
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <cstdlib>
#include <signal.h>

// socket相关头文件：用于开机自检mediaMTX服务器是否在运行
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>

#include "yolo_rtsp/python_bridge.h"
#include "yolo_rtsp/rtsp_pusher.h"
#include "yolo_rtsp/DetectionArray.h"

using namespace yolo_rtsp;
// ========== 全局状态 ==========
// ROS服务回调必须是独立的函数，拿不到main函数里的局部变量，
// 所以回调要修改的开关只能放全局
static bool g_running = false;  // 主循环运行开关，false时主循环退出

/**
 * 探测某个TCP端口是否有服务在监听（1秒超时）
 * 用途：启动时检查mediaMTX(8554)是否已启动，没启动就提前大声报警，
 *       避免推流失败后一头雾水
 * @return 端口可达返回true
 */
static bool probeTcpPort(const char* ip, int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;

    // 设为非阻塞再connect，配合select实现1秒超时
    // （直接connect阻塞式连接时，对方不存在一般立即失败，但保险起见加超时）
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &addr.sin_addr);

    int r = connect(fd, (sockaddr*)&addr, sizeof(addr));
    bool ok = false;
    if (r == 0)
    {
        ok = true;  // 立即连上
    }
    else if (errno == EINPROGRESS)
    {
        // 连接进行中，等最多1秒
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(fd, &wfds);
        timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        if (select(fd + 1, nullptr, &wfds, nullptr, &tv) > 0)
        {
            int err = 0;
            socklen_t len = sizeof(err);
            getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
            ok = (err == 0);
        }
    }
    close(fd);
    return ok;
}

/**
 * /yolo/enable 话题回调（由 controller 的 vision_en 并入驱动）
 * 功能：true=启动检测+推流，false=停止
 */
static void enableCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data && !g_running)
    {
        g_running = true;
        ROS_INFO("Streamer enabled by /yolo/enable");
    }
    else if (!msg->data && g_running)
    {
        g_running = false;
        ROS_INFO("Streamer disabled by /yolo/enable");
    }
}

/**
 * /yolo/start 服务回调 (兼容保留)
 */
static bool startYoloCallback(std_srvs::Empty::Request& req,
                               std_srvs::Empty::Response& res)
{
    g_running = true;  // 打开主循环开关
    ROS_INFO("Streamer started");
    return true;
}

/**
 * /yolo/stop 服务回调
 * 功能：停止检测和推流
 * 调用方式：rosservice call /yolo/stop
 */
static bool stopYoloCallback(std_srvs::Empty::Request& req,
                              std_srvs::Empty::Response& res)
{
    g_running = false;  // 关闭主循环开关，主循环会退出
    ROS_INFO("Streamer stopped");
    return true;
}

int main(int argc, char** argv)
{
    // 忽略SIGPIPE（ffmpeg管道断开时不会杀死进程）
    signal(SIGPIPE, SIG_IGN);

    // ========== 1. ROS初始化 ==========
    // ros::init会初始化ROS系统，创建节点
    // 第三个参数"streamer_node"是节点名，可以用rosnode list查看
    ros::init(argc, argv, "streamer_node");
    ros::NodeHandle nh;       // 全局句柄：用于发布话题、注册服务
    ros::NodeHandle pnh("~"); // 私有句柄：用于读取launch文件中的<param>
                              // "~"代表当前节点的命名空间
    // // ===== 自动启动 mediaMTX =====
    // {   // 检查 mediaMTX 是否已运行
    //     int check = system("pgrep -f mediamtx > /dev/null 2>&1");
    //     if (check != 0) {
    //         ROS_INFO("mediaMTX not running, starting...");
    //         int ret = system("/home/nvidia/mediamtx/mediamtx /home/nvidia/mediamtx/mediamtx.yml &");
    //         if (ret == 0) {
    //             ROS_INFO("mediaMTX started successfully");
    //             // 等待 mediaMTX 就绪
    //             sleep(2);
    //         } else {
    //             ROS_WARN("Failed to start mediaMTX, continuing anyway...");
    //         }
    //     } else {
    //         ROS_INFO("mediaMTX already running");
    //     }
    // }

    // ========== 2. 读取launch文件中的参数 ==========
    // 格式：pnh.param<类型>("参数名", 变量, 默认值)
    // 参数名对应launch文件中的：<param name="xxx" value="yyy"/>

    // --- 相机配置 ---
    std::string rtsp_url;     // 360摄像机的RTSP地址
    int width, height, fps;   // 视频分辨率和帧率

    pnh.param<std::string>("camera/rtsp_url", rtsp_url, "rtsp://192.168.1.65:554/11");
    pnh.param<int>("camera/width", width, 640);      // 默认640像素宽
    pnh.param<int>("camera/height", height, 480);    // 默认480像素高
    pnh.param<int>("camera/fps", fps, 30);           // 默认30帧/秒

    // --- RTSP服务器配置 ---
    int rtsp_port;                    // RTSP服务器端口
    std::string raw_path;             // 原始流路径（VLC播放地址的后缀）
    std::string annotated_path;       // 标注流路径
    int bitrate;                      // 推流码率（kbps）
    bool enable_raw;
    int skip_frames; 
    int push_width, push_height;

    pnh.param<int>("rtsp_server/port", rtsp_port, 8554);  // 默认8554（mediamtx默认端口）
    pnh.param<std::string>("rtsp_server/raw_path", raw_path, "raw");
    pnh.param<std::string>("rtsp_server/annotated_path", annotated_path, "annotated");
    pnh.param<int>("rtsp_server/bitrate", bitrate, 1000);  // 默认1000kbps
    pnh.param<bool>("rtsp_server/enable_raw", enable_raw, false);

    pnh.param<int>("rtsp_server/push_width", push_width, 320);
    pnh.param<int>("rtsp_server/push_height", push_height, 240);
    int push_fps;
    pnh.param<int>("rtsp_server/push_fps", push_fps, 30);

    // --- Python脚本配置 ---
    std::string script_dir;     // Python脚本所在目录
    std::string model_path;     // YOLO模型文件路径（.pt格式）
    float conf_thresh;          // 置信度阈值

    pnh.param<std::string>("python/script_dir", script_dir, "");
    pnh.param<std::string>("python/model_path", model_path, "");
    pnh.param<float>("python/conf_thresh", conf_thresh, 0.3);  // 默认0.3（30%置信度）
               // 跳帧数：每N帧推理1次（CPU/GPU吃力时调大）
    pnh.param<int>("python/skip_frames", skip_frames, 1);      // 默认1=每帧都推理

    // --- 输出配置 ---
    bool publish_topics;        // 是否发布ROS话题
    pnh.param<bool>("output/publish_ros_topics", publish_topics, true);

    // 打印配置信息（便于调试）
    ROS_INFO("Camera RTSP URL: %s", rtsp_url.c_str());
    ROS_INFO("Resolution: %dx%d @ %d fps", width, height, fps);
    ROS_INFO("RTSP Server: port=%d, raw=/%s, annotated=/%s",
             rtsp_port, raw_path.c_str(), annotated_path.c_str());

    // ========== 3. 创建ROS话题发布器 ==========
    // advertise<消息类型>("话题名", 队列长度)
    // 队列长度1：只保留最新一帧，网络慢时丢旧帧也不积压

    // 原始图像话题：其他节点可以订阅这个话题获取原始画面
    ros::Publisher raw_image_pub = nh.advertise<sensor_msgs::Image>("/camera/image", 1);

    // 检测结果话题：其他节点可以订阅这个话题获取YOLO检测结果
    ros::Publisher detection_pub = nh.advertise<yolo_rtsp::DetectionArray>("/yolo/detections", 1);

    // ========== 4. 注册ROS服务(兼容) 与 使能话题订阅 ==========
    // 服务允许其他节点远程控制本节点的启停(兼容旧控制)
    ros::ServiceServer start_srv = nh.advertiseService("/yolo/start", startYoloCallback);
    ros::ServiceServer stop_srv = nh.advertiseService("/yolo/stop", stopYoloCallback);
    // 使能话题：由 controller 的 vision_en 驱动，统一控制识别+推流启停
    ros::Subscriber enable_sub = nh.subscribe("/yolo/enable", 1, enableCallback);
    g_running = false;  // 默认关闭，等待 /yolo/enable 开启

    // ========== 5. 启动YOLO推理进程（Python子进程）==========
    // PythonBridge负责管理Python子进程的生命周期
    std::unique_ptr<PythonBridge> python_bridge;
    if (!script_dir.empty() && !model_path.empty())
    {
        python_bridge = std::make_unique<PythonBridge>();

        // 通过环境变量把配置传给Python脚本
        // Python脚本会读取这些环境变量来确定输入输出地址
        setenv("RTSP_URL", rtsp_url.c_str(), 1);  // 相机地址
        std::string annotated_url = "rtsp://127.0.0.1:" + std::to_string(rtsp_port) + "/" + annotated_path;
        setenv("OUTPUT_RTSP_URL", annotated_url.c_str(), 1);  // 标注流输出地址
        setenv("YOLO_SKIP_FRAMES", std::to_string(skip_frames).c_str(), 1);  // 跳帧数
        setenv("PUSH_BITRATE", std::to_string(bitrate).c_str(), 1);     

        setenv("PUSH_WIDTH", std::to_string(push_width).c_str(), 1);     
        setenv("PUSH_HEIGHT", std::to_string(push_height).c_str(), 1);     
        setenv("PUSH_FPS", std::to_string(push_fps).c_str(), 1);

    }
    else
    {
        ROS_WARN("python/script_dir or python/model_path is empty, YOLO detection disabled");
    }

    // ========== 6. 初始化raw流推流器 ==========
    // raw流 = 原始画面（没有画检测框）
    // 推到本机的mediamtx服务器，上位机通过VLC播放

    // 开机自检：mediaMTX没启动的话，两路推流都会失败
    // 提前报警，避免"跑起来了但流是黑的"这种难排查的问题
    if (!probeTcpPort("127.0.0.1", rtsp_port))
    {
        ROS_ERROR("==================================================");
        ROS_ERROR("WARNING: nothing listening on port %d (mediaMTX not started?)", rtsp_port);
        ROS_ERROR("Both RTSP streams (raw & annotated) will FAIL to publish!");
        ROS_ERROR("Start mediaMTX first, then relaunch this node:");
        ROS_ERROR("  cd ~/mediamtx && ./mediamtx &");
        ROS_ERROR("==================================================");
    }
    else
    {
        ROS_INFO("mediaMTX server OK (port %d)", rtsp_port);
    }

    std::unique_ptr<RtspPusher> raw_pusher;
    if (enable_raw){
        raw_pusher = std::make_unique<RtspPusher>();
        {
        // RTSP地址格式：rtsp://127.0.0.1:8554/raw
        // 127.0.0.1 = 本机（mediamtx运行在同一台机器上）
        // 8554 = mediamtx默认RTSP端口
        // raw = 流路径（VLC播放时输入rtsp://工控机IP:8554/raw）
        std::string raw_url = "rtsp://127.0.0.1:" + std::to_string(rtsp_port) + "/" + raw_path;

        // 初始化推流器：指定分辨率、帧率、码率
        if (!raw_pusher->init(raw_url, width, height, fps, bitrate))
            {
            ROS_ERROR("Failed to init raw RTSP pusher");
            raw_pusher.reset();  // 初始化失败则禁用
            }
        }
    }
 

    // ========== 7. 打开360摄像机 ==========
    // cv::VideoCapture可以通过RTSP URL直接读取网络摄像机
    //
    // 为什么带重试：相机源可能比本节点晚就绪——例如开机自启动时相机还没起来，
    // 或测试时先起了roslaunch、假相机ffmpeg还没推上流。
    // 一次失败就退出太脆弱，这里最多重试10次、每次间隔3秒（共约30秒）。
    cv::VideoCapture cap;
    const int MAX_CAM_RETRIES = 10;   // 相机最大重试次数
    for (int attempt = 1; attempt <= MAX_CAM_RETRIES; attempt++)
    {
        cap.open(rtsp_url);
        if (cap.isOpened())
        {
            break;                    // 打开成功，跳出重试循环
        }
        ROS_WARN("Camera open failed (attempt %d/%d): %s -- retry in 3s",
                 attempt, MAX_CAM_RETRIES, rtsp_url.c_str());
        ros::Duration(3.0).sleep();   // 等3秒再试
        if (!ros::ok())               // 等待期间用户按了Ctrl+C
        {
            return 1;                 // 直接退出，不再重试
        }
    }
    if (!cap.isOpened())
    {
        // 重试耗尽仍打不开：给出两个最可能的排查方向
        ROS_ERROR("Camera failed to open after %d retries: %s", MAX_CAM_RETRIES, rtsp_url.c_str());
        ROS_ERROR("Check: 1) is anything PUBLISHING this stream? 2) mediaMTX log shows 'publishing to path'?");
        return 1;  // 相机最终打不开，退出
    }

    ROS_INFO("Camera opened successfully");

    ros::Rate loop_rate(fps);  // 控制帧率

    // 启动Python推理进程
    if (python_bridge){
        if (!python_bridge->start(script_dir, model_path, conf_thresh))
        {
            ROS_ERROR("Failed to start python bridge, YOLO detection disabled");
            python_bridge.reset();  // 释放对象，后面按"无YOLO"继续跑
        }
    }

    // 心跳日志：每5秒报一次状态，"卡死"和"正常运行"一眼就能区分
    ros::Time last_heartbeat = ros::Time::now();
    int frames_read = 0;
    bool python_dead_reported = false;  // Python崩溃只报一次，避免刷屏
    int cam_fail_count = 0;             // 相机连续读帧失败计数（用于判断断流）

    // ========== 8. 主循环：读帧 → 发话题 → 推流 ==========
    while (ros::ok())
    {
        // 8.0 使能门控：false 时挂起，等待 /yolo/enable 再次开启（可反复启停）
        if (!g_running)
        {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
            continue;
        }

        cv::Mat frame;

        // 8.1 从相机读取一帧
        if (cap.read(frame) && !frame.empty())
        {
            frames_read++;
            cam_fail_count = 0;  // 读到帧了，失败计数清零

            // 8.2 原始画面发布到ROS话题
            // 其他节点（如rviz、rosbag）可以订阅 /camera/image 获取画面
            if (publish_topics)
            {
                try
                {
                    cv_bridge::CvImage cv_image;
                    cv_image.header.stamp = ros::Time::now();
                    cv_image.header.frame_id = "camera";
                    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
                    cv_image.image = frame.clone();
                    raw_image_pub.publish(cv_image.toImageMsg());
                }
                catch (const std::exception& e)
                {
                    // 注意：cv_bridge抛的是cv_bridge::Exception（继承std::runtime_error），
                    // 不是cv::Exception，所以这里必须catch std::exception才接得住
                    ROS_WARN("cv_bridge publish failed: %s (frame: %dx%d)", e.what(), frame.cols, frame.rows);
                }
            }

            // 8.3 原始画面推raw流（上位机可以VLC播放）
            if (raw_pusher)
            {
                raw_pusher->pushFrame(frame);
            }

            // 8.4 发布检测结果（从Python子进程获取）
            if (python_bridge && publish_topics)
            {
                // getLatestDetections()获取Python最近一次推理的结果
                DetectionResult result = python_bridge->getLatestDetections();

                // 打包成ROS消息
                yolo_rtsp::DetectionArray msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "camera";
                msg.frame_count = result.frame_count;    // 帧计数
                msg.image_width = result.image_width;    // 图像宽度
                msg.image_height = result.image_height;  // 图像高度

                // 遍历每个检测结果
                for (const auto& det : result.detections)
                {
                    yolo_rtsp::Detection d;
                    d.class_id = det.class_id;       // 类别ID（0=垃圾, 1=船, 2=人...）
                    d.confidence = det.confidence;   // 置信度（0~1，越高越确定）
                    d.x = det.x;                     // 检测框左上角x坐标
                    d.y = det.y;                     // 检测框左上角y坐标
                    d.width = det.width;             // 检测框宽度
                    d.height = det.height;           // 检测框高度
                    msg.detections.push_back(d);
                }
                detection_pub.publish(msg);
            }
        }
        else
        {
            // 读帧失败处理。
            // 关键机制：OpenCV的RTSP会话一旦被服务器断开，read()会永远返回false，
            // 它自己不会重连——必须手动release()再open()才能恢复。
            // 这在现场很重要：360相机网络抖一下、重启一下，节点必须能自愈，
            // 否则无人值守就是一句空话。
            cam_fail_count++;

            if (cam_fail_count < 30)
            {
                // 短暂失败（偶发丢帧）：等0.1秒再试，不打日志避免刷屏
                ros::Duration(0.1).sleep();
            }
            else
            {
                // 连续失败30次(约3秒) = 会话真死了，进入重连流程
                ROS_WARN("Camera stream dead (30 consecutive failures), reconnecting...");
                cap.release();

                // 重连带重试，最多10次、每次3秒（和第7节初始打开同一套逻辑）
                for (int attempt = 1; attempt <= MAX_CAM_RETRIES; attempt++)
                {
                    cap.open(rtsp_url);
                    if (cap.isOpened())
                    {
                        break;
                    }
                    ROS_WARN("Camera reconnect failed (attempt %d/%d): %s -- retry in 3s",
                             attempt, MAX_CAM_RETRIES, rtsp_url.c_str());
                    ros::Duration(3.0).sleep();
                    if (!ros::ok())
                    {
                        return 1;
                    }
                }

                if (cap.isOpened())
                {
                    cam_fail_count = 0;
                    ROS_INFO("Camera reconnected successfully");

                    // 推流器很可能在断流期间也死了（mediaMTX把空闲的RTSP会话
                    // 超时断开→ffmpeg写入失败退出）。重启它，raw流恢复发布
                    if (raw_pusher)
                    {
                        std::string raw_url = "rtsp://127.0.0.1:" + std::to_string(rtsp_port) + "/" + raw_path;
                        if (raw_pusher->init(raw_url, width, height, fps, bitrate))
                        {
                            ROS_INFO("Raw RTSP pusher restarted");
                        }
                        else
                        {
                            ROS_ERROR("Failed to restart raw RTSP pusher");
                            raw_pusher.reset();
                        }
                    }
                }
                else
                {
                    ROS_ERROR("Camera reconnect failed after %d attempts, exiting", MAX_CAM_RETRIES);
                    return 1;
                }
            }
        }

        // 8.5 处理一次积压的ROS服务请求
        ros::spinOnce();

        // 8.6 Python进程存活检查：崩溃立刻报告（只报一次）
        if (python_bridge && !python_bridge->isAlive() && !python_dead_reported)
        {
            python_dead_reported = true;
            ROS_ERROR("Python inference process is NOT running! annotated stream & detection topic stopped.");
            ROS_ERROR("  1. Scroll terminal for '[Python]' lines (model/dependency errors show there)");
            ROS_ERROR("  2. Check OOM kill: dmesg | grep -i 'killed process'");
            ROS_ERROR("  3. Run manually to see the error: python3 scripts/main.py --model det/best.pt");
        }

        // 8.7 心跳日志：每5秒打印一次状态
        // 正常时每5秒看到一条；卡死时心跳停止——两种情况一眼区分
        if ((ros::Time::now() - last_heartbeat).toSec() >= 5.0)
        {
            ROS_INFO("[heartbeat] frames(5s)=%d | raw_push=%s | python=%s",
                     frames_read,
                     raw_pusher ? "running" : "off",
                     python_bridge ? (python_bridge->isAlive() ? "alive" : "DEAD") : "off");
            frames_read = 0;
            last_heartbeat = ros::Time::now();
        }

        loop_rate.sleep();
    }

    // ========== 9. 清理退出 ==========
    cap.release();  // 释放相机
    // python_bridge和raw_pusher是unique_ptr，离开main时自动析构
    // 析构时会自动停止Python子进程、关闭推流管线
    ROS_INFO("streamer_node exited");
    return 0;
}