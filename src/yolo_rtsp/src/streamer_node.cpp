/**
 * streamer_node.cpp - ROS1主节点
 *
 * 功能：协调所有模块工作
 *   1. 读取360摄像机的RTSP流
 *   2. 启动Python子进程进行YOLO推理
 *   3. 推RTSP流（标注的）
 *   4. 发布ROS话题供其他节点使用
 *
 * 数据流：
 *   360摄像机 → C++读帧 → Python推理 → 标注画面

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
#include "yolo_rtsp/DetectionArray.h"

using namespace yolo_rtsp;

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

int main(int argc, char** argv)
{
    // 忽略SIGPIPE（ffmpeg管道断开时不会杀死进程）
    signal(SIGPIPE, SIG_IGN);

    // ========== 1. ROS初始化 ==========
    // ros::init会初始化ROS系统，创建节点
    ros::init(argc, argv, "streamer_node");
    ros::NodeHandle nh;       // 全局句柄：用于发布话题、注册服务
    ros::NodeHandle pnh("~"); // 私有句柄：用于读取launch文件中的<param>
                              // "~"代表当前节点的命名空间
    // --- 相机配置 ---
    std::string rtsp_url;     // 360摄像机的RTSP地址
    int width, height, fps;   // 视频分辨率和帧率

    pnh.param<std::string>("camera/rtsp_url", rtsp_url, "rtsp://192.168.1.65:554/11");
    pnh.param<int>("camera/width", width, 640);      // 默认640像素宽
    pnh.param<int>("camera/height", height, 480);    // 默认480像素高
    pnh.param<int>("camera/fps", fps, 30);           // 默认30帧/秒

    // --- RTSP服务器配置 ---
    int rtsp_port;                    // RTSP服务器端口
    std::string annotated_path;       // 标注流路径
    int bitrate;                      // 推流码率（kbps）

    int skip_frames; 
    int push_width, push_height;

    pnh.param<int>("rtsp_server/port", rtsp_port, 8554);  // 默认8554（mediamtx默认端口）
    pnh.param<std::string>("rtsp_server/annotated_path", annotated_path, "annotated");
    pnh.param<int>("rtsp_server/bitrate", bitrate, 1000);  // 默认1000kbps

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

    // 打印配置信息（便于调试）
    ROS_INFO("Camera RTSP URL: %s", rtsp_url.c_str());
    ROS_INFO("Resolution: %dx%d @ %d fps", width, height, fps);
    ROS_INFO("RTSP Server: port=%d, annotated=/%s",
             rtsp_port, annotated_path.c_str());

    // ========== 3.话题发布==========
    ros::Publisher detection_pub = nh.advertise<yolo_rtsp::DetectionArray>("/yolo/detections", 1);

    // 话题订阅
    ros::Subscriber enable_sub = nh.subscribe("/yolo/enable", 1, enableCallback);

    // ========== 4. 启动YOLO推理进程（Python子进程）==========
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

    // 开机自检：mediaMTX没启动的话，两路推流都会失败
    // 提前报警，避免"跑起来了但流是黑的"这种难排查的问题
    if (!probeTcpPort("127.0.0.1", rtsp_port))
    {
        ROS_ERROR("==================================================");
        ROS_ERROR("WARNING: nothing listening on port %d (mediaMTX not started?)", rtsp_port);
        ROS_ERROR("Both RTSP streams (annotated) will FAIL to publish!");
        ROS_ERROR("Start mediaMTX first, then relaunch this node:");
        ROS_ERROR("  cd ~/mediamtx && ./mediamtx &");
        ROS_ERROR("==================================================");
    }
    else
    {
        ROS_INFO("mediaMTX server OK (port %d)", rtsp_port);
    }

    // ========== 5. 打开360摄像机 ==========
    // cv::VideoCapture可以通过RTSP URL直接读取网络摄像机
    // 重试相机源，这最多重试10次、每次间隔3秒（共约30秒）。
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
        // 重试失败，打日志
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
            python_bridge.reset();  // 释放对象
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
        // 6.0 整个程序使能开关：false 时挂起，等待 /yolo/enable 再次开启（可反复启停）
        if (!g_running)
        {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
            continue;
        }

        cv::Mat frame;

        // 6.1 从相机读取一帧
        if (cap.read(frame) && !frame.empty())
        {
            frames_read++;
            cam_fail_count = 0;  // 读到帧了，失败计数清零

            // 6.2 发布检测结果（从Python子进程获取）
            if (python_bridge)
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
            // 读帧失败处理
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
                }
                else
                {
                    ROS_ERROR("Camera reconnect failed after %d attempts, exiting", MAX_CAM_RETRIES);
                    return 1;
                }
            }
        }

        // 6.3 处理一次积压的ROS服务请求
        ros::spinOnce();

        // 6.4 Python进程存活检查：崩溃立刻报告（只报一次）
        if (python_bridge && !python_bridge->isAlive() && !python_dead_reported)
        {
            python_dead_reported = true;
            ROS_ERROR("Python inference process is NOT running! annotated stream & detection topic stopped.");
            ROS_ERROR("  1. Scroll terminal for '[Python]' lines (model/dependency errors show there)");
            ROS_ERROR("  2. Check OOM kill: dmesg | grep -i 'killed process'");
            ROS_ERROR("  3. Run manually to see the error: python3 scripts/main.py --model det/best.pt");
        }

        // 6.5 心跳日志：每5秒打印一次状态
        // 正常时每5秒看到一条；卡死时心跳停止——两种情况一眼区分
        if ((ros::Time::now() - last_heartbeat).toSec() >= 5.0)
        {
            ROS_INFO("[heartbeat] frames(5s)=%d | python=%s",
                     frames_read,
                     python_bridge ? (python_bridge->isAlive() ? "alive" : "DEAD") : "off");
            frames_read = 0;
            last_heartbeat = ros::Time::now();
        }

        loop_rate.sleep();
    }

    // ========== 7. 清理退出 ==========
    cap.release();  // 释放相机
    ROS_INFO("streamer_node exited");
    return 0;
}