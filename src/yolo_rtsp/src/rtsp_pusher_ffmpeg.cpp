/**
 * rtsp_pusher_ffmpeg.cpp - RTSP视频推流核心模块（ffmpeg CPU编码版）
 *
 * 功能：把OpenCV的图像帧通过ffmpeg子进程推送到RTSP服务器
 *
 * 推流流程：
 *   原始图像(BGR) → 非阻塞写入管道 → ffmpeg内部H.264编码 → RTSP发送
 *
 * 防卡死设计（重要）：
 *   旧版用fwrite往管道里写帧，fwrite是"阻塞"的：如果ffmpeg因为
 *   某种原因（比如mediaMTX没启动导致它在等连接）不读管道，fwrite
 *   就会永远卡住，整个节点冻结——表现就是"相机打开成功后就没反应"。
 *
 *   现在的做法：
 *   1. 管道fd设为O_NONBLOCK（非阻塞），用write()代替fwrite()
 *   2. write()返回EAGAIN（管道暂时满）→ 剩余数据留在整帧队列，马上返回
 *   3. write()返回EPIPE（ffmpeg死了）→ 打印排查提示，停止推流
 *   4. 队列超过3帧 → 丢掉最旧的"整帧"（绝不能丢半个帧，否则画面花屏）
 */

#include "yolo_rtsp/rtsp_pusher.h"
#include <ros/ros.h>
#include <unistd.h>   // write/fcntl/fileno
#include <fcntl.h>    // fcntl、O_NONBLOCK
#include <cerrno>     // errno、EAGAIN、EPIPE
#include <cstring>

namespace yolo_rtsp
{

RtspPusher::RtspPusher()
    : ffmpeg_(nullptr),
      ffmpeg_fd_(-1),
      ffmpeg_dead_(false),
      partial_offset_(0),
      initialized_(false)
{
}

RtspPusher::~RtspPusher()
{
    close();
}

/**
 * 初始化推流器（启动ffmpeg子进程）
 * ffmpeg参数说明：
 *   -f rawvideo       输入格式：原始未压缩像素数据
 *   -pix_fmt bgr24    像素格式：BGR三通道（OpenCV默认）
 *   -s WxH            画面尺寸
 *   -r fps            帧率
 *   -i -              从stdin读数据（就是我们喂的管道）
 *   -preset ultrafast 最快编码（CPU占用低）
 *   -tune zerolatency 低延迟模式
 *   -f rtsp -rtsp_transport tcp  以RTSP/TCP方式输出
 */
bool RtspPusher::init(const std::string& url, int width, int height, int fps, int bitrate)
{
    if (initialized_)
    {
        close();
    }

    url_ = url;
    width_ = width;
    height_ = height;
    fps_ = fps;
    bitrate_ = bitrate;

    // ffmpeg命令（CPU软编码）
    // 不使用2>/tmp/...重定向，在popen("sh -c ...")中可能导致shell语法错误
    std::string cmd =
        "ffmpeg -y -f rawvideo -pix_fmt bgr24 "
        "-s " + std::to_string(width_) + "x" + std::to_string(height_) + " "
        "-r " + std::to_string(fps_) + " -i - "
        "-c:v libx264 -preset ultrafast -tune zerolatency "
        "-pix_fmt yuv420p "
        "-b:v " + std::to_string(bitrate_) + "k "
        "-maxrate " + std::to_string(bitrate_) + "k "
        "-bufsize " + std::to_string(bitrate_ * 2) + "k "
        "-g " + std::to_string(fps_) + " "
        "-threads 4 "
        "-f rtsp -rtsp_transport tcp "
        "\"" + url_ + "\"";


//         // 当前使用 FFmpeg（软件编码）❌
// std::string cmd =
//     "ffmpeg -y -f rawvideo -pix_fmt bgr24 "
//     "-s " + std::to_string(width_) + "x" + std::to_string(height_) + " "
//     "-r " + std::to_string(fps_) + " -i - "
//     "-c:v libx264 -preset ultrafast -tune zerolatency "
//     "-b:v " + std::to_string(bitrate_) + "k "
//     "-maxrate " + std::to_string(bitrate_) + "k "
//     "-bufsize " + std::to_string(bitrate_ * 2) + "k "
//     "-g " + std::to_string(fps_) + " "
//     "-threads 4 "
//     "-f rtsp -rtsp_transport tcp "
//     "\"" + url_ + "\" 2>/tmp/yolo_rtsp_ffmpeg.log";

    // popen：启动"sh -c 命令"，返回管道的FILE*（我们拿写端，ffmpeg拿读端）
    ffmpeg_ = popen(cmd.c_str(), "w");
    if (!ffmpeg_)
    {
        ROS_ERROR("Failed to start ffmpeg, cmd: %s", cmd.c_str());
        return false;
    }

    ffmpeg_fd_ = fileno(ffmpeg_);

    // 关键：把管道设为非阻塞。
    // 之后用write()写数据：管道满时立即返回EAGAIN而不是卡住等待
    int flags = fcntl(ffmpeg_fd_, F_GETFL, 0);
    if (flags >= 0)
    {
        fcntl(ffmpeg_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    ffmpeg_dead_ = false;
    partial_offset_ = 0;
    out_queue_.clear();

    initialized_ = true;
    ROS_INFO("RTSP pusher initialized (ffmpeg CPU, non-blocking): %s", url_.c_str());
    return true;
}

/**
 * 推送一帧图像（非阻塞）
 *
 * 工作流程：
 *   1. 尺寸不符时先缩放到初始化时指定的尺寸
 *   2. 整帧数据追加到发送队列（保证"整帧"为最小单位，永不写半帧）
 *   3. 尽力排空队列：能写多少写多少
 *      - 写满（EAGAIN）→ 剩下的留在队列，下一帧继续，立刻返回
 *      - ffmpeg死了（EPIPE）→ handleProcessDead()报错一次，之后不再尝试
 *   4. 队列超过3帧 → 丢最旧的整帧（推流落后太多时保实时性）
 */
bool RtspPusher::pushFrame(const cv::Mat& frame)
{
    if (!initialized_ || !ffmpeg_ || ffmpeg_dead_ || frame.empty())
    {
        return false;
    }

    // 尺寸不符时缩放（相机的原始分辨率可能不是配置的640x480）
    cv::Mat resized;
    const cv::Mat* src = &frame;
    if (frame.cols != width_ || frame.rows != height_)
    {
        try
        {
            cv::resize(frame, resized, cv::Size(width_, height_));
            src = &resized;
        }
        catch (const cv::Exception& e)
        {
            ROS_WARN("resize failed: %s", e.what());
            return false;
        }
    }

    // 整帧拷贝进发送队列
    const char* data = reinterpret_cast<const char*>(src->data);
    size_t sz = src->total() * src->elemSize();
    out_queue_.push_back(std::vector<char>(data, data + sz));

    // 防积压：最多缓存3帧，超了丢最旧的整帧
    while (out_queue_.size() > 3)
    {
        out_queue_.pop_front();
        partial_offset_ = 0;
        ROS_WARN_THROTTLE(5, "raw stream write slower than fps, dropping oldest frame");
    }

    // 尽力排空队列（非阻塞）
    while (!out_queue_.empty())
    {
        std::vector<char>& front = out_queue_.front();
        size_t remain = front.size() - partial_offset_;

        // write()：往管道写字节。非阻塞模式下管道满立即返回-1且errno=EAGAIN
        ssize_t n = ::write(ffmpeg_fd_, front.data() + partial_offset_, remain);

        if (n > 0)
        {
            partial_offset_ += static_cast<size_t>(n);
            if (partial_offset_ >= front.size())
            {
                // 这一帧写完了，弹出队列
                out_queue_.pop_front();
                partial_offset_ = 0;
            }
            continue;  // 接着写下一块
        }

        if (n < 0 && errno == EINTR)
        {
            continue;  // 被信号打断，重试一次
        }

        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            // 管道暂时满：剩余数据留在队列，下帧继续。绝不阻塞等待
            return true;
        }

        // 走到这里：EPIPE等错误 = ffmpeg进程已经退出
        handleProcessDead();
        return false;
    }
    return true;
}

/**
 * ffmpeg进程死亡的处理：只报一次错，给出排查方向
 * （最常见的死因：mediaMTX没启动，ffmpeg连不上8554端口直接退出）
 */
void RtspPusher::handleProcessDead()
{
    if (ffmpeg_dead_)
    {
        return;
    }
    ffmpeg_dead_ = true;
    out_queue_.clear();
    partial_offset_ = 0;

    ROS_ERROR("==============================================");
    ROS_ERROR("raw-stream ffmpeg exited, raw video publish stopped!");
    ROS_ERROR("Most common cause: mediaMTX not started (port 8554)");
    ROS_ERROR("Check: 1) mediaMTX running?  2) ps aux | grep ffmpeg");
    ROS_ERROR("==============================================");
}

/** 关闭推流器：清空队列并结束ffmpeg进程 */
void RtspPusher::close()
{
    out_queue_.clear();
    partial_offset_ = 0;

    if (ffmpeg_)
    {
        pclose(ffmpeg_);  // 关闭管道并等待ffmpeg退出
        ffmpeg_ = nullptr;
    }

    ffmpeg_fd_ = -1;
    ffmpeg_dead_ = false;
    initialized_ = false;
}

}  // namespace yolo_rtsp
