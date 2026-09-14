#ifndef YOLO_RTSP_RTSP_PUSHER_H
#define YOLO_RTSP_RTSP_PUSHER_H

#include <string>
#include <deque>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cstdio>

namespace yolo_rtsp
{

/**
 * RTSP推流器（ffmpeg CPU编码）
 *
 * 设计要点（防卡死）：
 *   ffmpeg子进程通过popen启动，帧数据从管道stdin喂入。
 *   管道的fd被设为"非阻塞"（O_NONBLOCK）：
 *     - write()写不进（管道满）时立即返回EAGAIN，绝不卡住调用者
 *     - ffmpeg进程死掉时write()返回EPIPE，立即发现推流断了
 *   写不完的数据存进"整帧队列"（out_queue_），下一帧时继续排空，
 *   队列超过3帧就丢弃最旧的整帧（保证画面永远是较新的，且不花屏）
 */
class RtspPusher
{
public:
    RtspPusher();
    ~RtspPusher();

    /**
     * 初始化推流器：启动ffmpeg子进程
     * @param url      RTSP推流目标地址，如 "rtsp://127.0.0.1:8554/raw"
     * @param width    视频宽度（像素）
     * @param height   视频高度（像素）
     * @param fps      帧率（每秒帧数）
     * @param bitrate  码率（kbps）
     * @return 成功返回true
     */
    bool init(const std::string& url, int width, int height, int fps, int bitrate);

    /**
     * 推送一帧图像（非阻塞，最多耗时几毫秒）
     * @param frame BGR格式的图像帧
     * @return 成功送入ffmpeg返回true；ffmpeg已死返回false
     */
    bool pushFrame(const cv::Mat& frame);

    /** 关闭推流器，结束ffmpeg进程 */
    void close();

private:
    /** ffmpeg进程死亡时统一处理：打日志+清队列，只报一次 */
    void handleProcessDead();

    std::string url_;
    int width_;
    int height_;
    int fps_;
    int bitrate_;

    FILE* ffmpeg_;      // popen返回的管道文件指针
    int ffmpeg_fd_;     // 管道的文件描述符（非阻塞write用）
    bool ffmpeg_dead_;  // ffmpeg已退出的标志（退出后不再尝试写）

    // 整帧发送队列：写不进的数据按"整帧"为单位保存和丢弃
    std::deque<std::vector<char>> out_queue_;
    size_t partial_offset_;  // 队首帧已经成功写出的字节数

    bool initialized_;
};

}  // namespace yolo_rtsp

#endif  // YOLO_RTSP_RTSP_PUSHER_H
