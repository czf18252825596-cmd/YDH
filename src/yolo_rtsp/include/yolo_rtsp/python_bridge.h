#ifndef YOLO_RTSP_PYTHON_BRIDGE_H
#define YOLO_RTSP_PYTHON_BRIDGE_H

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>

#include <opencv2/opencv.hpp>
#include <yolo_rtsp/Detection.h>

// boost/process 用于启动和管理python子进程
#include <boost/process.hpp>

namespace yolo_rtsp
{

struct DetectionResult
{
    int frame_count;
    int image_width;
    int image_height;
    std::vector<Detection> detections;
};

class PythonBridge
{
public:
    PythonBridge();
    ~PythonBridge();

    bool start(const std::string& script_dir,
               const std::string& model_path,
               float conf_thresh);
    void stop();

    /** Python子进程是否还在运行（主循环心跳日志用，发现崩溃立刻报告） */
    bool isAlive();

    void sendFrame(const cv::Mat& frame);
    DetectionResult getLatestDetections();

private:
    void readStdout();
    void readStderr();
    DetectionResult parseJson(const std::string& json_str);

    std::unique_ptr<boost::process::child> process_;
    // 流对象必须是类成员：如果放在start()的局部变量里，
    // 线程lambda按引用捕获，函数返回后引用失效会导致崩溃
    boost::process::ipstream stdout_stream_;
    boost::process::ipstream stderr_stream_;
    std::thread stdout_thread_;
    std::thread stderr_thread_;

    std::atomic<bool> running_;
    std::mutex detection_mutex_;
    DetectionResult latest_detections_;

    std::string socket_path_;
};

}  // namespace yolo_rtsp

#endif  // YOLO_RTSP_PYTHON_BRIDGE_H
