#include "yolo_rtsp/python_bridge.h"
#include <sstream>
#include <cstdlib>  // getenv
#include <ros/ros.h>
#include <iostream>
#include <boost/process.hpp>

namespace yolo_rtsp
{

PythonBridge::PythonBridge() : running_(false)
{
}

PythonBridge::~PythonBridge()
{
    stop();
}

bool PythonBridge::start(const std::string& script_dir,
                         const std::string& model_path,
                         float conf_thresh)
{
    if (running_)
    {
        return true;
    }

    std::string script_path = script_dir + "/main.py";

    // 选择用哪个python启动：
    // 默认找PATH里的python3；可用环境变量PYTHON_BIN指定别的（比如conda的python）
    // 场景：torch/ultralytics装在conda环境里而系统python3没有时，
    //       roslaunch前 export PYTHON_BIN=/home/nvidia/miniconda3/bin/python3
    std::string python_exe = "python3";
    if (const char* env_py = getenv("PYTHON_BIN"))
    {
        if (env_py[0] != '\0')
        {
            python_exe = env_py;
        }
    }

    // 带路径的（如/usr/bin/python3）直接用；只有名字的（python3）去PATH里找
    std::string exe_path = python_exe;
    if (python_exe.find('/') == std::string::npos)
    {
        exe_path = boost::process::search_path(python_exe).string();
    }

    // 启动python子进程：python3 main.py --model 模型路径 --conf 置信度
    // 检测结果以JSON格式从stdout读回，日志从stderr读回
    // 用try包住：python找不到时boost会抛异常，不抓住会把整个节点带崩
    try
    {
        process_ = std::make_unique<boost::process::child>(
            exe_path,
            boost::process::args = {script_path, "--model", model_path, "--conf", std::to_string(conf_thresh)},
            boost::process::std_out > stdout_stream_,
            boost::process::std_err > stderr_stream_
        );
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Failed to start Python process: %s", e.what());
        ROS_ERROR("Check python3 exists, or set PYTHON_BIN env to the right python");
        return false;
    }

    running_ = true;

    stdout_thread_ = std::thread([this]() {
        std::string line;
        while (running_ && std::getline(stdout_stream_, line))
        {
            if (!line.empty())
            {
                DetectionResult result = parseJson(line);
                std::lock_guard<std::mutex> lock(detection_mutex_);
                latest_detections_ = result;
            }
        }
        // 循环退出 = Python的stdout关闭了 = Python进程退出（或还没输出就死了）
        // running_仍为true说明不是正常关停，是Python自己崩了
        if (running_)
        {
            ROS_ERROR("Python inference process exited! Detection stopped. Check:");
            ROS_ERROR("  1. Scroll terminal for '[Python]' lines (the actual error)");
            ROS_ERROR("  2. If OOM-killed: dmesg | grep -i 'killed process'");
        }
    });

    stderr_thread_ = std::thread([this]() {
        std::string line;
        while (running_ && std::getline(stderr_stream_, line))
        {
            if (!line.empty())
            {
                ROS_INFO("[Python] %s", line.c_str());
            }
        }
    });

    ROS_INFO("Python bridge started");
    return true;
}

void PythonBridge::stop()
{
    running_ = false;

    if (process_ && process_->running())
    {
        process_->terminate();
        process_->wait();
    }

    if (stdout_thread_.joinable())
    {
        stdout_thread_.join();
    }

    if (stderr_thread_.joinable())
    {
        stderr_thread_.join();
    }

    process_.reset();
    ROS_INFO("Python bridge stopped");
}

void PythonBridge::sendFrame(const cv::Mat& frame)
{
    // TODO: Send frame to Python process via socket or shared memory
    // For now, Python reads RTSP directly
}

bool PythonBridge::isAlive()
{
    // running()内部用waitpid(WNOHANG)检查，不会阻塞
    // 进程正常退出或崩溃都返回false
    return process_ && process_->running();
}

DetectionResult PythonBridge::getLatestDetections()
{
    std::lock_guard<std::mutex> lock(detection_mutex_);
    return latest_detections_;
}

DetectionResult PythonBridge::parseJson(const std::string& json_str)
{
    DetectionResult result;

    try
    {
        // Simple JSON parsing without external library
        // Expected format: {"frame_count":N,"w":N,"h":N,"target_num":N,"detect_results":[...]}

        auto findValue = [&json_str](const std::string& key) -> std::string {
            size_t pos = json_str.find("\"" + key + "\"");
            if (pos == std::string::npos) return "";
            pos = json_str.find(":", pos);
            if (pos == std::string::npos) return "";
            pos++;
            while (pos < json_str.size() && json_str[pos] == ' ') pos++;
            size_t end = pos;
            while (end < json_str.size() && json_str[end] != ',' && json_str[end] != '}') end++;
            return json_str.substr(pos, end - pos);
        };

        std::string frame_count_str = findValue("frame_count");
        std::string w_str = findValue("w");
        std::string h_str = findValue("h");
        std::string target_num_str = findValue("target_num");

        if (!frame_count_str.empty()) result.frame_count = std::stoi(frame_count_str);
        if (!w_str.empty()) result.image_width = std::stoi(w_str);
        if (!h_str.empty()) result.image_height = std::stoi(h_str);

        int target_num = 0;
        if (!target_num_str.empty()) target_num = std::stoi(target_num_str);

        size_t array_pos = json_str.find("\"detect_results\"");
        if (array_pos != std::string::npos)
        {
            size_t bracket_pos = json_str.find("[", array_pos);
            if (bracket_pos != std::string::npos)
            {
                // Parse each detection object
                size_t obj_start = json_str.find("{", bracket_pos);
                while (obj_start != std::string::npos && result.detections.size() < target_num)
                {
                    size_t obj_end = json_str.find("}", obj_start);
                    if (obj_end == std::string::npos) break;

                    std::string obj_str = json_str.substr(obj_start, obj_end - obj_start + 1);

                    Detection det;
                    std::string class_str = obj_str.find("\"class\"") != std::string::npos ?
                        findValue("class") : "0";
                    std::string conf_str = obj_str.find("\"confidence\"") != std::string::npos ?
                        findValue("confidence") : "0";

                    det.class_id = std::stoi(class_str.empty() ? "0" : class_str);
                    det.confidence = std::stof(conf_str.empty() ? "0" : conf_str);

                    // Parse bbox array [x1, y1, x2, y2]
                    size_t bbox_pos = obj_str.find("\"bbox\"");
                    if (bbox_pos != std::string::npos)
                    {
                        size_t arr_start = obj_str.find("[", bbox_pos);
                        size_t arr_end = obj_str.find("]", arr_start);
                        if (arr_start != std::string::npos && arr_end != std::string::npos)
                        {
                            std::string bbox_str = obj_str.substr(arr_start + 1, arr_end - arr_start - 1);
                            std::stringstream ss(bbox_str);
                            std::string token;
                            std::vector<int> coords;

                            while (std::getline(ss, token, ','))
                            {
                                coords.push_back(std::stoi(token));
                            }

                            if (coords.size() == 4)
                            {
                                det.x = coords[0];
                                det.y = coords[1];
                                det.width = coords[2] - coords[0];
                                det.height = coords[3] - coords[1];
                            }
                        }
                    }

                    result.detections.push_back(det);
                    obj_start = json_str.find("{", obj_end);
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        ROS_WARN("Failed to parse JSON: %s", e.what());
    }

    return result;
}

}  // namespace yolo_rtsp
