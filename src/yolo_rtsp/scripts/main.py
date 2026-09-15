"""
main.py - Python YOLO检测 + RTSP推流 主程序

功能：
  1. 从360摄像机读取RTSP流
  2. 用YOLO进行目标检测
  3. 把标注后的画面推到RTSP服务器（上位机VLC播放）
  4. 把检测结果输出到stdout（C++节点读取）

数据流：
  ┌───────────────────────────────────────────────────────────────┐
  │  360摄像机 → 读帧线程 → YOLO推理 → 标注画面 → ffmpeg推流         │
  │                         ↓                                     │
  │                    检测结果JSON → stdout → C++节点读取          │
  └───────────────────────────────────────────────────────────────┘

输出说明：
  - 标注画面：通过RTSP推流输出，上位机用VLC播放
  - 检测结果：通过stdout输出JSON格式，C++节点解析后发布ROS话题
  - 日志信息：通过stderr输出（stdout只输出JSON，否则C++解析会失败）

无显示器说明：
  本脚本没有任何弹窗代码（不用cv2.imshow），
  画面通过RTSP流输出，在上位机用VLC查看。
"""

import cv2
import threading
import queue
import json
import sys
import argparse

import config
from config import control_vars
from rtsp_reader import read_rtsp_frames
from yolo_inferencer import init_yolo_model, infer_frame
from rtsp_pusher import AnnotatedPusher

# ========== ROS话题发布（可选功能）==========
# 在roslaunch环境下运行时，把标注画面发布到 /camera/image_annotated 话题
# 供rosbag录制或其他节点使用；单独运行（没有roscore）时自动跳过
ros_pub = None
ros_bridge = None
ros_enable_sub = None
def _ros_enable_callback(data):
    if "enabled" in control_vars:
        control_vars["enabled"] = bool(data.data)
        _log(f"[CTRL] /yolo/enable -> {'ON (detect+push)' if control_vars['enabled'] else 'OFF (stop)'}")
try:
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool
    from cv_bridge import CvBridge

    # 初始化ROS节点
    # anonymous=True: 自动添加随机数避免节点名冲突
    # disable_signals=True: 不让rospy抢Ctrl+C信号，退出统一由本脚本管理
    rospy.init_node("yolo_annotated_pub", anonymous=True, disable_signals=True)

    # 创建发布器：发布标注后的图像
    # 话题名：/camera/image_annotated
    # 队列长度1：只保留最新一帧
    ros_pub = rospy.Publisher("/camera/image_annotated", Image, queue_size=1)
    ros_bridge = CvBridge()  # 用于OpenCV和ROS图像消息的转换

    # 使能订阅：由 controller 的 vision_en 并入驱动，统一控制识别+推流启停
    # (原 UDP 9999 独立使能通道已废弃，与 streamer_node 一致)
    ros_enable_sub = rospy.Subscriber("/yolo/enable", Bool, _ros_enable_callback, queue_size=1)
except Exception as e:
    rospy = None
    print(f"[ROS] not connected (standalone mode): {e}", file=sys.stderr)


def _log(msg, level="info"):
    if rospy:
        if level == "warn":
            rospy.logwarn(msg)
        elif level == "error":
            rospy.logerr(msg)
        else:
            rospy.loginfo(msg)
    else:
        print(msg, file=sys.stderr, flush=True)


def main():
    # ========== 命令行参数解析 ==========
    parser = argparse.ArgumentParser(description='YOLO RTSP Streamer')
    parser.add_argument('--model', type=str, help='模型路径(.pt)，覆盖配置文件')
    parser.add_argument('--conf', type=float, default=0.3, help='置信度阈值（0~1）')
    parser.add_argument('--rtsp', type=str, help='相机RTSP地址，覆盖配置文件')
    parser.add_argument('--output', type=str, default=config.OUTPUT_RTSP_URL,
                        help='标注流输出地址')
    args = parser.parse_args()

    # 命令行参数优先于配置文件
    if args.model:
        config.MODEL_PATH = args.model
    if args.rtsp:
        config.RTSP_URL = args.rtsp
    control_vars["control_cmd"]["conf_thresh"] = args.conf

    # ========== 1. 初始化YOLO模型 ==========
    # 模型格式：.pt（PyTorch）或 .engine（TensorRT）
    # 自动判断使用GPU还是CPU
    model = init_yolo_model()
    if not model:
        _log("[ERROR] YOLO model init failed, exiting", "error")
        return

    # ========== 2. 创建帧队列 ==========
    # maxsize=2: 最多缓存2帧，满了丢旧帧，保证处理的是最新画面
    frame_queue = queue.Queue(maxsize=2)

    # ========== 3. 启动RTSP读帧线程 ==========
    # daemon=True: 主程序退出时线程自动结束，不用手动join
    rtsp_thread = threading.Thread(target=read_rtsp_frames, args=(frame_queue,))
    rtsp_thread.daemon = True
    rtsp_thread.start()
    
    # ========== 4. 推流器初始化 ==========
    # 推流器在收到第一帧后才创建（需要先知道画面的实际宽高）
    pusher = None

    frame_count = 0          # 帧计数器
    last_w, last_h = 0, 0   # 记住最近一帧的尺寸（关掉开关时发空结果用）
    was_enabled = False      # 默认关闭，等待 /yolo/enable 开启

    try:
        while not control_vars["stop_flag"]:
            # ========== 5. 从队列取帧 ==========
            try:
                frame = frame_queue.get(timeout=1.0)  # 队列空时最多等1秒
            except queue.Empty:
                continue  # 队列空，跳过本次循环

            # ========== 6. 使能开关检查 ==========
            # 关闭时：不推理、不推流，只把帧丢掉（防止旧帧积压）
            if not control_vars["enabled"]:
                if pusher is not None:
                    pusher.release()   # 停止ffmpeg → annotated流随之停止
                    pusher = None
                if was_enabled:
                    # 开→关的瞬间发一次空检测结果，清掉C++侧缓存的旧数据
                    print(json.dumps({
                        "frame_count": 0,
                        "w": last_w,
                        "h": last_h,
                        "target_num": 0,
                        "detect_results": []
                    }), flush=True)
                    was_enabled = False
                continue
            was_enabled = True

            # ========== 7. 跳帧处理 ==========
            frame_count += 1
            if frame_count % control_vars["control_cmd"]["skip_frames"] != 0:
                continue  # 跳过这一帧（GPU吃力时调大skip_frames）

            # ========== 8. YOLO推理 ==========
            # 返回：
            #   detect_results: 检测结果列表 [{"class":0, "confidence":0.9, "bbox":[x1,y1,x2,y2]}, ...]
            #   annotated_frame: 画好检测框的标注画面
            detect_results, annotated_frame = infer_frame(model, frame, frame_count)
            h, w = annotated_frame.shape[:2]
            last_w, last_h = w, h

            # ========== 8.1 缩放标注帧到配置分辨率（降低编码CPU占用）==========
            target_w, target_h = config.PUSH_WIDTH, config.PUSH_HEIGHT
            if w != target_w or h != target_h:
                annotated_frame = cv2.resize(annotated_frame, (target_w, target_h))
                w, h = target_w, target_h

            # ========== 9. 首帧时启动推流器 ==========
            # 需要知道实际画面尺寸才能启动ffmpeg
            if pusher is None:
                pusher = AnnotatedPusher(args.output)  # 创建推流器
                pusher.start(w, h)                      # 启动ffmpeg进程

            # ========== 10. 推标注流 ==========
            # 把画好检测框的帧喂给ffmpeg，ffmpeg自动编码+RTSP发送
            pusher.write(annotated_frame)

            # ========== 11. 发布到ROS话题（如果ROS可用）==========
            if ros_pub is not None:
                try:
                    # cv2_to_imgmsg: OpenCV图像转ROS图像消息
                    ros_pub.publish(ros_bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))
                except Exception:
                    pass  # 发布失败不影响主流程

            # ========== 12. 输出检测结果（JSON格式到stdout）==========
            # 注意：stdout只能输出JSON，其他日志一律走stderr
            # 否则C++节点解析会失败
            output_json = {
                "frame_count": frame_count,      # 帧计数
                "w": last_w,                           # 图像宽度
                "h": last_h,                           # 图像高度
                "target_num": len(detect_results), # 检测到的目标数量
                "detect_results": detect_results  # 检测结果列表
            }
            print(json.dumps(output_json), flush=True)  # flush=True确保立即输出

    except KeyboardInterrupt:
        # 用户按Ctrl+C退出
        control_vars["stop_flag"] = True
        _log("[INFO] interrupted by user (Ctrl+C)")
    finally:
        # ========== 13. 清理退出 ==========
        control_vars["stop_flag"] = True
        if pusher:
            pusher.release()  # 关闭ffmpeg推流器
        _log("[INFO] program exited normally")


if __name__ == "__main__":
    main()
