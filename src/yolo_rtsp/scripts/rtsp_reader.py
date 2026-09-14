"""
rtsp_reader.py - RTSP视频流读取模块

功能：
  1. 从360摄像机读取RTSP视频流
  2. 把帧放入队列供YOLO推理使用
  3. 断线自动重连

数据流：
  360摄像机 ──RTSP──→ GStreamer/OpenCV解码 ──帧──→ 队列 ──→ YOLO推理

队列策略：
  队列满了就丢最旧的帧，保证下游永远拿到最新画面
  监控场景不追求完整性，只追求实时性

GStreamer解码管线：
  rtspsrc → rtph264depay → h264parse → 解码器 → videoconvert → appsink
  各环节作用：
    rtspsrc:        从相机拉RTSP流
    rtph264depay:   从RTP包里取出H.264数据
    h264parse:      整理H.264帧格式
    解码器:         H.264解码成原始图像
    videoconvert:   颜色空间转换（转成OpenCV需要的BGR）
    appsink:        输出给应用程序（OpenCV）
"""

import cv2
import queue
import time
import sys
import config


def open_capture():
    """
    打开RTSP视频流

    优先用GStreamer管线（可用硬件解码），失败则回退到OpenCV默认方式（FFmpeg软解）

    返回：
      成功：cv2.VideoCapture对象
      失败：None或未打开的对象
    """
    if config.GSTREAMER_ENABLE:
        # 构建GStreamer管线字符串
        # 每个环节用 ! 连接，表示数据从左到右流动
        gst_str = (
            # 从相机拉RTSP流
            # latency=0: 最低延迟（不缓冲）
            f"rtspsrc location={config.RTSP_URL} latency=0 ! "

            # 从RTP包里取出H.264数据
            "rtph264depay ! "

            # 整理H.264帧格式（确保解码器能正确解析）
            "h264parse ! "

            # H.264解码器（根据硬件选择）
            # avdec_h264: 软件解码（通用，但CPU占用高）
            # nvv4l2h264dec: NVIDIA硬件解码（Jetson专用，省CPU）
            f"{config.GST_DECODER} ! "

            # 颜色空间转换：YUV → BGR（OpenCV需要BGR格式）
            "videoconvert ! video/x-raw,format=BGR ! "

            # 输出给应用程序
            # sync=false: 不同步时钟，尽快输出（降低延迟）
            "videoconvert ! appsink sync=false"
        )

        # 使用GStreamer管线打开视频流
        cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            print(f"[RTSP] GStreamer方式连接成功: {config.RTSP_URL} (解码器: {config.GST_DECODER})",
                  file=sys.stderr)
            return cap

        # GStreamer失败，回退到OpenCV默认方式
        print("[RTSP] GStreamer方式打开失败，回退到OpenCV默认方式", file=sys.stderr)

    # 使用OpenCV默认方式打开（内部用FFmpeg软解）
    cap = cv2.VideoCapture(config.RTSP_URL)
    if cap.isOpened():
        print(f"[RTSP] OpenCV方式连接成功: {config.RTSP_URL}", file=sys.stderr)
    return cap


def read_rtsp_frames(frame_queue):
    """
    读帧线程的主体

    功能：循环从相机读帧放入队列，断线自动重连，stop_flag置位后退出

    参数：
      frame_queue: queue.Queue对象，用于存放读取到的帧

    工作流程：
      1. 打开相机
      2. 循环读帧
      3. 读到帧就放入队列
      4. 队列满则丢弃旧帧再放新帧
      5. 读帧失败则重连
      6. stop_flag置位后退出
    """
    cap = open_capture()
    if cap is None or not cap.isOpened():
        print(f"[RTSP] 无法打开视频流: {config.RTSP_URL}，请检查相机IP、网络和RTSP地址",
              file=sys.stderr)
        return

    while not config.control_vars["stop_flag"]:
        # 读取一帧
        ret, frame = cap.read()

        if not ret:
            # 读帧失败（可能是相机断线）
            print("[RTSP] 读取帧失败，尝试重连...", file=sys.stderr)
            time.sleep(1)  # 等1秒再重连
            cap.release()

            # 重新打开相机
            cap = open_capture()
            if cap is None or not cap.isOpened():
                print("[RTSP] 重连失败，3秒后重试", file=sys.stderr)
                time.sleep(3)  # 重连失败，等3秒再试
            continue

        # 把帧放入队列
        # 队列满则先丢弃旧帧再放新帧，保证队列里始终是最新画面
        try:
            frame_queue.put_nowait(frame)  # 非阻塞放入
        except queue.Full:
            # 队列满了，丢弃最旧的帧
            try:
                frame_queue.get_nowait()  # 丢弃旧帧
                frame_queue.put_nowait(frame)  # 放入新帧
            except queue.Empty:
                pass  # 理论上不会发生

    cap.release()
    print("[RTSP] 读帧线程退出", file=sys.stderr)
