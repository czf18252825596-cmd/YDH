#!/usr/bin/env python3
"""
rtsp_pusher.py - Python版RTSP推流器（使用ffmpeg）

功能：把YOLO标注后的画面帧通过ffmpeg子进程推到RTSP服务器

为什么用ffmpeg而不用GStreamer或OpenCV：
┌────────────────────────────────────────────────────────────────┐
│ 方案                优点                    缺点               │
├────────────────────────────────────────────────────────────────┤
│ GStreamer(C++)       性能最好                需要C++调用        │
│ cv2.VideoWriter      简单                    大多数pip装的      │
│                                            opencv不支持RTSP    │
│ ffmpeg子进程         通用、稳定              需要系统安装ffmpeg  │
└────────────────────────────────────────────────────────────────┘

推流流程：
  标注帧(cv::Mat) → stdin喂入 → ffmpeg内部H.264编码 → RTSP发送

ffmpeg命令参数详解：
  -f rawvideo       输入格式：原始视频（无压缩的像素数据）
  -pix_fmt bgr24    像素格式：BGR三通道（OpenCV默认）
  -s WxH            视频尺寸：宽x高
  -r FPS            帧率：每秒帧数
  -i -              数据来源：从stdin（标准输入）读取
  -c:v libx264      编码器：H.264（libx264是开源实现）
  -preset ultrafast 编码速度：最快（牺牲压缩率换速度）
  -tune zerolatency 优化目标：最低延迟
  -b:v Nk           目标码率：N kbps
  -g 30             关键帧间隔：30帧（1秒）
  -f rtsp           输出格式：RTSP
  -rtsp_transport tcp 传输协议：TCP（比UDP稳定）
"""

import subprocess
import sys
import cv2
import config


class AnnotatedPusher:
    """把YOLO标注后的画面帧通过ffmpeg持续推到RTSP服务器（mediaMTX）"""

    def __init__(self, url):
        """
        初始化推流器
        @param url: RTSP推流目标地址，如 "rtsp://127.0.0.1:8554/annotated"
        """
        self.url = url          # 推流目标地址
        self.proc = None        # ffmpeg子进程对象
        self.width = 0          # 视频宽度（启动时从第一帧获取）
        self.height = 0         # 视频高度
        self.died_notified = False  # 避免重复提示ffmpeg退出

    def start(self, width, height):
        """
        启动ffmpeg推流进程
        @param width: 视频宽度（像素）
        @param height: 视频高度（像素）

        注意：width/height必须是实际帧尺寸，之后写入的帧都会缩放到该尺寸
        """
        self.width = width
        self.height = height

        # 构建ffmpeg命令
        cmd = [
            "ffmpeg", "-y",              # -y: 覆盖输出文件（不提示）

            # ========== 输入配置 ==========
            "-f", "rawvideo",            # 输入格式：原始视频
            "-pix_fmt", "bgr24",         # 像素格式：BGR（OpenCV默认，3通道）
            "-s", f"{width}x{height}",   # 视频尺寸，如"640x480"
            "-r", str(config.PUSH_FPS),  # 帧率，如"15"
            "-i", "-",                   # 数据来源：stdin（标准输入）

            # ========== 编码配置 ==========
            "-c:v", "libx264",           # 视频编码器：H.264（libx264开源实现）
            "-preset", "ultrafast",      # 编码速度预设：
                                         #   ultrafast > superfast > veryfast > faster > fast > medium
                                         #   速度越快，CPU占用越低，但压缩率越差
            "-tune", "zerolatency",      # 优化模式：低延迟
                                         #   适合实时推流场景，牺牲一点压缩率换取更小延迟
            "-b:v", f"{config.PUSH_BITRATE}k",  # 目标码率，如"4000k"=4Mbps
                                                 # 码率越大，画质越好，但带宽占用越大
            "-g", "30",                  # 关键帧间隔（GOP大小）：30帧
                                         # 设为帧率值表示每秒一个关键帧
                                         # 关键帧越大，压缩率越高，但seek越慢

            # ========== 输出配置 ==========
            "-f", "rtsp",                # 输出格式：RTSP
            "-rtsp_transport", "tcp",    # RTSP传输协议：TCP（比UDP稳定）
            self.url,                    # 目标地址
        ]

        try:
            # 启动ffmpeg子进程
            # stdin=PIPE: 我们往里写帧数据
            # stdout/stderr丢弃: 防止缓冲区塞满导致卡死
            self.proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,     # 标准输入：用于喂帧数据
                stdout=subprocess.DEVNULL, # 标准输出：丢弃
                stderr=subprocess.DEVNULL, # 标准错误：丢弃
            )
        except FileNotFoundError:
            print("[推流] 找不到ffmpeg命令，请先安装: sudo apt install ffmpeg", file=sys.stderr)
            self.proc = None
            return False

        print(f"[推流] 标注流已启动 -> {self.url}", file=sys.stderr)
        return True

    def write(self, frame):
        """
        写入一帧图像
        @param frame: OpenCV图像帧（BGR格式）
        @return: 成功返回True，失败返回False

        工作流程：
          1. 检查ffmpeg进程是否还活着
          2. 缩放图像到目标尺寸（如果需要）
          3. 把图像二进制数据写入ffmpeg的stdin
          4. ffmpeg内部自动进行H.264编码+RTSP发送
        """
        if self.proc is None:
            return False

        # 检查ffmpeg进程是否还活着
        # poll()返回None表示进程还在运行，否则返回退出码
        if self.proc.poll() is not None:
            if not self.died_notified:
                print("[推流] ffmpeg进程已退出（最常见原因：mediaMTX服务器没启动），本进程停止推流", file=sys.stderr)
                self.died_notified = True
            return False

        # 缩放图像到目标尺寸（如果需要）
        h, w = frame.shape[:2]
        if (w, h) != (self.width, self.height):
            frame = cv2.resize(frame, (self.width, self.height))

        try:
            # 把图像二进制数据写入ffmpeg的stdin
            # frame.tobytes() 把OpenCV图像转成字节数组
            # ffmpeg会自动读取这些数据，进行H.264编码，然后RTSP发送
            self.proc.stdin.write(frame.tobytes())
        except BrokenPipeError:
            # BrokenPipeError表示ffmpeg进程已关闭，无法写入
            return False
        return True

    def release(self):
        """
        关闭推流器，释放资源

        关闭流程：
          1. 关闭stdin → ffmpeg知道没有更多数据了
          2. 等待ffmpeg正常退出（最多3秒）
          3. 如果超时，强制杀死ffmpeg进程
        """
        if self.proc is not None:
            try:
                self.proc.stdin.close()      # 关闭stdin，让ffmpeg正常收尾
                self.proc.wait(timeout=3)    # 等待最多3秒
            except Exception:
                self.proc.kill()             # 超时则强杀
            self.proc = None
            print("[推流] 推流器已关闭", file=sys.stderr)