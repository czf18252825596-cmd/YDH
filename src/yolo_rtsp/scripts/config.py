"""
config.py - 全局配置模块

功能：集中管理所有可配置参数，避免硬编码

唯一配置源：config/params.yaml
  - roslaunch启动时，C++节点从params.yaml读取参数
  - C++节点通过环境变量传递给Python子进程
  - 本文件的默认值仅用于独立测试（不经过roslaunch时）

优先级：环境变量 > params.yaml（通过C++节点传递）

配置分类：
  1. 模型/硬件配置：YOLO模型路径、推理参数
  2. RTSP输入配置：相机地址、解码方式
  3. 推流配置：输出地址、码率、帧率
  4. UDP配置：兼容Qt端（暂未启用）
  5. 控制配置：使能开关端口
"""

import os

# ===================== 模型/硬件配置 =====================
# 注意：以下默认值仅用于独立测试，正式运行以 params.yaml 为准

# YOLO模型文件路径（.pt格式，训练好的权重文件）
# 独立测试时可通过环境变量 YOLO_MODEL_PATH 覆盖
MODEL_PATH = os.environ.get("YOLO_MODEL_PATH", "yolo11n.pt")

IMGSZ = 640            # 推理输入尺寸（像素）
                        # 模型训练时用640就保持640
                        # 越大精度越高但速度越慢

MAX_DET = 50           # 单帧最多检测目标数
                        # 超过50个目标只返回前50个

DEFAULT_CONF_THRESH = 0.3   # 默认置信度阈值（0~1）
                            # 低于此值的检测框被丢弃
                            # 越大越严格（漏检多但误检少）
                            # 越小越宽松（误检多但漏检少）

# 跳帧数：优先读环境变量 YOLO_SKIP_FRAMES（roslaunch时由C++节点传入
# params.yaml 里 python/skip_frames 的值），单独运行时用默认值1
DEFAULT_SKIP_FRAMES = int(os.environ.get("YOLO_SKIP_FRAMES", "1"))
                            # 1=每帧都推理
                            # 2=每2帧推理1次
                            # 3=每3帧推理1次
                            # GPU吃力时调大


# ===================== RTSP输入配置 =====================
# 注意：以下默认值仅用于独立测试，正式运行以 params.yaml 为准

# 360摄像机的RTSP地址
# 独立测试时可通过环境变量 RTSP_URL 覆盖
RTSP_URL = os.environ.get("RTSP_URL", "rtsp://admin:robosea123@192.168.1.65:554/11")

RTSP_BUFFER_SIZE = 1    # RTSP缓冲区大小（帧数）

GSTREAMER_ENABLE = True  # 是否启用GStreamer解码
                         # True: 使用GStreamer（支持硬件解码，性能更好）
                         # False: 使用OpenCV默认解码（FFmpeg软解）

# 视频解码器选择（按硬件平台选）
# Jetson系列  -> nvv4l2decoder（NVIDIA硬件解码，省CPU）
# 其他ARM/x86 -> avdec_h264（软件解码，通用默认值）
GST_DECODER = os.environ.get("GST_DECODER", "nvv4l2decoder")


# ===================== 推流配置 =====================
# 注意：以下默认值仅用于独立测试，正式运行以 params.yaml 为准

# 标注流推到本机的mediaMTX服务器
# 独立测试时可通过环境变量 OUTPUT_RTSP_URL 覆盖
OUTPUT_RTSP_URL = os.environ.get("OUTPUT_RTSP_URL", "rtsp://127.0.0.1:8554/annotated")

PUSH_FPS = int(os.environ.get("PUSH_FPS", "30"))       # 推流帧率（每秒帧数）
                        # 低于相机帧率可减轻编码压力
                        # 10fps足够监控场景

PUSH_BITRATE = int(os.environ.get("PUSH_BITRATE", "4000"))  # 推流码率（kbps）
                        # 4000 = 4Mbps（1080p足够）
                        # 网络卡顿就调低

PUSH_WIDTH = int(os.environ.get("PUSH_WIDTH", "1920"))  # 推流输出宽度（像素）
PUSH_HEIGHT = int(os.environ.get("PUSH_HEIGHT", "1080")) # 推流输出高度（像素） 

# ===================== UDP配置（兼容Qt端，暂未启用）====================
# 注意：以下默认值仅用于独立测试，正式运行以 params.yaml 为准

UDP_IP = "127.0.0.1"    # UDP发送目标IP
UDP_PORT = 8888         # UDP发送目标端口
JPG_QUALITY = 80        # JPG编码质量（0~100，越大画质越好但数据量越大）
MAX_WIDTH = 640         # 图像最大宽度，超过则等比例缩放


# ===================== 控制链路配置 =====================
# 注意：以下默认值仅用于独立测试，正式运行以 params.yaml 为准

# 使能开关：上位机通过UDP发命令控制"目标识别+推流"总开关
CONTROL_PORT = 9999    # 工控机监听的UDP端口
                       # 上位机往这里发 on/off 命令


# ===================== 控制变量 =====================
# 运行时动态变化的状态，不是配置

control_vars = {
    "stop_flag": False,        # 程序总退出开关
                               # True时主循环退出

    "enabled": True,          # 使能开关（由 /yolo/enable 话题驱动，并入 vision_en）
                               # True: 运行YOLO推理+推流
                               # False: 停止推理和推流（省资源）

    "control_cmd": {
        "skip_frames": DEFAULT_SKIP_FRAMES,   # 当前跳帧数
        "conf_thresh": DEFAULT_CONF_THRESH    # 当前置信度阈值
    }
}
