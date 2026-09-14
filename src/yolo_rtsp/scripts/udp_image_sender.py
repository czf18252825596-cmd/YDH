"""UDP图像发送模块 - 兼容Qt端"""
import cv2
import socket
import struct
import sys
from config import UDP_IP, UDP_PORT, JPG_QUALITY, MAX_WIDTH

# UDP socket
udp_socket = None

def init_udp_sender():
    """初始化UDP发送器"""
    global udp_socket
    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"[UDP] 发送器初始化成功: {UDP_IP}:{UDP_PORT}", file=sys.stderr)
        return True
    except Exception as e:
        print(f"[UDP] 初始化失败: {e}", file=sys.stderr)
        return False

def send_image_by_udp(image, frame_count=0):
    """
    通过UDP发送图像
    :param image: OpenCV图像 (BGR)
    :param frame_count: 帧计数
    """
    global udp_socket

    if udp_socket is None:
        return

    try:
        # 缩放图像
        h, w = image.shape[:2]
        if w > MAX_WIDTH:
            scale = MAX_WIDTH / w
            image = cv2.resize(image, (MAX_WIDTH, int(h * scale)))

        # 编码为JPG
        _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, JPG_QUALITY])

        # 发送
        udp_socket.sendto(buffer.tobytes(), (UDP_IP, UDP_PORT))

    except Exception as e:
        print(f"[UDP] 发送失败: {e}", file=sys.stderr)

def close_udp_sender():
    """关闭UDP发送器"""
    global udp_socket
    if udp_socket:
        udp_socket.close()
        udp_socket = None
        print("[UDP] 发送器已关闭", file=sys.stderr)
