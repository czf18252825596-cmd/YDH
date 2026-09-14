"""YOLO推理模块 - 独立管理模型加载和推理"""
import cv2
import torch
from ultralytics import YOLO
import config
from config import control_vars
import sys


def init_yolo_model():
    """加载YOLO模型（.pt格式），自动判断用GPU还是CPU"""
    print("===== 模型初始化 =====", file=sys.stderr)
    print(f"模型路径: {config.MODEL_PATH}", file=sys.stderr)

    model = YOLO(config.MODEL_PATH, task='detect')

    print(f"PyTorch版本: {torch.__version__}", file=sys.stderr)
    print(f"CUDA可用: {torch.cuda.is_available()}", file=sys.stderr)
    if torch.cuda.is_available():
        print(f"设备名称: {torch.cuda.get_device_name(0)}", file=sys.stderr)
        print("模型将在GPU上推理", file=sys.stderr)
    else:
        print("CUDA不可用，使用CPU推理（速度较慢，可加大skip_frames跳帧）", file=sys.stderr)

    return model


def infer_frame(model, frame, frame_count):
    """对单帧进行推理

    :param model: YOLO模型实例
    :param frame: 输入帧(BGR格式)
    :param frame_count: 帧ID（用于结果里的帧计数）
    :return: (检测结果列表, 画好检测框的标注帧)
    """
    results = model.predict(
        source=frame,
        verbose=False,
        imgsz=config.IMGSZ,
        conf=control_vars["control_cmd"]["conf_thresh"],
        max_det=config.MAX_DET,
        device=0 if torch.cuda.is_available() else 'cpu',  # 有N卡用0号GPU，否则CPU
        stream=False,
        augment=False
    )

    detect_results = []
    annotated_frame = frame.copy()

    for result in results:
        if result.boxes is not None:
            # box.data每行是 [x1, y1, x2, y2, 置信度, 类别ID]
            result_lists = result.boxes.data.tolist()
            for box in result_lists:
                x1, y1 = round(box[0]), round(box[1])
                x2, y2 = round(box[2]), round(box[3])
                conf = round(box[4], 2)
                cls = int(box[5])

                detect_results.append({
                    "class": cls,
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2]
                })

            # plot()返回画好检测框和标签的画面
            annotated_frame = result.plot()

    return detect_results, annotated_frame
