# YOLO RTSP Streamer

ROS1 功能包：YOLO 目标检测 + RTSP 视频推流

- **适配环境**：Ubuntu 20.04 / ROS Noetic / aarch64（ARM）工控机
- **模型格式**：`.pt`（训练好的 PyTorch 权重，GPU/CPU 自动适配）
- **无显示器运行**：支持，代码里没有任何弹窗，画面全部通过 RTSP 流输出

---

## 1. 功能包概述

### 1.1 功能说明

本功能包实现：
1. 从360网络摄像机读取RTSP视频流
2. 使用YOLO进行目标检测（垃圾、船、人等）
3. 推两路RTSP流供上位机查看（原始画面 + 标注画面）
4. 发布ROS话题供其他节点使用
5. 支持上位机远程控制启停（UDP协议）

### 1.2 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        系统架构图                                │
└─────────────────────────────────────────────────────────────────┘

360摄像机 ──RTSP──→ C++主节点 ──读帧──→ Python子进程 ──YOLO──→ 标注画面
  192.168.1.65        │                                        │
                      │                                        ↓
                      │                                   ffmpeg推流
                      │                                        │
                      ↓                                        ↓
               推raw流（原始）                         推annotated流（标注）
               rtsp://127.0.0.1:                       rtsp://127.0.0.1:
               8554/raw                                8554/annotated
                      │                                        │
                      ↓                                        ↓
               上位机VLC播放                            上位机VLC播放
```

### 1.3 文件结构与功能说明

```
yolo_rtsp/
├── CMakeLists.txt              # CMake编译配置（定义编译目标、依赖）
├── package.xml                 # ROS包描述文件（声明依赖）
├── README.md                   # 本说明文件
│
├── msg/                        # 自定义ROS消息定义
│   ├── Detection.msg           # 单个检测结果（类别、置信度、坐标）
│   └── DetectionArray.msg      # 检测结果数组（一帧的所有检测）
│
├── launch/                     # ROS启动文件
│   └── streamer.launch         # 一键启动所有节点和参数
│
├── config/                     # 配置文件
│   └── params.yaml             # 运行参数（相机地址、模型路径、推流配置）
│
├── scripts/                    # Python脚本
│   ├── main.py                 # Python主程序（YOLO推理 + 推流调度）
│   ├── config.py               # 全局配置（所有参数集中管理）
│   ├── rtsp_reader.py          # RTSP视频流读取（从相机读帧）
│   ├── yolo_inferencer.py      # YOLO模型加载和推理
│   ├── rtsp_pusher.py          # Python版RTSP推流器（ffmpeg实现）
│   └── udp_image_sender.py     # UDP图像发送（兼容Qt端，暂未启用）
│
├── include/yolo_rtsp/          # C++头文件
│   ├── streamer_node.h         # ROS主节点类定义（已删除，改为扁平main）
│   ├── python_bridge.h         # Python子进程管理类
│   └── rtsp_pusher.h           # C++版RTSP推流器类（GStreamer实现）
│
└── src/                        # C++源文件
    ├── streamer_node.cpp       # ROS主节点（核心调度中心）
    ├── python_bridge.cpp       # Python子进程管理（启动/停止/通信）
    └── rtsp_pusher.cpp         # C++版RTSP推流器（GStreamer实现）
```

### 1.4 各文件详细功能

| 文件 | 语言 | 功能 | 关键参数 |
|------|------|------|----------|
| **streamer_node.cpp** | C++ | ROS主节点，协调所有模块 | rtsp_url, rtsp_port |
| **rtsp_pusher.cpp** | C++ | C++推流器，推原始画面流 | bitrate, fps, key-int-max |
| **python_bridge.cpp** | C++ | 管理Python子进程 | script_dir, model_path |
| **main.py** | Python | Python主程序，YOLO推理+推流 | conf_thresh, skip_frames |
| **config.py** | Python | 全局配置，参数集中管理 | 所有配置项 |
| **rtsp_reader.py** | Python | 从相机读取RTSP流 | RTSP_URL, GST_DECODER |
| **yolo_inferencer.py** | Python | YOLO模型加载和推理 | MODEL_PATH, IMGSZ |
| **rtsp_pusher.py** | Python | Python推流器，推标注画面流 | PUSH_FPS, PUSH_BITRATE |
| **streamer.launch** | XML | 启动文件，一键启动所有功能 | 所有launch参数 |
| **params.yaml** | YAML | 运行参数配置 | 所有配置项 |

### 1.5 数据流详解

```
┌─────────────────────────────────────────────────────────────────┐
│                        数据流详解                                │
└─────────────────────────────────────────────────────────────────┘

【输入】360摄像机 RTSP流
  │
  ├──→ rtsp_reader.py（GStreamer解码）──→ frame_queue（帧队列）
  │                                         │
  │                                         ↓
  │                                    main.py主循环
  │                                         │
  │                    ┌────────────────────┼────────────────────┐
  │                    ↓                    ↓                    ↓
  │              yolo_inferencer.py   rtsp_pusher.py      stdout输出
  │              （YOLO推理）         （ffmpeg推流）       （JSON检测结果）
  │                    │                    │                    │
  │                    ↓                    ↓                    ↓
  │              检测结果+标注帧     annotated流           C++节点读取
  │                                  (8554/annotated)          │
  │                                                            ↓
  │                                                     /yolo/detections话题
  │
  └──→ streamer_node.cpp（C++主节点）
              │
              ├──→ 推raw流（GStreamer）──→ raw流(8554/raw)
              │
              └──→ 发布ROS话题 ──→ /camera/image（原始图像）
```

### 1.6 关键参数说明

| 参数 | 位置 | 意义 | 建议值 | 说明 |
|------|------|------|--------|------|
| `rtsp_url` | params.yaml | 相机RTSP地址 | - | 360摄像机地址 |
| `rtsp_port` | params.yaml | 推流端口 | 8554 | mediamtx默认端口 |
| `model_path` | params.yaml | YOLO模型路径 | - | .pt格式权重文件 |
| `conf_thresh` | params.yaml | 置信度阈值 | 0.3 | 越大越严格 |
| `bitrate` | rtsp_pusher.cpp | H.264码率 | 4000 | kbps，越大画质越好 |
| `speed-preset` | rtsp_pusher.cpp | 编码速度 | ultrafast | 越快CPU占用越低 |
| `tune` | rtsp_pusher.cpp | 优化模式 | zerolatency | 低延迟 |
| `key-int-max` | rtsp_pusher.cpp | 关键帧间隔 | fps值 | 每秒一个关键帧 |
| `PUSH_FPS` | config.py | 推流帧率 | 15 | 低于相机帧率可省资源 |
| `PUSH_BITRATE` | config.py | 推流码率 | 4000 | kbps |
| `skip_frames` | config.py | 跳帧数 | 1 | GPU吃力时调大 |
| `GST_DECODER` | config.py | 解码器 | avdec_h264 | Jetson用nvv4l2h264dec |

---

## 2. 功能与数据流

```
360摄像机(RTSP) ──┬──> C++节点：读帧 → 推raw原始流(8554/raw) → 发布ROS话题
                  │
                  └──> Python进程：读帧 → YOLO推理 → 标注画面ffmpeg推流(8554/annotated)
                                              └→ 检测结果JSON → C++节点 → /yolo/detections话题

上位机(Windows)用VLC连接：rtsp://工控机IP:8554/raw 或 rtsp://工控机IP:8554/annotated
```

**三个角色分工**（理解这个就理解了整个系统）：

| 组件 | 职责 |
|------|------|
| `streamer_node`（C++） | 读相机、推 raw 流、把检测结果转成 ROS 话题 |
| `main.py`（Python 子进程） | YOLO 推理、标注画面推流，结果以 JSON 从 stdout 传回 C++ |
| **mediaMTX**（流媒体服务器） | 本机 8554 端口的中转站，两路流都推给它，上位机从它这里拉流 |

> ⚠️ mediaMTX 是**必装组件**，不启动它两路流都推不出去（黑屏）。

---

## 2. 明日测试清单（按顺序执行）

### 第 0 步：确认硬件型号（决定解码方案）

```bash
cat /proc/device-tree/model
```

- 输出含 `NVIDIA Jetson` → 用硬件解码（快，省 CPU）：
  ```bash
  # 临时生效（也可写进 ~/.bashrc 永久生效）
  export GST_DECODER=nvv4l2h264dec
  ```
- 输出其他型号（RK3588 等）→ 什么都不用做，默认软解 `avdec_h264`

### 第 1 步：确认相机可达

```bash
ping 192.168.1.65    # 改成相机实际IP
```

### 第 2 步：安装依赖（见第 3 节，首次部署执行一次）

### 第 3 步：安装并启动 mediaMTX（见第 4 节，首次部署执行一次）

### 第 4 步：放置模型

```bash
# 把训练好的 .pt 模型传到工控机（scp 从 Windows 发，或U盘）
# 放到主目录，例如：
cp best.pt ~/yolo11n.pt
```

### 第 5 步：编译（见第 6 节，代码有改动时才需要重新编译）

### 第 6 步：运行（两个终端）

```bash
# 终端1：启动流媒体服务器
cd ~/mediamtx && ./mediamtx

# 终端2：启动检测推流（model_path 换成你的实际路径）
source ~/catkin_ws/devel/setup.bash
roslaunch yolo_rtsp streamer.launch model_path:=/home/你的用户名/yolo11n.pt
```

### 第 7 步：Windows 上用 VLC 验证

打开 VLC → 媒体 → 打开网络串流 → 输入：

```
rtsp://工控机IP:8554/raw         # 原始画面
rtsp://工控机IP:8554/annotated   # 带检测框画面（核心验证项）
```

`annotated` 流里能看到检测框 = 目标识别 + 视频推流两项功能全部达标。

---

## 3. 依赖安装（Ubuntu 20.04 / ROS Noetic）

### 3.1 apt 部分

```bash
sudo apt update
sudo apt install -y \
    ros-noetic-cv-bridge ros-noetic-image-transport \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools \
    ffmpeg \
    python3-pip
```

> `ffmpeg` 是标注流推流的核心依赖，必须装。

### 3.2 pip 部分（Python 3.8）

```bash
pip3 install "numpy<2" ultralytics opencv-python
```

> - `numpy<2`：Noetic 的 cv_bridge 基于 numpy 1.x 编译，装 numpy 2.x 会冲突
> - 若 ultralytics 安装时提示 Python 版本过低，安装稍旧的版本：`pip3 install "ultralytics==8.3.0"`

### 3.3 PyTorch

- **Jetson 系列**：不要用 pip 装！去 NVIDIA 官方论坛下载对应 JetPack 版本的 wheel 手动安装
- **其他 ARM（如 RK3588）/ 无独显**：
  ```bash
  pip3 install torch --index-url https://download.pytorch.org/whl/cpu
  ```
  CPU 推理较慢，把 `params.yaml` 里 `skip_frames` 调到 3~5 缓解

### 3.4 验证依赖

```bash
python3 -c "import torch; print('CUDA:', torch.cuda.is_available())"
python3 -c "import ultralytics; print('ultralytics OK')"
ffmpeg -version | head -1
```

---

## 4. mediaMTX 安装（RTSP 服务器）

1. 浏览器打开 https://github.com/bluenviron/mediamtx/releases
2. 下载 `mediamtx_*_linux_arm64v8.tar.gz`（ARM 机器对应这个，注意不是 amd64）
3. 解压并运行：

```bash
mkdir -p ~/mediamtx && cd ~/mediamtx
# 把下载的压缩包放到这个目录后：
tar -xzf mediamtx_*_linux_arm64v8.tar.gz
./mediamtx
```

看到 `RTSP listener opened on :8554` 即成功。**默认配置就是 8554 端口，无需修改任何配置文件。**

测试完想让它在后台跑：`nohup ./mediamtx > mediamtx.log 2>&1 &`

---

## 5. 模型说明

- 当前默认加载 **`.pt` 格式**（PyTorch 权重），路径优先级：命令行 `model_path:=` > `config/params.yaml`
- 模型路径默认写的 `/home/nvidia/yolo11n.pt`，**记得改成实际路径**（用户名用 `whoami` 查）
- Jetson 用户以后想提速，可把 .pt 导出为 TensorRT 引擎：

```bash
yolo export model=yolo11n.pt format=engine device=0
# 生成 yolo11n.engine 后把 model_path 指向它即可（.engine 只能在生成它的那台机器上用）
```

---

## 6. 编译

```bash
cd ~/catkin_ws/src
# 把本功能包放进 src 目录（如果还没有）
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> 如果之前编译过旧版本，先 `rm -rf build/yolo_rtsp devel/lib/yolo_rtsp devel/include/yolo_rtsp` 清缓存再编译。

---

## 7. 运行参数速查

```bash
# 常规启动
roslaunch yolo_rtsp streamer.launch

# 指定模型路径（最常用）
roslaunch yolo_rtsp streamer.launch model_path:=/home/user/best.pt

# 换相机地址
roslaunch yolo_rtsp streamer.launch rtsp_url:=rtsp://192.168.1.100:554/11

# 只测Python推理（不经过ROS，方便排查）
python3 ~/catkin_ws/src/yolo_rtsp/scripts/main.py --model ~/yolo11n.pt
```

---

## 8. ROS 接口

| 类型 | 名称 | 说明 |
|------|------|------|
| 话题 | `/camera/image` | 原始图像（sensor_msgs/Image） |
| 话题 | `/camera/image_annotated` | 标注图像（由 Python 进程发布） |
| 话题 | `/yolo/detections` | 检测结果（yolo_rtsp/DetectionArray） |
| 服务 | `/yolo/start` `/yolo/stop` | 启停检测 |

调试常用：

```bash
rostopic hz /yolo/detections        # 检测帧率
rostopic echo /yolo/detections      # 看检测结果内容
```

## 8.1 上位机对接方案（上位机待开发，预留接口）

工控机输出的所有数据都是**标准网络服务**，上位机(Qt)开发时按下面方式接入，无需改动工控机侧代码：

**视频画面：拉 RTSP 流（推荐）**

上位机用 OpenCV 连流，拿到的就是带检测框的实时画面，和 VLC 里看到的完全一致：

```cpp
cv::VideoCapture cap("rtsp://工控机IP:8554/annotated");
cv::Mat frame;
while (cap.read(frame)) {
    // frame → QImage → 显示到界面
}
```

- 原始画面（不带框）用 `rtsp://工控机IP:8554/raw`
- 延迟约 0.2~0.5 秒；若上位机不集成 OpenCV，可改用 FFmpeg SDK（libavformat），延迟更低但开发量大

**检测结果数据：两种方式按上位机部署位置选**

- 上位机和工控机在同一 ROS 网络（装了 ROS）：直接订阅 `/yolo/detections` 话题
- 上位机是纯 Windows 程序（无 ROS，常见情况）：后续可在 Python 侧加一路 UDP JSON 发送（项目已预留 `udp_image_sender.py` 可扩展），届时告知上位机 IP 和端口即可启用

**使能开关：控制链路（已实现）**

上位机一个开关同时控制"目标识别+标注推流"的启停（绑定逻辑在工控机内部，上位机只发一个命令）：

- 开 = YOLO推理 + 带框画面推流 + 检测数据输出
- 关 = 全部停止（省算力；raw 原始流保持常开作为基础监控画面）
- 协议：UDP，工控机监听 **9999 端口**，命令收到后回执 `ok`

命令格式（任选其一）：

```
纯文本:  on  /  off        （也接受 1/0、true/false）
JSON:    {"enable": true}  /  {"enable": false}
```

上位机发送示例（Qt C++）：

```cpp
QUdpSocket sock;
sock.writeDatagram(QByteArray("on"), QHostAddress("192.168.1.50"), 9999);
```

Windows 上快速测试（装了 Python 即可，无需写代码）：

```powershell
python -c "import socket; socket.socket(socket.AF_INET,socket.SOCK_DGRAM).sendto(b'off',('工控机IP',9999))"
```

开关状态可通过 VLC 验证：发 `off` 后 annotated 流几秒内停止，发 `on` 后恢复。

---

## 9. 常见问题

**Q: 显示"Camera opened successfully"之后就崩溃/没反应了？**（2026-09-02 现场遇到）

这正是"相机打开成功 → 主循环第一帧 → 推流写入"这段出问题。按下面四步排查（大概率第 1、2 步就能定位）：

1. **看 roslaunch 的死亡报告**：终端里找 `process has died [pid xxx, exit code N]`
   - `exit code -11` = 段错误（把完整输出发给助手）
   - `exit code 137` 或 `-9` = **被系统杀掉，九成是内存不足**（Jetson/小内存工控机跑 torch+YOLO+双 ffmpeg 很容易触发）
2. **查内存杀手**（是否被 OOM 杀掉，一条命令定案）：
   ```bash
   dmesg | grep -i "killed process"
   # 有输出 = 内存不足。对策：dmesg里看哪个进程被杀；free -h 看剩余内存；
   # 先测推流时可以暂时不起 Python（params.yaml 的 script_dir 置空），或加大 skip_frames
   ```
3. **看 Python 侧报错**：终端往上翻 `[Python]` 开头的行——模型路径不对、torch 版本不对都会在这显示。也可以手动单跑：
   ```bash
   python3 ~/catkin_ws/src/yolo_rtsp/scripts/main.py --model ~/catkin_ws/src/yolo_rtsp/det/best.pt
   ```
4. **看 ffmpeg 侧报错**：`cat /tmp/yolo_rtsp_ffmpeg.log`——`Connection refused` = mediaMTX 没启动

> 2026-09-02 已修复一个确定性问题：旧版推流用阻塞式 fwrite 写 ffmpeg 管道，ffmpeg 没及时读数据时**整个节点会永久卡死**（表现就是"相机打开成功后没动静"）。现已改为非阻塞写入+整帧队列，同时节点新增：启动时自检 mediaMTX、Python 崩溃立即报告、每 5 秒心跳日志（心跳还在=节点活着，只是某环节慢；心跳停了=真卡死/崩溃）。

**Q: VLC 连上 8554 端口黑屏 / 连接被拒？**
mediaMTX 没启动，或推流端没起来。排查顺序：
1. `./mediamtx` 的终端是否在运行、有没有报错
2. roslaunch 终端里有没有 `[推流] 标注流已启动` 字样
3. `ffmpeg已退出` 字样出现 = 推流时 mediaMTX 不在线，先起 mediaMTX 再起 roslaunch

**Q: 相机连不上？**
- `ping` 相机 IP 确认网络通
- 360 相机对并发 RTSP 会话数有限制：本方案 C++ 和 Python **各连一路**。若第二路连不上（日志反复"重连失败"），把相机重启一次，或联系相机厂商确认会话上限

**Q: 画面卡顿？**
按顺序调：`params.yaml` 里降 `skip_frames` 前先降推理负担 → `config.py` 里 `PUSH_BITRATE` 降到 2000 → 相机侧降分辨率

**Q: Python 报 `CUDA not available`？**
Jetson 装的是 pip 版 torch（不对）。卸载后按 3.3 节装 NVIDIA 官方 wheel。

**Q: `import cv_bridge` 报错？**
没装 `ros-noetic-cv-bridge`（见 3.1）。

**Q: 单独跑 main.py 时提示未连接 ROS？**
正常现象。单独运行不经过 roscore，只推流不发话题；经 roslaunch 启动时会自动连上。

**Q: 画面有延迟怎么处理？**

按"先测量 → 再定位 → 后调参"三步走：

1. **测量**：手机秒表放在摄像头前，播放画面里的数字与手机实际数字的差值 = 端到端延迟
2. **定位**：分别测 raw 流和 annotated 流
   - raw 本身延迟高 → 问题在传输/编码/播放器
   - raw 低、annotated 高 → 瓶颈是 YOLO 推理（最常见）
3. **调参**（按见效程度排序）：
   - **播放器缓存经常占一半延迟**：VLC 用 `vlc --network-caching=100 rtsp://...` 启动；上位机 OpenCV 拉流前设置环境变量 `OPENCV_FFMPEG_CAPTURE_OPTIONS="rtsp_transport;tcp|fflags;nobuffer|flags;low_delay"` 并 `cap.set(cv::CAP_PROP_BUFFERSIZE, 1)`（需在 import cv2 之前设置）
   - 推理慢：`skip_frames` 调到 2~3（标注帧率降但每帧更新鲜）；`IMGSZ` 降到 320（约3倍速，小目标检出率降）
   - 编码已是 zerolatency+ultrafast 无需动；ffmpeg 的 `-g 30` 改 `-g 15` 可加快首画面
   - 网络卡顿换有线；相机 App 里降分辨率/码率

期望值：调好后端到端 0.3~0.8 秒属正常。**注意：任何依赖画面做实时决策的功能不要走视频流（秒级延迟），走 `/yolo/detections` 数据链路（毫秒级）。**

---

## 10. 部署要求对照表

| 部署要求 | 状态 | 说明 |
|----------|------|------|
| 视频推流 | ✅ 本包实现 | 双路 RTSP（raw + annotated），经 mediaMTX 中转 |
| 目标识别 | ✅ 本包实现 | YOLO .pt 模型，结果走 /yolo/detections 话题 |
| 无显示器启动 | ✅ 已适配 | 代码无弹窗依赖，SSH/自启动环境下直接跑 |
| 静态 IP | ✅ 系统层已配置 | netplan 配置，与本项目无关 |
| 远程 NoMachine | ✅ 系统层已配置 | GNOME 走 X11（`WaylandEnable=false`），避免黑屏 |
| 程序自启动 | ⏳ 最后阶段统一做 | 见下方说明 |

### 开机自启动规划（最后阶段做，不要现在做）

思路：**一个 systemd 服务包住整个 roslaunch**，不是每个程序单独配。将来所有功能（检测、推流、路径规划、状态机）都塞进一个 launch 文件，开机只需拉起这一个服务。

参考模板（到时候按实际路径改）：

```ini
# /etc/systemd/system/yolo-streamer.service
[Unit]
Description=YOLO RTSP Streamer
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=你的用户名
ExecStart=/bin/bash -c "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch yolo_rtsp streamer.launch model_path:=/home/你的用户名/yolo11n.pt"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

mediaMTX 同样做一个服务（`mediamtx.service`，ExecStart 指向 `~/mediamtx/mediamtx`），并让上面这个服务 `After=` 依赖它。

### 自启动的管理命令

```bash
sudo systemctl daemon-reload            # 改过服务文件后必须重新加载
sudo systemctl enable --now mediamtx    # 启用自启 + 立刻启动（两个服务各执行一次）
sudo systemctl enable --now yolo-streamer
systemctl status yolo-streamer          # 看状态（绿色 active (running) = 正常）
sudo journalctl -u yolo-streamer -f     # 实时看日志（排错用）
sudo reboot                             # 终极验证：重启后不登录，直接VLC连流
```

> 注意：改了服务文件要 `daemon-reload`；换模型路径等日常改动只改 launch/params.yaml，不用动服务文件。

---

## 11. 本次代码修复记录（相对原始版本）

| # | 问题 | 后果（修复前） | 修复 |
|---|------|----------------|------|
| 1 | CMakeLists 漏声明 `DetectionArray.msg` | **编译直接失败** | 补上 |
| 2 | `python_bridge.h` 用了 boost::process 却没 include | **编译失败** | 补 include |
| 3 | C++ 节点创建了 Python 桥但从不调用 `start()` | YOLO 永远不运行，检测和标注流全死 | 启动时调用 |
| 4 | `main.py` 里的 `cv2.imshow` | **无显示器环境直接崩溃** | 删除，画面走 RTSP |
| 5 | 推流地址写 `0.0.0.0` | 0.0.0.0 不是合法目标地址 | 改 127.0.0.1 |
| 6 | Python 桥线程按引用捕获局部流对象 | 函数返回后悬垂引用，随机崩溃 | 流对象改为类成员 |
| 7 | 标注流没有数据通路（sendFrame 是空实现） | annotated 流永远黑屏 | Python 侧用 ffmpeg 直接推流 |
| 8 | 解码器写死 Jetson 专属的 `nvv4l2h264dec` | 非 Jetson 机器打不开相机 | 默认软解，环境变量 `GST_DECODER` 可切硬解 |
| 9 | 模型路径默认 `.engine` | 与实际 `.pt` 模型不匹配 | 改为 `.pt`，路径可配置 |
| 10 | GStreamer 打开失败无回退 | pip 版 opencv 直接连不上相机 | 自动回退 OpenCV 默认方式 |
| 11 | 命令行 `--model`/`--rtsp` 参数不生效 | from-import 拿到的是旧值副本 | 改为运行时读 config 模块属性 |
| 12 | C++ 节点是类结构，主流程藏在 `node.run()` 里 | 初学者难从头到尾读懂执行顺序 | 整个节点重写为扁平 `main` 函数，从上往下读即完整流程；`streamer_node.h` 已删除 |
| 13 | raw 流推流用阻塞式 `fwrite` 写 ffmpeg 管道 | **ffmpeg 消费慢时主循环永久卡死**（现场"相机打开成功后崩溃"的头号嫌疑） | 改非阻塞 `write()` + 整帧队列（超过 3 帧丢旧帧，永不花屏） |
| 14 | ffmpeg / Python 进程死掉时节点毫无感知 | 推流断了没提示，只能黑屏猜 | ffmpeg 死亡检测（EPIPE）+ Python 存活检查 + 每 5 秒心跳日志 + 启动时自检 mediaMTX |
| 15 | cv_bridge 的 catch 写成 `cv::Exception` | cv_bridge 实际抛 `std::runtime_error` 派生异常，原 catch 接不住 | 改为 catch `std::exception` |
| 16 | python3 找不到时 boost::process 构造抛异常 | 未捕获异常直接终结整个节点 | start() 内 try/catch，失败返回 false 优雅降级 |
| 17 | Python 解释器写死 PATH 里的 python3 | torch 装在 conda 时找不到依赖 | 支持环境变量 `PYTHON_BIN` 指定 |
| 18 | `rtsp_pusher.h` 残留从未调用的 GStreamer 管线代码（且未 `gst_init`） | 死代码误导读者，误调用有崩溃风险 | 整体删除，推流统一走 ffmpeg 方案 |
| 19 | `params.yaml` 的 `python/skip_frames` 从未传给 Python | 改配置不生效，调参被误导 | C++ 启动 Python 前设环境变量 `YOLO_SKIP_FRAMES`，`config.py` 读取 |
| 20 | 诊断日志里的中文在部分终端显示成 `????`（VM 实测复现） | 关键报警信息不可读 | 所有 ROS 日志改英文（代码注释仍保留中文） |
| 21 | 相机打不开时节点立即退出（exit 1） | 启动顺序竞态：相机源比节点晚就绪时服务直接死（VM 实测：先 roslaunch 后起假相机 ffmpeg） | 打开相机加 10 次×3 秒重试，等待期间可 Ctrl+C |
| 22 | RTSP 断流后节点永远刷 `Failed to read frame`（VM 实测复现：USB 摄像头数据损坏 → mediaMTX 超时踢掉会话） | OpenCV 的 RTSP 会话被断开后 `read()` 永远返回 false，不会自愈；现场相机网络抖动也会触发同样问题，无人值守直接失效 | 连续失败 30 次(约3秒)自动 release+重连(带重试)；重连成功后自动重启 raw 推流器（它的 ffmpeg 大概率也在断流期间被踢死了） |

---

## 12. 在家测试：虚拟机 + 笔记本摄像头（代码零改动）

没有 360 相机也能测全链路：用 **ffmpeg 把笔记本摄像头"伪装成"RTSP 相机**，项目代码一行不用改。
还能顺带验证第 9 节修的崩溃问题（非阻塞推流、心跳日志、mediaMTX 自检）。

```
笔记本摄像头(USB直通进VM) → ffmpeg → mediaMTX:8554/usb(假相机)
                                        ↑拉 /usb          ↓推 /raw /annotated
                                     yolo_rtsp ──→ Windows 用 VLC 拉流验证
```

### 步骤

1. **装虚拟机**：VMware Workstation Player 或 VirtualBox；Ubuntu 20.04 桌面版；
   配置 4核 / 8GB内存 / 60GB磁盘；**网络模式选"桥接"**（Windows 才能直接访问 VM 的 IP）
2. **装 ROS Noetic**（清华镜像）：
   ```bash
   sudo sh -c 'echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
   sudo apt update && sudo apt install ros-noetic-ros-base ros-noetic-cv-bridge -y
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source ~/.bashrc
   ```
3. **传项目 + 装依赖**：整个 yolo_rtsp 拷进 VM（共享文件夹或直接拖）；其余依赖照第 3 节装，
   torch 用 CPU 版（第 3.3 节是 Jetson 用的，VM 里换成下面这个）：
   ```bash
   pip3 install torch==1.13.1 torchvision==0.14.1 --index-url https://download.pytorch.org/whl/cpu
   pip3 install ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
   ```
   mediaMTX 照第 4 节装（VM 是 x86_64，下载 `mediamtx_linux_amd64` 包）；编译照第 6 节
4. **摄像头直通**：VMware 菜单 虚拟机 → 可移动设备 → 你的摄像头 → 连接。
   验证：`ls /dev/video0` 有输出即成功（直通期间 Windows 自己用不了这个摄像头，断开即恢复）
5. **造"假相机"**（两个终端）：
   ```bash
   # 终端A：先起 mediaMTX
   cd ~/mediamtx && ./mediamtx
   # 终端B：USB摄像头 → RTSP（推给 mediaMTX 的 /usb 路径）
   ffmpeg -f v4l2 -input_format mjpeg -framerate 15 -video_size 640x480 -i /dev/video0 \
     -c:v libx264 -preset ultrafast -tune zerolatency -vf format=yuv420p \
     -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/usb
   ```
6. **检查 params.yaml**（2026-09-02 起已预置 VM 版，正常情况无需再改）：
   - 相机已指向假相机 `rtsp://127.0.0.1:8554/usb`，`fps: 15` 与终端B命令一致
   - `script_dir`/`model_path` 留空 = 只测推流、不跑 YOLO（日志会提示 `YOLO detection disabled`）
   - **想连识别一起测**：填回这两项（用户名换成 VM 的，如 `/home/czf/...`），
     模型用自带的 `det/yolo11n.pt` 即可
   - **回公司部署**：文件里 `公司工控机` 注释块里有现成的原值，取消注释换回来
7. **roslaunch 启动**，盯日志：应看到 `mediaMTX server OK (port 8554)` + 每 5 秒一条 `[heartbeat]`
8. **Windows 验证**：VM 里 `ip addr` 查 IP，浏览器直接开 `http://VM的IP:8889/raw`
   （mediaMTX 自带网页播放器，比 VLC 更快）；或 VLC 打开 `rtsp://VM的IP:8554/annotated`
   看带检测框画面。顺手再发一条开关命令（第 8.2 节）验证 UDP 链路

### 说明

- VM 里 CPU 推理 0.2~0.5 秒/帧属正常，**测的是链路通不通，不是速度**
- `dropping oldest frame` 警告：VM 里 4 核要同时跑"假相机编码+本节点解码+raw流编码"，
  偶尔跟不上帧率属正常（丢帧保护在工作），公司机器上不会这样
- **假相机画面花屏/绿屏/红屏**（VM 实测）：VM 的虚拟 USB 带宽有限，640x480 的 MJPEG
  数据会传坏（终端2 刷 `mjpeg decode error`）。三选一解决：
  1. 降分辨率：命令里 `-video_size 640x480` 改 `-video_size 320x240`（params.yaml 不用动，节点会自动缩放）
  2. 换像素格式：`-input_format mjpeg` 改 `-input_format yuyv422`（320x240 下更稳）
  3. 干脆不用摄像头，用视频文件当源（最稳，适合纯链路验证）：
     `ffmpeg -re -stream_loop -1 -i 任意视频.mp4 -vf scale=640:480 -c:v libx264 -preset ultrafast -tune zerolatency -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/usb`
- 假相机经 mediaMTX 中转，天然没有真相机"并发 RTSP 会话数限制"的问题，双路读取无忧
- 断流自愈：2026-09-02 起 C++ 节点带"断流自动重连+推流器自动重启"（修复记录 22 号），
  假相机挂了重启它，节点会自己在几秒内恢复，不用重启 roslaunch

---

## 许可证
# """
# rtsp_pusher.py - Python版RTSP推流器（使用ffmpeg）

# 功能：把YOLO标注后的画面帧通过ffmpeg子进程推到RTSP服务器

# 为什么用ffmpeg而不用GStreamer或OpenCV：
# ┌────────────────────────────────────────────────────────────────┐
# │ 方案                优点                    缺点               │
# ├────────────────────────────────────────────────────────────────┤
# │ GStreamer(C++)       性能最好                需要C++调用        │
# │ cv2.VideoWriter      简单                    大多数pip装的      │
# │                                            opencv不支持RTSP    │
# │ ffmpeg子进程         通用、稳定              需要系统安装ffmpeg  │
# └────────────────────────────────────────────────────────────────┘

# 推流流程：
#   标注帧(cv::Mat) → stdin喂入 → ffmpeg内部H.264编码 → RTSP发送

# ffmpeg命令参数详解：
#   -f rawvideo       输入格式：原始视频（无压缩的像素数据）
#   -pix_fmt bgr24    像素格式：BGR三通道（OpenCV默认）
#   -s WxH            视频尺寸：宽x高
#   -r FPS            帧率：每秒帧数
#   -i -              数据来源：从stdin（标准输入）读取
#   -c:v libx264      编码器：H.264（libx264是开源实现）
#   -preset ultrafast 编码速度：最快（牺牲压缩率换速度）
#   -tune zerolatency 优化目标：最低延迟
#   -b:v Nk           目标码率：N kbps
#   -g 30             关键帧间隔：30帧（1秒）
#   -f rtsp           输出格式：RTSP
#   -rtsp_transport tcp 传输协议：TCP（比UDP稳定）
# """

# import subprocess
# import sys
# import cv2
# import config


# class AnnotatedPusher:
#     """把YOLO标注后的画面帧通过ffmpeg持续推到RTSP服务器（mediaMTX）"""

#     def __init__(self, url):
#         """
#         初始化推流器
#         @param url: RTSP推流目标地址，如 "rtsp://127.0.0.1:8554/annotated"
#         """
#         self.url = url          # 推流目标地址
#         self.proc = None        # ffmpeg子进程对象
#         self.width = 0          # 视频宽度（启动时从第一帧获取）
#         self.height = 0         # 视频高度
#         self.died_notified = False  # 避免重复提示ffmpeg退出

#     def start(self, width, height):
#         """
#         启动ffmpeg推流进程
#         @param width: 视频宽度（像素）
#         @param height: 视频高度（像素）

#         注意：width/height必须是实际帧尺寸，之后写入的帧都会缩放到该尺寸
#         """
#         self.width = width
#         self.height = height

#         # 构建ffmpeg命令
#         cmd = [
#             "ffmpeg", "-y",              # -y: 覆盖输出文件（不提示）

#             # ========== 输入配置 ==========
#             "-f", "rawvideo",            # 输入格式：原始视频
#             "-pix_fmt", "bgr24",         # 像素格式：BGR（OpenCV默认，3通道）
#             "-s", f"{width}x{height}",   # 视频尺寸，如"640x480"
#             "-r", str(config.PUSH_FPS),  # 帧率，如"15"
#             "-i", "-",                   # 数据来源：stdin（标准输入）

#             # ========== 编码配置 ==========
#             "-c:v", "libx264",           # 视频编码器：H.264（libx264开源实现）
#             "-preset", "ultrafast",      # 编码速度预设：
#                                          #   ultrafast > superfast > veryfast > faster > fast > medium
#                                          #   速度越快，CPU占用越低，但压缩率越差
#             "-tune", "zerolatency",      # 优化模式：低延迟
#                                          #   适合实时推流场景，牺牲一点压缩率换取更小延迟
#             "-b:v", f"{config.PUSH_BITRATE}k",  # 目标码率，如"4000k"=4Mbps
#                                                  # 码率越大，画质越好，但带宽占用越大
#             "-g", "30",                  # 关键帧间隔（GOP大小）：30帧
#                                          # 设为帧率值表示每秒一个关键帧
#                                          # 关键帧越大，压缩率越高，但seek越慢

#             # ========== 输出配置 ==========
#             "-f", "rtsp",                # 输出格式：RTSP
#             "-rtsp_transport", "tcp",    # RTSP传输协议：TCP（比UDP稳定）
#             self.url,                    # 目标地址
#         ]

#         try:
#             # 启动ffmpeg子进程
#             # stdin=PIPE: 我们往里写帧数据
#             # stdout/stderr丢弃: 防止缓冲区塞满导致卡死
#             self.proc = subprocess.Popen(
#                 cmd,
#                 stdin=subprocess.PIPE,     # 标准输入：用于喂帧数据
#                 stdout=subprocess.DEVNULL, # 标准输出：丢弃
#                 stderr=subprocess.DEVNULL, # 标准错误：丢弃
#             )
#         except FileNotFoundError:
#             print("[推流] 找不到ffmpeg命令，请先安装: sudo apt install ffmpeg", file=sys.stderr)
#             self.proc = None
#             return False

#         print(f"[推流] 标注流已启动 -> {self.url}", file=sys.stderr)
#         return True

#     def write(self, frame):
#         """
#         写入一帧图像
#         @param frame: OpenCV图像帧（BGR格式）
#         @return: 成功返回True，失败返回False

#         工作流程：
#           1. 检查ffmpeg进程是否还活着
#           2. 缩放图像到目标尺寸（如果需要）
#           3. 把图像二进制数据写入ffmpeg的stdin
#           4. ffmpeg内部自动进行H.264编码+RTSP发送
#         """
#         if self.proc is None:
#             return False

#         # 检查ffmpeg进程是否还活着
#         # poll()返回None表示进程还在运行，否则返回退出码
#         if self.proc.poll() is not None:
#             if not self.died_notified:
#                 print("[推流] ffmpeg进程已退出（最常见原因：mediaMTX服务器没启动），本进程停止推流", file=sys.stderr)
#                 self.died_notified = True
#             return False

#         # 缩放图像到目标尺寸（如果需要）
#         h, w = frame.shape[:2]
#         if (w, h) != (self.width, self.height):
#             frame = cv2.resize(frame, (self.width, self.height))

#         try:
#             # 把图像二进制数据写入ffmpeg的stdin
#             # frame.tobytes() 把OpenCV图像转成字节数组
#             # ffmpeg会自动读取这些数据，进行H.264编码，然后RTSP发送
#             self.proc.stdin.write(frame.tobytes())
#         except BrokenPipeError:
#             # BrokenPipeError表示ffmpeg进程已关闭，无法写入
#             return False
#         return True

#     def release(self):
#         """
#         关闭推流器，释放资源

#         关闭流程：
#           1. 关闭stdin → ffmpeg知道没有更多数据了
#           2. 等待ffmpeg正常退出（最多3秒）
#           3. 如果超时，强制杀死ffmpeg进程
#         """
#         if self.proc is not None:
#             try:
#                 self.proc.stdin.close()      # 关闭stdin，让ffmpeg正常收尾
#                 self.proc.wait(timeout=3)    # 等待最多3秒
#             except Exception:
#                 self.proc.kill()             # 超时则强杀
#             self.proc = None
#             print("[推流] 推流器已关闭", file=sys.stderr)
MIT License
