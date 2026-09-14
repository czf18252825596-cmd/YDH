# usvc_vservo — 视觉伺服（目标跟随）功能包操作文档

> 版本：v0.1.0 ｜ 配套文档：`D:\桌面\USVC\docs\11_自主识别清障方案.md`
> 本文档放在包内，作为接入与使用的完整操作手册。

---

## 1. 包的定位：是"数据源"，不是"模式"

接入前先分清它占哪一层，这决定你怎么往 controller 里接。

```
L1 顶层模式(runMode)  ── 定航SAIL / 返航BACK / 任务TASK   （互斥，决定"去哪"）
L2 任务类型(task_type)── TASK 下：轨迹跟踪PATH_TRACKING / 沿边EDGE_FOLLOWING
L3 子功能/数据源       ── 沿边下：水岸掩码 water_on_left / 视觉伺服 vservo
```

**关键结论：**

- vservo **不是**和定航/返航/轨迹/沿边平级的"模式"。
- 它是**沿边模式（EDGE_FOLLOWING）下的一个可选数据源**，和水岸掩码同位。
- 它不改变 controller 的状态机，只在沿边分支里"决定读不读 `/usvc_vservo/cmd`"。
- 它和 planner 同性质：都只产期望量，由 controller 在沿边时**采信**。

> 所以接入 controller 时，加的不是"新模式"，而是"沿边分支里的一个数据来源选择 + 采信条件"。

---

## 2. 包的职责与边界

| | 做 | 不做 |
|---|---|---|
| 输入 | 订阅 `/yolo/detections`(DetectionArray)；`/usvc_vservo/ctrl`(启停)；`/usvc_vservo/ref_heading`(基准航向) | 不碰识别/推流 |
| 处理 | 多类归并 → 归一化 → 主目标选择 → 期望航向/速度 | 不管理控制/推进 |
| 输出 | `/usvc_vservo/cmd`(GuidanceCmd) | **不写 RunCtrl** |
| 隔离 | 独立进程，崩溃不影响 controller/yolo | — |

**崩溃隔离**：vservo 挂了，controller 只是"读不到伺服数据"，照常沿边或安全停下——因为它是被"采信"而非"必须依赖"。

---

## 3. 文件结构

```
usvc_vservo/
├── package.xml             依赖：std_msgs(纯基础)、usvc_planner(出 GuidanceCmd)、yolo_rtsp(收 DetectionArray)
├── CMakeLists.txt          消息生成 + vservo_node
├── msg/VservoCtrl.msg      启停(0不启用 / 1跟随)
├── src/vservo_node.cpp     核心节点
├── config/vservo.yaml      参数
└── launch/vservo.launch
```

---

## 4. 消息

**输入（跨包订阅）**

| 话题 | 类型 | 说明 |
|---|---|---|
| `/yolo/detections` | `yolo_rtsp/DetectionArray` | YOLO 输出：绝对像素框 `x,y,width,height` + 帧内 `image_width/height` |
| `/usvc_vservo/ctrl` | `usvc_vservo/VservoCtrl` | 启停开关（controller / 上位机桥接下发的入口） |
| `/usvc_vservo/ref_heading` | `std_msgs/Float32` | 沿边基准航向(绝对,deg)，运行中可更新 |

**输出**

| 话题 | 类型 | 说明 |
|---|---|---|
| `/usvc_vservo/cmd` | `usvc_planner/GuidanceCmd` | 期望量：`obj_speed` / `obj_heading` / `reached` |

> 复用 planner 的 `GuidanceCmd`，是为让 controller 用一个统一的"期望量来源"概念去采信——vservo 和 planner 输出同结构，桥接一致。

---

## 5. 节点逻辑（vservo_node.cpp）

```
/yolo/detections
  │ ① 白名单过滤   class_id ∈ target_class_ids（多类归并，剔船/人）
  ▼
  │ ② 置信度门槛   confidence ≥ conf_thresh
  ▼
  │ ③ 归一化       cx=(x+w/2)/W, cy=(y+h/2)/H, w=w/W, h=h/H   → 0~1
  ▼
  │ ④ 主目标       面积最大（最贴镜头、最可能是要清的垃圾）
  ▼
  │ ⑤ 期望量
  │    obj_heading = 基准艏向 + (cx−0.5)×FOV_h
  │    obj_speed   = 按归一化面积 S 减速（越大越近越慢）
  ▼
  │ ⑥ reached      S≥reached_area_thresh  或  (cy+h/2)触及底边(进收集盲区) → 停车
  ▼
 /usvc_vservo/cmd
```

**多类归并的意义**：模型不准时，不要求"具体是哪类垃圾"，只要识别成任一白名单垃圾类就视为目标去清——彻底规避"识别错类别就漏清"。

---

## 6. 参数（config/vservo.yaml）

| 参数 | 默认 | 说明 |
|---|---|---|
| `target_class_ids` | `[0,3,5,7]` | 目标类别白名单（多类归并），**按训练模型 yaml 的 names 填** |
| `conf_thresh` | 0.5 | 置信度阈值，模型不准时调低 |
| `fov_h` | 60.0 | 相机水平 FOV(deg)，需标定 |
| `ref_heading_deg` | 0.0 | 沿边基准航向(绝对deg)，可被话题更新 |
| `approach_speed` | 1.0 | 接近最大速度(m/s) |
| `reached_area_thresh` | 0.35 | 面积达该阈值判 reached |
| `bottom_thresh` | 0.92 | (cy+h/2) 超该值判进收集盲区 |
| `slow_area` | 0.10 | 面积到该值开始线性减速 |

---

## 7. 编译

在 Linux / 工控机（有 ROS Noetic）：

```bash
cd ~/usvc_ws
catkin_make --pkg usvc_planner yolo_rtsp   # 若这两个包的头未生成，先单独编
catkin_make                                # 全编
source devel/setup.bash
```

> 依赖顺序：vservo 依赖 usvc_planner、yolo_rtsp 的消息头，catkin 会自动排序；报 `find ... DetectionArray.h` 之类找不到时，先把那两包编一遍。

---

## 8. 独立冒烟验证（先不接 controller）

```bash
roslaunch usvc_vservo vservo.launch
```

另开终端：

```bash
# 喂一帧假检测（垃圾在画面中心偏左）
rostopic pub -1 /yolo/detections yolo_rtsp/DetectionArray \
  "{header:{}, frame_count:1, image_width:1920, image_height:1080,
    detections:[{class_id:0, confidence:0.9, x:700, y:400, width:180, height:120}]}"

# 开使能
rostopic pub -1 /usvc_vservo/ctrl usvc_vservo/VservoCtrl "{run_mode: 1, task_state: 1}"

# 看输出
rostopic echo /usvc_vservo/cmd
```

预期：`obj_speed` 非零，`obj_heading` = 基准 + (0.42−0.5)×FOV(负偏，追左偏垃圾)，`reached=false`。
把检测 center 移到 `x:960`(正中) → `θ=0`。

---

## 9. 接入 controller（桥接落位）

遵循"唯一写者 + 采信"原则，vservo **不写 RunCtrl**，由 controller 在沿边分支采信：

```
上电/沿边进入 ── controller 沿边分支 ── 是否采信 vservo？
   采信条件：runMode==TASK 且 task_type==EDGE_FOLLOWING 且 识别使能
   │
   ├─ 有目标 → objHeading/objSpeed ← /usvc_vservo/cmd（追垃圾）
   ├─ 无目标 → 沿边正常（planner 采信）
   └─ 全程 /control/RunCtrl 仍唯一由 controller 发布
```

改 controller（control_node）大致：
1. 订阅 `/usvc_vservo/cmd`（GuidanceCmd）→ 存数据槽。
2. 沿边分支里：使能且收到伺服数据 → 采信 vservo 的值填进 RunCtrl；否则沿用边。
3. 触发时给 `/usvc_vservo/ctrl` 发使能（或上位机开关传达）。

> 详细 pattern 同 planner 接入：参见 `docs/07_planner接入详细蓝图.md`、`docs/08_总入手顺序.md` 的 A~D 步骤方法论。

---

## 10. 已知待确认点

1. **垃圾 class_id**：`target_class_ids` 必须按你的训练模型 yaml 的 names 字典填，否则什么都追不到（或误追船）。
2. **相机 FOV**：`fov_h` 需标定，直接影响 θ 换算精度。
3. **reached 改进可选**：拾获完成目前"面积大或贴底边"即停；如需"保持盲开 X 秒"可后续加。
4. **自持消息（优化项）**：当前依赖 yolo/planner 的消息类型；未来可自持并加薄适配，使包零第三方依赖更可移植。**本次保持现状，功能优先。**