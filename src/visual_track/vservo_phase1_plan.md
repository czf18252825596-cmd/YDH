# usvc_vservo 第一阶段实施计划

> 版本：v1.0 | 日期：2026-09-05 | 状态：待实施

---

## 1. 目标概述

### 1.1 功能目标

在沿边模式下，当检测到垃圾时：
1. 自动暂停当前沿边任务
2. 跟随垃圾进行清理
3. 清理完成后，从原路线最近点继续沿边

### 1.2 技术方案

- **视觉测距**：仅使用 YOLO 检测框面积估算距离（第一阶段）
- **多目标队列**：按距离排序，依次清理
- **状态机**：管理清理流程
- **控制器集成**：在沿边分支中添加 vservo 采信逻辑

---

## 2. 硬件配置确认

| 项目 | 值 |
|------|-----|
| **相机型号** | 海康 DS-2DC2D40IW-DE3 |
| **分辨率** | 2560×1440（400万像素） |
| **焦距** | 2.8-12mm（4倍光学变倍） |
| **水平FOV** | 97°（广角）~ 32°（望远） |
| **最小聚焦距离** | 1.0米 |
| **安装方式** | 倒置安装 |
| **俯角** | 正常23° → 倒置后67° |
| **图像方向** | 已校正（正的） |

### 模型类别（best.pt）

| class_id | 类别 | 是否目标 |
|----------|------|---------|
| 0 | Human | ❌ 排除 |
| 1 | Passing_Boat | ❌ 排除 |
| 2 | Bottle | ✅ 垃圾 |
| 3 | Buoy | ❌ 排除（浮标非垃圾） |
| 4 | Aquatic_Vegetation | ✅ 垃圾 |
| 5 | Floating_Foam | ✅ 垃圾 |
| 6 | Other_Debris | ✅ 垃圾 |

---

## 3. 修改文件清单

### 第一阶段：完善 usvc_vservo 功能包

| 文件 | 修改内容 | 工作量 |
|------|---------|--------|
| `usvc_vservo/msg/GarbageTarget.msg` | **新建**：单个垃圾目标消息 | 5分钟 |
| `usvc_vservo/msg/GarbageTargetArray.msg` | **新建**：垃圾目标数组消息 | 5分钟 |
| `usvc_vservo/CMakeLists.txt` | 添加新消息文件 | 5分钟 |
| `usvc_vservo/config/vservo.yaml` | 更新参数配置 | 5分钟 |
| `usvc_vservo/src/vservo_node.cpp` | 支持多目标输出 + 距离估算 | 2小时 |

### 第二阶段：并入 usvc_controller

| 文件 | 修改内容 | 工作量 |
|------|---------|--------|
| `usvc_controller/CMakeLists.txt` | 添加 usvc_vservo 依赖 | 5分钟 |
| `usvc_controller/package.xml` | 添加 usvc_vservo 依赖 | 5分钟 |
| `usvc_controller/src/control/control.h` | 添加 EdgeSubState 枚举 | 10分钟 |
| `usvc_controller/src/control_node.cpp` | 添加 vservo 桥接 + 状态机 + 路径恢复 | 4小时 |
| `usvc_bringup/launch/usvc_bringup.launch` | 添加 vservo.launch | 5分钟 |

---

## 4. 第一阶段详细实施

### 4.1 创建新消息类型

#### GarbageTarget.msg

**文件路径**：`/home/nvidia/USVC/usvc_ws/src/usvc_vservo/msg/GarbageTarget.msg`

```msg
# 单个垃圾目标
uint8 id                # 目标ID（用于去重）
float32 cx              # 归一化中心x (0~1)
float32 cy              # 归一化中心y (0~1)
float32 area            # 归一化面积 (0~1)
float32 distance        # 估算距离 (米)
bool is_current         # 是否当前目标
```

#### GarbageTargetArray.msg

**文件路径**：`/home/nvidia/USVC/usvc_ws/src/usvc_vservo/msg/GarbageTargetArray.msg`

```msg
# 垃圾目标数组
Header header
usvc_vservo/GarbageTarget[] targets    # 目标列表（按距离排序）
```

---

### 4.2 修改 CMakeLists.txt

**文件路径**：`/home/nvidia/USVC/usvc_ws/src/usvc_vservo/CMakeLists.txt`

**修改内容**：在 `add_message_files` 中添加新消息

```cmake
add_message_files(
  FILES
  VservoCtrl.msg
  GarbageTarget.msg        # 新增
  GarbageTargetArray.msg   # 新增
)
```

---

### 4.3 更新 vservo.yaml 配置

**文件路径**：`/home/nvidia/USVC/usvc_ws/src/usvc_vservo/config/vservo.yaml`

```yaml
# 目标类别白名单（根据 best.pt 模型）
target_class_ids: [2, 4, 5, 6]  # Bottle, Aquatic_Vegetation, Floating_Foam, Other_Debris

# 置信度阈值
conf_thresh: 0.4

# 相机水平 FOV
fov_h: 60.0

# 图像方向（已校正，不需要翻转）
image_inverted: false

# 相机参数
camera_height: 1.0      # 安装高度（米）- 后续根据实际调整
tilt_angle: 67.0        # 俯角（度）- 倒置安装时 90°-23°=67°
focal_length: 6.0       # 焦距（mm）
min_focus_distance: 1.0  # 最小聚焦距离（米）

# 沿边基准航向
ref_heading_deg: 0.0

# 接近目标的最大速度 (m/s)
approach_speed: 0.8

# reached 判据
reached_area_thresh: 0.07   # 安全停车距离约1.3米
bottom_thresh: 0.75         # 不要太贴近底部

# 减速区
slow_area: 0.03
```

---

### 4.4 修改 vservo_node.cpp

**文件路径**：`/home/nvidia/USVC/usvc_ws/src/usvc_vservo/src/vservo_node.cpp`

#### 4.4.1 添加新头文件

```cpp
// 在现有 include 后添加
#include "usvc_vservo/GarbageTarget.h"
#include "usvc_vservo/GarbageTargetArray.h"
```

#### 4.4.2 添加新成员变量

```cpp
// 在类成员变量中添加
ros::Publisher pub_targets_;

// 新增参数
float camera_height_ = 1.0f;
float tilt_angle_ = 45.0f;
float focal_length_ = 6.0f;
float min_focus_distance_ = 1.0f;
bool image_inverted_ = false;
```

#### 4.4.3 修改构造函数

```cpp
VservoNode() {
    ros::NodeHandle pnh("~");

    // 现有参数...
    pnh.getParam("conf_thresh",        conf_thresh_);
    pnh.getParam("fov_h",              fov_h_deg_);
    pnh.getParam("ref_heading_deg",    ref_heading_deg_);
    pnh.getParam("approach_speed",     approach_speed_);
    pnh.getParam("reached_area_thresh",reached_area_thresh_);
    pnh.getParam("bottom_thresh",      bottom_thresh_);
    pnh.getParam("slow_area",          slow_area_);
    std::vector<int> ids;
    pnh.getParam("target_class_ids", ids);
    for (int i : ids) target_class_ids_.insert(i);

    // 新增参数
    pnh.getParam("camera_height", camera_height_);
    pnh.getParam("tilt_angle", tilt_angle_);
    pnh.getParam("focal_length", focal_length_);
    pnh.getParam("min_focus_distance", min_focus_distance_);
    pnh.getParam("image_inverted", image_inverted_);

    // 话题
    sub_det_    = nh_.subscribe("/yolo/detections", 1, &VservoNode::onDetection, this);
    sub_ctrl_   = nh_.subscribe("/usvc_vservo/ctrl", 1, &VservoNode::onCtrl, this);
    sub_refhdg_ = nh_.subscribe("/usvc_vservo/ref_heading", 1, &VservoNode::onRefHeading, this);
    pub_cmd_    = nh_.advertise<usvc_planner::GuidanceCmd>("/usvc_vservo/cmd", 1);
    pub_targets_ = nh_.advertise<usvc_vservo::GarbageTargetArray>("/usvc_vservo/targets", 1);  // 新增

    timer_ = nh_.createTimer(ros::Duration(0.05), &VservoNode::onPublish, this);  // 20 Hz
    ROS_INFO("[vservo] up. classes={%s} conf=%.2f fov_h=%.1f min_focus=%.1f",
             joinClassIds().c_str(), conf_thresh_, fov_h_deg_, min_focus_distance_);
}
```

#### 4.4.4 添加距离估算函数

```cpp
// 距离估算（仅视觉，基于面积）
float estimateDistance(float normalized_area) {
    // K 值需要根据实际标定
    // 初始值 0.3，后续根据实际测试调整
    float K = 0.3f;
    return K / std::sqrt(normalized_area + 0.001f);
}
```

#### 4.4.5 修改 collectTargets 函数（支持倒置）

```cpp
// 从最近一帧筛出候选（白名单归类 + 置信度门槛）并归一化
std::vector<Target> collectTargets() {
    std::vector<Target> out;
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_det_) return out;
    const auto& d = last_det_;
    float W = d.image_width > 0  ? static_cast<float>(d.image_width)  : 1920.f;
    float H = d.image_height > 0 ? static_cast<float>(d.image_height) : 1080.f;
    for (const auto& det : d.detections) {
      if (std::find(target_class_ids_.begin(), target_class_ids_.end(), det.class_id)
          == target_class_ids_.end()) continue;
      if (det.confidence < conf_thresh_) continue;
      Target t;
      t.class_id = det.class_id;
      float cx = det.x + det.width  * 0.5f;
      float cy = det.y + det.height * 0.5f;
      t.cx = cx / W;
      // 根据图像方向处理 y 坐标
      if (image_inverted_) {
          t.cy = 1.0f - (cy / H);  // 倒置：反转y坐标
      } else {
          t.cy = cy / H;            // 正常
      }
      t.w  = static_cast<float>(det.width)  / W;
      t.h  = static_cast<float>(det.height) / H;
      t.area = t.w * t.h;
      out.push_back(t);
    }
    return out;
}
```

#### 4.4.6 修改 onPublish 函数

```cpp
void onPublish(const ros::TimerEvent&) {
    auto cands = collectTargets();
    
    // 发布目标数组（新增）
    usvc_vservo::GarbageTargetArray targets_msg;
    targets_msg.header.stamp = ros::Time::now();
    
    int id_counter = 0;
    for (const auto& c : cands) {
        usvc_vservo::GarbageTarget target;
        target.id = id_counter++;
        target.cx = c.cx;
        target.cy = c.cy;
        target.area = c.area;
        target.distance = estimateDistance(c.area);
        target.is_current = false;
        targets_msg.targets.push_back(target);
    }
    
    // 按距离排序
    std::sort(targets_msg.targets.begin(), targets_msg.targets.end(),
        [](const usvc_vservo::GarbageTarget& a, const usvc_vservo::GarbageTarget& b) {
            return a.distance < b.distance;
        });
    
    // 标记当前目标
    if (!targets_msg.targets.empty()) {
        targets_msg.targets[0].is_current = true;
    }
    
    pub_targets_.publish(targets_msg);
    
    // 原有 GuidanceCmd 逻辑保持不变
    usvc_planner::GuidanceCmd cmd;
    cmd.is_avoiding = false;
    cmd.reached  = false;

    if (!enabled_ || cands.empty()) {
      cmd.obj_speed   = 0.f;
      cmd.obj_heading = normDeg(ref_heading_deg_);
      pub_cmd_.publish(cmd);
      return;
    }

    // 主目标 = 面积最大（最贴近镜头、最可能是要清的垃圾）
    const Target* main = &cands[0];
    for (const auto& c : cands) if (c.area > main->area) main = &c;

    // ① 艏向：中心偏移 × 水平 FOV
    float theta = (main->cx - 0.5f) * (fov_h_deg_ * PI / 180.f) * 180.f / PI;
    cmd.obj_heading = normDeg(ref_heading_deg_ + theta);
    cmd.is_avoiding = false;

    // ② 速度：按归一化面积 S 代理距离，越大越近越慢
    float S = main->area;
    bool reached = (S >= reached_area_thresh_)
                || (main->cy + main->h * 0.5f >= bottom_thresh_);
    if (reached) {
      cmd.obj_speed = 0.f;
      cmd.reached  = true;
    } else {
      float k = 0.f;
      if (S > slow_area_)
        k = std::clamp((S - slow_area_) / (reached_area_thresh_ - slow_area_), 0.f, 1.f);
      cmd.obj_speed = approach_speed_ * (1.f - k);
    }

    pub_cmd_.publish(cmd);
}
```

---

## 5. 编译验证

### 5.1 编译顺序

```bash
cd /home/nvidia/USVC/usvc_ws

# 1. 编译消息依赖
catkin_make --pkg usvc_planner yolo_rtsp

# 2. 编译 vservo
catkin_make --pkg usvc_vservo

# 3. 刷新环境
source devel/setup.bash
```

### 5.2 单元测试

```bash
# 终端1：启动 vservo
roslaunch usvc_vservo vservo.launch

# 终端2：喂假检测数据
rostopic pub -1 /yolo/detections yolo_rtsp/DetectionArray \
  "{header:{}, frame_count:1, image_width:1920, image_height:1080,
    detections:[
      {class_id:2, confidence:0.9, x:700, y:400, width:180, height:120},
      {class_id:5, confidence:0.85, x:1200, y:350, width:150, height:100}
    ]}"

# 终端3：启用 vservo
rostopic pub -1 /usvc_vservo/ctrl usvc_vservo/VservoCtrl "{run_mode: 1, task_state: 1}"

# 终端4：观察输出
rostopic echo /usvc_vservo/targets
rostopic echo /usvc_vservo/cmd
```

### 5.3 预期输出

```
/usvc_vservo/targets:
  - id: 0
    cx: 0.42
    cy: 0.58
    area: 0.019
    distance: 2.17
    is_current: true
  - id: 1
    cx: 0.67
    cy: 0.42
    area: 0.012
    distance: 2.74
    is_current: false

/usvc_vservo/cmd:
  obj_speed: 0.8
  obj_heading: 285.0
  is_avoiding: false
  reached: false
```

---

## 6. 第二阶段：并入 usvc_controller

### 6.1 修改 CMakeLists.txt

**文件路径**：`/home/nvidia/USVC/usvc_ws/src/usvc_controller/CMakeLists.txt`

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  roslib  
  pcl_conversions
  message_generation
  usvc_planner
  usvc_vservo    # 新增
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime pcl_conversions usvc_planner usvc_vservo
)
```

### 6.2 修改 package.xml

**文件路径**：`/home/nvidia/USVC/usvc_ws/src/usvc_controller/package.xml`

```xml
<build_depend>usvc_vservo</build_depend>
<exec_depend>usvc_vservo</exec_depend>
```

### 6.3 修改 control.h

**文件路径**：`/home/nvidia/USVC/usvc_ws/src/usvc_controller/src/control/control.h`

```cpp
// 在 enum TASK_STATUS_ENUM 后面添加

// 沿边+识别 子状态
enum EdgeSubState {
    EDGE_FOLLOW = 0,    // 正常沿边（使用 planner）
    GARBAGE_FOUND,      // 发现垃圾，准备切换
    CLEANING,           // 正在清理（使用 vservo）
    CLEAN_DONE,         // 清理完成，准备恢复
    RESUMING            // 恢复中
};
```

### 6.4 修改 control_node.cpp

**文件路径**：`/home/nvidia/USVC/usvc_ws/src/usvc_controller/src/control_node.cpp`

#### 6.4.1 添加头文件

```cpp
// 在现有 include 后添加
#include "usvc_vservo/VservoCtrl.h"
#include "usvc_vservo/GarbageTargetArray.h"
#include "std_msgs/Float32.h"
```

#### 6.4.2 添加全局变量

```cpp
// vservo 桥接
static ros::Subscriber vservo_cmd_sub;
static ros::Subscriber vservo_targets_sub;
static ros::Publisher  vservo_ctrl_pub;
static ros::Publisher  vservo_ref_heading_pub;

// 清理队列
struct GarbageTargetSimple {
    int id;
    float cx, cy, area, distance;
    bool cleaned;
};

static std::vector<GarbageTargetSimple> cleanup_queue;
static int current_target_idx = 0;
static EdgeSubState edge_sub_state = EDGE_FOLLOW;
static bool vservo_has_target = false;
static bool vservo_reached = false;

// 保存原路径用于恢复
static std::vector<Eigen::Vector2d> saved_path;
static bool path_saved = false;
```

#### 6.4.3 添加回调函数

```cpp
// vservo 输出回调（复用 planner 的 Edge_ctrl）
static void vservoCmdCallback(const usvc_planner::GuidanceCmd& msg)
{
    // 只有在 CLEANING 状态才更新 Edge_ctrl
    if (edge_sub_state == CLEANING) {
        Edge_ctrl ec;
        ec.objSpeed   = msg.obj_speed;
        ec.objHeading = msg.obj_heading;
        ec.isAvoiding = msg.is_avoiding;
        ec.reached    = msg.reached;
        m_Control.setEdgeCtrl(ec);
        
        vservo_reached = msg.reached;
    }
    
    vservo_has_target = (msg.obj_speed > 0.f);
}

// vservo 目标数组回调
static void vservoTargetsCallback(const usvc_vservo::GarbageTargetArray& msg)
{
    if (edge_sub_state != EDGE_FOLLOW) return;  // 只在沿边状态添加新目标
    
    for (const auto& target : msg.targets) {
        // 去重检查
        bool exists = false;
        for (const auto& existing : cleanup_queue) {
            float dist = std::sqrt(std::pow(existing.cx - target.cx, 2) + 
                                   std::pow(existing.cy - target.cy, 2));
            if (dist < 0.1f) {  // 归一化距离 < 0.1 视为同一目标
                exists = true;
                break;
            }
        }
        
        if (!exists) {
            GarbageTargetSimple t;
            t.id = target.id;
            t.cx = target.cx;
            t.cy = target.cy;
            t.area = target.area;
            t.distance = target.distance;
            t.cleaned = false;
            cleanup_queue.push_back(t);
        }
    }
    
    // 按距离排序
    std::sort(cleanup_queue.begin(), cleanup_queue.end(),
        [](const GarbageTargetSimple& a, const GarbageTargetSimple& b) {
            return a.distance < b.distance;
        });
}
```

#### 6.4.4 添加恢复函数

```cpp
// 恢复到原路线最近点
static void resumeToNearestPoint(double current_lat, double current_lon)
{
    if (saved_path.empty()) {
        ROS_WARN("[RESUME] 无保存路径，无法恢复");
        return;
    }
    
    // 找到最近的路径点
    int nearest_idx = 0;
    double min_dist = 1e9;
    
    for (size_t i = 0; i < saved_path.size(); ++i) {
        double dlat = saved_path[i].y() - current_lat;
        double dlon = saved_path[i].x() - current_lon;
        double dist = dlat * dlat + dlon * dlon;
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    
    // 构造新路径
    usvc_planner::LosPath lp;
    lp.header.stamp = ros::Time::now();
    lp.header.frame_id = "usv";
    
    for (size_t i = nearest_idx; i < saved_path.size(); ++i) {
        geometry_msgs::PoseStamped ps;
        ps.header = lp.header;
        ps.pose.position.x = saved_path[i].x();  // lon
        ps.pose.position.y = saved_path[i].y();  // lat
        lp.path.poses.push_back(ps);
    }
    
    edge_path_pub.publish(lp);
    
    // 重启 planner
    usvc_planner::OtterCtrl oc;
    oc.run_mode = 0;
    oc.task_state = 1;
    oc.boundary_select = 0;
    edge_ctrl_pub.publish(oc);
    
    ROS_INFO("[RESUME] 从航点 %d/%zu 恢复沿边", nearest_idx, saved_path.size());
}
```

#### 6.4.5 修改 plannerFeedIfNeeded 函数

```cpp
static void plannerFeedIfNeeded(const RunCtrl_t& runCtrl)
{
    bool wantRun = (runCtrl.runMode    == RUN_MODE_ENUM::TASK)
                && (runCtrl.task_state == TASK_STATUS_ENUM::BEGIN);
    if(wantRun)
    {
        auto tl = m_Control.getTaskFile();
        bool has_ef = false;
        for(auto& ti : tl) if(ti.first == TASK_TYPE_ENUM::EDGE_FOLLOWING) { has_ef = true; break; }
        wantRun = has_ef;
    }

    if(wantRun && !planner_loaded)
    {
        planner_loaded = true;

        // 1) 从任务文件取当前 EDGE_FOLLOWING 的航点，构造 LosPath
        auto task_list = m_Control.getTaskFile();
        usvc_planner::LosPath lp;
        lp.header.stamp = ros::Time::now();
        lp.header.frame_id = "usv";
        bool water_left = false;
        
        // 保存原路径用于恢复
        saved_path.clear();
        
        for(size_t i = 0; i < task_list.size(); ++i)
        {
            if(task_list[i].first != TASK_TYPE_ENUM::EDGE_FOLLOWING) continue;
            for(const auto& kv : task_list[i].second)
            {
                if(kv.first == PLANNER_WATER_KEY)
                {
                    water_left = (!kv.second.empty() && kv.second[0] != 0.0);
                    continue;
                }
                geometry_msgs::PoseStamped ps;
                ps.header = lp.header;
                ps.pose.position.x = kv.second[1];   // lon → x
                ps.pose.position.y = kv.second[0];   // lat → y
                lp.path.poses.push_back(ps);
                
                // 保存路径点
                saved_path.emplace_back(kv.second[1], kv.second[0]);
            }
            break;
        }
        lp.water_is_on_left = water_left;
        edge_path_pub.publish(lp);

        // 2) 启动 planner（沿边模式）
        usvc_planner::OtterCtrl oc;
        oc.run_mode        = 0;    // ALONG_EDGE
        oc.task_state      = 1;    // 开始
        oc.boundary_select = 0;
        edge_ctrl_pub.publish(oc);

        // 3) 重置清理状态
        edge_sub_state = EDGE_FOLLOW;
        cleanup_queue.clear();
        current_target_idx = 0;
        vservo_has_target = false;
        vservo_reached = false;
        path_saved = true;

        ROS_INFO("[control_node] planner 已装载，进入沿边模式");
    }
    else if(!wantRun && runCtrl.runMode != RUN_MODE_ENUM::TASK)
    {
        planner_loaded = false;
        path_saved = false;
        
        // 停止 planner
        usvc_planner::OtterCtrl oc;
        oc.run_mode = 0; oc.task_state = 0; oc.boundary_select = 0;
        edge_ctrl_pub.publish(oc);
        
        // 重置状态
        edge_sub_state = EDGE_FOLLOW;
        cleanup_queue.clear();
    }
}
```

#### 6.4.6 修改 main 函数初始化

```cpp
// 在 planner 桥接附近添加

// planner 桥接
edge_cmd_sub = nh.subscribe("/usvc_planner/cmd",  10, plannerCmdCallback);
edge_path_pub = nh.advertise<usvc_planner::LosPath>("/usvc_planner/path", 1);
edge_ctrl_pub = nh.advertise<usvc_planner::OtterCtrl>("/usvc_planner/ctrl", 1);

// vservo 桥接（新增）
vservo_cmd_sub = nh.subscribe("/usvc_vservo/cmd", 10, vservoCmdCallback);
vservo_targets_sub = nh.subscribe("/usvc_vservo/targets", 10, vservoTargetsCallback);
vservo_ctrl_pub = nh.advertise<usvc_vservo::VservoCtrl>("/usvc_vservo/ctrl", 1);
vservo_ref_heading_pub = nh.advertise<std_msgs::Float32>("/usvc_vservo/ref_heading", 1);
```

#### 6.4.7 修改 executeTask 中的 EDGE_FOLLOWING 分支

```cpp
case TASK_TYPE_ENUM::EDGE_FOLLOWING:
{
    Edge_ctrl ec;
    
    switch (edge_sub_state) {
        case EDGE_FOLLOW:
        {
            // 正常沿边：使用 planner 的值
            ec = getEdgeCtrl();
            
            // 检查是否发现垃圾
            if (!cleanup_queue.empty()) {
                edge_sub_state = GARBAGE_FOUND;
                current_target_idx = 0;
                ROS_INFO("[EDGE] 发现 %zu 个垃圾目标，准备切换", cleanup_queue.size());
            }
            break;
        }
        
        case GARBAGE_FOUND:
        {
            // 切换到 vservo
            edge_sub_state = CLEANING;
            ROS_INFO("[EDGE] 切换到视觉跟随，目标 %d/%zu", 
                     current_target_idx + 1, cleanup_queue.size());
            
            // 发送 vservo 使能
            usvc_vservo::VservoCtrl vc;
            vc.run_mode   = 1;
            vc.task_state = 1;
            vservo_ctrl_pub.publish(vc);
            
            // 发送基准航向
            std_msgs::Float32 ref_heading;
            ref_heading.data = getEdgeCtrl().objHeading;  // 当前航向作为基准
            vservo_ref_heading_pub.publish(ref_heading);
            
            ec = getEdgeCtrl();
            break;
        }
        
        case CLEANING:
        {
            // 正在清理：使用 vservo 的值
            ec = getEdgeCtrl();
            
            // 检查是否到达
            if (vservo_reached) {
                // 标记当前目标完成
                if (current_target_idx < cleanup_queue.size()) {
                    cleanup_queue[current_target_idx].cleaned = true;
                }
                
                // 检查是否全部完成
                bool all_cleaned = true;
                for (const auto& target : cleanup_queue) {
                    if (!target.cleaned) {
                        all_cleaned = false;
                        break;
                    }
                }
                
                if (all_cleaned) {
                    // 全部清理完成
                    edge_sub_state = CLEAN_DONE;
                    ROS_INFO("[EDGE] 全部清理完成，准备恢复");
                } else {
                    // 还有目标
                    current_target_idx++;
                    while (current_target_idx < cleanup_queue.size() && 
                           cleanup_queue[current_target_idx].cleaned) {
                        current_target_idx++;
                    }
                    
                    if (current_target_idx < cleanup_queue.size()) {
                        ROS_INFO("[EDGE] 下一个目标: %d/%zu", 
                                 current_target_idx + 1, cleanup_queue.size());
                    } else {
                        edge_sub_state = CLEAN_DONE;
                        ROS_INFO("[EDGE] 无更多目标，准备恢复");
                    }
                }
            }
            break;
        }
        
        case CLEAN_DONE:
        {
            // 清理完成：停止 vservo
            usvc_vservo::VservoCtrl vc;
            vc.run_mode   = 0;
            vc.task_state = 0;
            vservo_ctrl_pub.publish(vc);
            
            // 恢复到原路线
            NAVIGnssData_t gnss = m_Control.getNAVIGnssData();
            resumeToNearestPoint(gnss.latitude, gnss.longitude);
            
            edge_sub_state = RESUMING;
            cleanup_queue.clear();
            current_target_idx = 0;
            
            ROS_INFO("[EDGE] 恢复沿边");
            
            ec = getEdgeCtrl();
            break;
        }
        
        case RESUMING:
        {
            // 恢复中：使用 planner 的值
            ec = getEdgeCtrl();
            
            // 稳定后回到正常沿边
            edge_sub_state = EDGE_FOLLOW;
            break;
        }
    }
    
    Ctrl_Result.ctrl_mode       = 1;
    Ctrl_Result.Y               = 0;
    Ctrl_Result.H               = 0;
    Ctrl_Result.objSpeed        = ec.objSpeed;
    Ctrl_Result.objHeading      = ec.objHeading;
    Ctrl_Result.isAvoiding      = ec.isAvoiding;
    Ctrl_Result.ctrl_finished   = ec.reached;
    
    if (Ctrl_Result.ctrl_finished)
        task_index++;
    break;
}
```

### 6.5 更新 launch 文件

**文件路径**：`/home/nvidia/USVC/usvc_ws/src/usvc_bringup/launch/usvc_bringup.launch`

```xml
<launch>
    <!-- 现有启动 -->
    <include file="$(find livox_ros_driver2)/launch_ROS1/rviz_MID360.launch" />
    <include file="$(find usvc_navi)/launch/usvc_navi_bringup.launch" />
    <include file="$(find usvc_controller)/launch/usvc_controller_bringup.launch" />
    <include file="$(find usvc_planner)/launch/usvc_planner.launch"/>
    <include file="$(find yolo_rtsp)/launch/yolo_rtsp.launch"/>
    
    <!-- 新增：vservo 启动 -->
    <include file="$(find usvc_vservo)/launch/vservo.launch"/>
</launch>
```

---

## 7. 编译顺序（第二阶段）

```bash
cd /home/nvidia/USVC/usvc_ws

# 1. 编译 vservo（已有消息）
catkin_make --pkg usvc_vservo

# 2. 编译 controller（依赖 vservo）
catkin_make --pkg usvc_controller

# 3. 全量编译
catkin_make

# 4. 刷新环境
source devel/setup.bash
```

---

## 8. 状态机流程图

```
                    ┌──────────────────┐
                    │   EDGE_FOLLOW    │
                    │  (正常沿边)      │
                    └────────┬─────────┘
                             │ 发现垃圾
                             ▼
                    ┌──────────────────┐
                    │  GARBAGE_FOUND   │
                    │  (准备切换)      │
                    └────────┬─────────┘
                             │ 启用 vservo
                             ▼
                    ┌──────────────────┐
                    │    CLEANING      │
                    │  (正在清理)      │
                    └────────┬─────────┘
                             │ vservo_reached
                             ▼
                    ┌──────────────────┐
                    │   CLEAN_DONE     │
                    │  (清理完成)      │
                    └────────┬─────────┘
                             │ 恢复路径
                             ▼
                    ┌──────────────────┐
                    │    RESUMING      │
                    │  (恢复中)        │
                    └────────┬─────────┘
                             │ 稳定后
                             ▼
                    ┌──────────────────┐
                    │   EDGE_FOLLOW    │
                    │  (正常沿边)      │
                    └──────────────────┘
```

---

## 9. 注意事项

### 9.1 调试要点

1. **距离系数 K**：需要实际标定，不同相机/垃圾大小会影响
2. **去重阈值**：太小会重复清理，太大会漏掉目标
3. **状态切换**：观察日志确认状态机转换正确

### 9.2 已知限制

1. **静态垃圾**：第一阶段只处理静态垃圾
2. **单帧检测**：不处理跨帧跟踪
3. **无深度**：距离估算基于面积，精度一般

### 9.3 后续优化方向

1. 雷达融合提高距离精度
2. 添加目标跟踪（跨帧关联）
3. 动态垃圾预测
4. 优化清理路径规划

---

## 10. 风险评估

| 风险 | 影响 | 缓解措施 |
|------|------|---------|
| 距离系数 K 不准 | 清理效果差 | 实际标定，提供默认值 |
| 去重失败 | 重复清理 | 调整阈值，添加位置+面积联合判断 |
| 状态机卡死 | 船停在原地 | 添加超时保护 |
| vservo 崩溃 | 无法清理 | 崩溃隔离，自动恢复沿边 |

---

## 11. 完成标准

- [ ] 消息类型编译通过
- [ ] vservo 输出多目标数组
- [ ] controller 状态机正常工作
- [ ] 发现垃圾自动切换
- [ ] 清理完成自动恢复
- [ ] 日志输出清晰可读

---

## 12. 预计时间

| 阶段 | 时间 |
|------|------|
| **第一阶段：完善 vservo** | |
| 消息类型 + vservo 修改 | 2小时 |
| 编译测试 | 30分钟 |
| **第二阶段：并入 controller** | |
| controller 修改 | 4小时 |
| 编译测试 | 1小时 |
| 调试优化 | 1小时 |
| **总计** | **约8.5小时** |

---

*文档结束*
