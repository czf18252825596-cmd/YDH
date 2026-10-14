// usvc_vservo 视觉伺服节点
// 订阅 /yolo/detections 的目标框 → 多类归并 + 归一化 → 选主目标 → 出期望航向/速度 (GuidanceCmd)
// 只产期望量，不写 RunCtrl；由 controller 在"沿边模式 + 识别使能"时采信。
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>

#include "usvc_planner/GuidanceCmd.h"
#include "usvc_planner/TunableParams.h"
#include "visual_track/VisualTrack.h"
#include "yolo_rtsp/DetectionArray.h"

namespace {
  // 一个候选目标（已归一化到 0~1）
  struct Target {
    uint8_t class_id = 0;
    float   cx = 0.f, cy = 0.f;   // 中心，归一化
    float   w = 0.f, h = 0.f;     // 宽高，归一化
    float   area = 0.f;           // w*h，归一化面积
  };

  const float PI = 3.14159265358979323846f;
} 

class VservoNode {
 public:
  VservoNode() {
    ros::NodeHandle pnh("~"); 
    // 先从本包 visual_track.yaml 读取默认参数（单包仿真直接生效；联调时由 latched 话题覆盖）
    pnh.param<float>("conf_thresh",         conf_thresh_,         0.5f);
    pnh.param<float>("approach_speed",      approach_speed_,      0.6f);
    pnh.param<float>("reached_area_thresh", reached_area_thresh_, 0.35f);
    pnh.param<float>("max_heading_offset",  max_heading_offset_,  15.0f);
    pnh.param<float>("heading_alpha",       heading_alpha_,       0.3f);
    pnh.param<float>("bottom_thresh",       bottom_thresh_,       0.92f);
    pnh.param<float>("slow_area",           slow_area_,           0.03f);
    std::vector<int> ids;
    pnh.getParam("target_class_ids", ids);                    //清理名单
    for (int i : ids) target_class_ids_.push_back(i);
    pnh.getParam("image_inverted", inverted_);                //相机安装正反 true=倒装需上下翻转

    // 话题
    sub_det_    = nh_.subscribe("/yolo/detections", 1, &VservoNode::onDetection, this);
    sub_ctrl_   = nh_.subscribe("/visual_track/ctrl", 1, &VservoNode::onCtrl, this);
    sub_refhdg_ = nh_.subscribe("/visual_track/ref_heading", 1, &VservoNode::onRefHeading, this);
    sub_tunable_ = nh_.subscribe<usvc_planner::TunableParams>("/planner_node/tunable_update", 1, &VservoNode::onTunableUpdate, this);
    pub_cmd_    = nh_.advertise<usvc_planner::GuidanceCmd>("/visual_track/cmd", 1);
    pub_has_target_ = nh_.advertise<std_msgs::Bool>("/visual_track/has_target", 1);

    timer_ = nh_.createTimer(ros::Duration(0.05), &VservoNode::onPublish, this);  // 20 Hz
    ROS_INFO("[vservo] up. classes={%s} conf=%.2f ",
             joinClassIds().c_str(), conf_thresh_);
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_det_, sub_ctrl_, sub_refhdg_, sub_tunable_;
  ros::Publisher  pub_cmd_;
  ros::Publisher  pub_has_target_;
  ros::Timer      timer_;

  std::mutex mtx_;
  bool           enabled_ = false;      // 由 ctrl 控制
  yolo_rtsp::DetectionArray last_det_;  // 最近一帧
  bool           has_det_ = false;

    float conf_thresh_         = 0.5f;
    float approach_speed_      = 0.6f;
    float reached_area_thresh_ = 0.35f;
    float bottom_thresh_       = 0.92f;
    float slow_area_           = 0.03f;
    float max_heading_offset_  = 15.0f;
    float heading_alpha_       = 0.3f;
    float filtered_theta_   = 0.f;
    float filtered_speed_   = 0.f;
    bool  inverted_         = false;   // 相机安装正反 true=倒装需上下翻转
    float ref_heading_deg_  = 0.f;
    std::vector<uint8_t> target_class_ids_;

  std::string joinClassIds() {
    std::string s;
    for (auto c : target_class_ids_) s += std::to_string(c) + ",";
    if (!s.empty()) s.pop_back();//去掉末尾逗号
    return s.empty() ? "(none)" : s;
  }

  void onDetection(const yolo_rtsp::DetectionArray& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_det_ = msg;
    has_det_  = true;
  }

  void onCtrl(const visual_track::VisualTrack& msg) {
    // run_mode=1 跟随 → 使能；其余(含 run_mode=0 停车) → 关闭
    enabled_  = (msg.run_mode == 1 );
    ROS_INFO("[visual_track] ctrl(run=%d type=%d) -> %s",
             msg.run_mode, msg.task_type,
             enabled_ ? "ENABLED" : "DISABLED");
  }

  void onRefHeading(const std_msgs::Float32& msg) {
    ref_heading_deg_ = msg.data;
  }

  // 上位机可调参数更新回调（visual_track 组）
  void onTunableUpdate(const usvc_planner::TunableParams::ConstPtr& msg) {
    if (msg) {
      // ========== visual_track 参数 ==========
      conf_thresh_        = static_cast<float>(msg->conf_thresh);
      max_heading_offset_ = static_cast<float>(msg->max_heading_offset);
      approach_speed_     = static_cast<float>(msg->approach_speed);
      heading_alpha_      = static_cast<float>(msg->heading_alpha);
      reached_area_thresh_= static_cast<float>(msg->reached_area_thresh);
      bottom_thresh_      = static_cast<float>(msg->bottom_thresh);
      slow_area_          = static_cast<float>(msg->slow_area);
      ROS_INFO("[visual_track] 上位机调参更新: conf_thresh=%.2f max_offset=%.1f approach_speed=%.2f heading_alpha=%.2f reached_area=%.2f bottom_thresh=%.2f slow_area=%.2f",
               conf_thresh_, max_heading_offset_, approach_speed_, heading_alpha_, reached_area_thresh_, bottom_thresh_, slow_area_);
    }
  }

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
          == target_class_ids_.end()) continue;              // ① 白名单
      if (det.confidence < conf_thresh_) continue;           // ② 置信度
      Target t;
      t.class_id = det.class_id;  
      //目标识别框的参数：x,y(框左上角位置)   
      //               width height 框宽度，高度  
      //               W，H 分辨率
      // 识别框的几何中心（cx,cy）                         
      float cx = det.x + det.width  * 0.5f;
      float cy = det.y + det.height * 0.5f;
      t.cx = cx / W;                                         // ③ 归一化
      t.cy = cy / H;
      if (inverted_) t.cy = 1.f - t.cy;                      // ③.5 相机倒装：上下翻转
      t.w  = static_cast<float>(det.width)  / W;
      t.h  = static_cast<float>(det.height) / H;
      t.area = t.w * t.h;
      out.push_back(t);
    }
    return out;
  }
        //角度[0,360]
  static float normDeg(float deg) {
    deg = std::fmod(deg, 360.f);
    return deg < 0.f ? deg + 360.f : deg;
  }

void onPublish(const ros::TimerEvent&) {
  auto cands = collectTargets();

  // 检测框判据（与使能/控制无关）：白名单+置信度过滤后有待清理框即 true
  std_msgs::Bool ht;
  ht.data = !cands.empty();
  pub_has_target_.publish(ht);

  usvc_planner::GuidanceCmd cmd;
  cmd.is_avoiding = false;
  cmd.reached = false;

    if (!enabled_ || cands.empty()) {
      filtered_theta_ = 0.f;
      filtered_speed_ = 0.f;                        // 复位，与输出 ref+0 一致
      cmd.obj_speed   = 0.f;
      cmd.obj_heading = normDeg(ref_heading_deg_);
      pub_cmd_.publish(cmd);
      return;
    }

    const Target* main = &cands[0];
    for (const auto& c : cands) if (c.area > main->area) main = &c;

    // 偏移角：中心偏移 × 最大偏移角，限幅
    float theta = (main->cx - 0.5f) * 2.0f * max_heading_offset_;
    theta = std::max(-max_heading_offset_, std::min(max_heading_offset_, theta));

    filtered_theta_ = heading_alpha_ * theta + (1.f - heading_alpha_) * filtered_theta_;

    cmd.obj_heading = normDeg(ref_heading_deg_ + filtered_theta_);
    cmd.is_avoiding = false;  
    // 速度：按归一化面积 S 代理距离，越大越近越慢
    float S = main->area;
    bool reached = (S >= reached_area_thresh_)
                || (main->cy + main->h * 0.5f >= bottom_thresh_);  // 进入收集盲区
    if (reached) {
      cmd.obj_speed = 0.f;      // 到位停车
      cmd.reached  = true;
    } else {
      float k = 0.f;
      if (S > slow_area_)       // 进入减速区线性减速
        k = std::max(0.f, std::min(1.f, (S - slow_area_) / (reached_area_thresh_ - slow_area_)));
      cmd.obj_speed = approach_speed_ * (1.f - k);
    }

    pub_cmd_.publish(cmd);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_track_node");
  VservoNode node;
  ros::spin();
  return 0;
}