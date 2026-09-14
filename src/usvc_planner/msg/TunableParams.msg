# 上位机可动态下发的调参消息（单 topic 广播，planner 与 visual_track 各取所需字段）
# 冷启动默认值仍由各节点 nh.param 从各自 config 读取；本消息用于运行时覆盖。

# ========== usvc_planner 参数 ==========

# --- Planner参数 ---
float64 target_speed      # 巡航目标速度 (m/s)
float64 los_distance      # LOS预瞄距离
float64 waypoint_tolerance # 航点到达容差
float64 min_speed_turn    # 急弯最低速度
float64 adaptive_los_scale # ALOS自适应系数

# --- 掩码参数 ---
float64 safe_margin       # 水侧掩码阈值（沿边安全距离）
float64 boom_x_max        # 喇叭口最前端离雷达距离

# ---  visual_track 参数 ---
float64 conf_thresh         # 目标置信度阈值
float64 max_heading_offset  # 目标在画面边缘时的最大偏转角(度)
float64 approach_speed      # 接近目标最大速度 (m/s)
float64 heading_alpha       # 航向低通滤波系数(0~1)
float64 reached_area_thresh # 到位判据：归一化面积阈值
float64 bottom_thresh       # 到达盲区阈值
float64 slow_area           # 减速区阈值