# 状态回报（由 guidance_node 发布，桥接节点转给上位机）

uint16  run_mode        # 0=沿边  1=自主导航
uint8   task_state      # 0=停止  1=运行中
uint8   nav_state       # 0=IDLE 1=TRACKING 2=ALIGNING 3=ARRIVED
uint16  waypoint_idx    # 当前航点索引
uint16  waypoint_num    # 总航点数
uint8   boundary_idx    # 当前沿边边界号 (0=未激活)
float64 x               # 船位 x（实船：经度 lon）
float64 y               # 船位 y（实船：纬度 lat）
float64 yaw             # 船头航向 (rad)
