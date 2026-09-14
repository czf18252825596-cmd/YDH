# 船运动控制

# 运行模式位域
uint16 run_mode

# 控制参数
int32 Y              # 位置Y
int32 H              # 位置H
float32 obj_speed    # 目标速度
float32 obj_heading  # 目标航向

# 控制标志
uint8 is_avoiding    # 避障状态

bool has_dock
uint8[4] dock_ip
uint16 dock_port

uint8 task_state
uint8 task_tepy
uint8 ctrl_mode