# 船状态数据
# 1. 传感器数据
float32 humidity           # 湿度
float32 temperature        # 温度
uint8[2] leak_detection  # 漏水检测 [0]:左 [1]:右

# 2. 电源状态
float32 voltage            # 电压
float32 current            # 电流

# 3. 电源管理
uint8 Thruster          # PMS状态
uint8 LEDScre
uint8 unused2
uint8 unused3

# 4. 电池状态
float32 battery_q          # 电量
float32 battery_v          # 电压
float32 battery_a          # 电流
float32 battery_t          # 时间

# 5. 运行状态
uint16 runMode            # 运行模式
int32 y                   # Y
int32 h                   # H
float32 objSpeed       # 目标速度
float32 objHeading     # 目标航向
float64[2] objPos

uint8 task_tepy #任务类型
uint8 task_sta  #任务状态
uint8 mapping_state#建图开关
uint8 vision_state  # 识别+推流+跟踪 0=关 1=识别推流中 2=跟踪中
    
# 6. 导航数据
uint8 ins_status           # 组合导航状态 (0:初始, 1:正常, 2:异常)
uint32 timestamp           # 时间戳

# 陀螺仪
float64 gyro_roll          # 陀螺X(deg/s)
float64 gyro_pitch         # 陀螺Y(deg/s)
float64 gyro_heading       # 陀螺Z(deg/s)

# 姿态角
float64 roll               # 横滚角(度)
float64 pitch              # 俯仰角(度)
float64 heading            # 航向角(度) 0~360°

# 加速度
float64 acc_x              # 加速度X(g)
float64 acc_y              # 加速度Y(g)
float64 acc_z              # 加速度Z(g)

# 速度
float64 vel_x              # 速度x(m/s)
float64 vel_y              # 速度y(m/s)
float64 vel_z              # 速度z(m/s)

# 位置
float64 latitude           # 纬度(度)
float64 longitude          # 经度(度)
float64 altitude           # 高度(米)

# NED速度
float64 vel_n              # 北向速度(m/s)
float64 vel_e              # 东向速度(m/s)
float64 vel_d              # 地向速度(m/s)

# GPS状态
uint8 satellite_count      # 收星数
uint8 pos_status           # 定位状态 (0:初始, 1:有定位, 2:无定位)
uint8 heading_status       # 定向状态 (0:初始, 1:有定向, 2:无定向)

# 定位精度
float64 pos_std_lat        # 纬度精度(m)
float64 pos_std_lon        # 经度精度(m)

# 7. 推进器状态
# 左推进器
int32 left_thruster_val
uint8 left_thruster_status
float32 left_duty_cycle     # 占空比
int32 left_rpm              # 转速
float32 left_mos_temp       # MOS温度
float32 left_total_current  # 总电流
float32 left_input_current  # 输入电流

# 右推进器
int32 right_thruster_val
uint8 right_thruster_status
float32 right_duty_cycle    # 占空比
int32 right_rpm             # 转速
float32 right_mos_temp      # MOS温度
float32 right_total_current # 总电流
float32 right_input_current # 输入电流

# 8. 灯状态
uint8 LED         # 灯状态
uint8 CautionLED

# 9. 码头信息
bool connect                    #连接状态
uint8 dock_ID

float64[2] preset_positions0  # 预设位置
float64[2] preset_positions1  # 预设位置

uint8 berth_status               # 系泊状态
uint8 charge_status              # 充电状态