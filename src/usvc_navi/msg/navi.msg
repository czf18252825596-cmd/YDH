std_msgs/Header header

# XYZ对应右前上
uint8 ins_sta	# 组合导航状态 0_初始 1_正常  2_异常
uint32 timeTick # 时间戳
float64 gyro_x # 陀螺X(deg/s)
float64 gyro_y # 陀螺Y(deg/s)
float64 gyro_z # 陀螺Z(deg/s)
float64 roll # 横滚角(度)
float64 pitch # 俯仰角(度)
float64 heading # 航向角(度) 0~360°
float64 acc_x # 加速度X(g)
float64 acc_y # 加速度Y(g)
float64 acc_z # 加速度Z(g)
float64 vel_x # 速度x(m/s)
float64 vel_y # 速度y(m/s)
float64 vel_z # 速度z(m/s)
float64 latitude # 纬度(度) 
float64 longitude # 经度(度)
float64 altitude # 高度(米)
float64 vel_n # 北向速度(m/s)
float64 vel_e # 东向速度(m/s)
float64 vel_d # 地向速度(m/s)
uint8 satellite_count # 收星数
uint8 pos_sta # 定位状态 0_初始 1_有定位 2_无定位
uint8 heading_sta # 定向状态 0_初始 1_有定向 2_无定向
float64 position_std_lat # 纬度精度(m)
float64 position_std_lon # 经度精度(m)
