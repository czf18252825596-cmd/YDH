#ifndef __MTLINKDATASTRUCT_H
#define __MTLINKDATASTRUCT_H


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


enum MTLINK_OBJECT
{
    //
    OBJ_SHIP_BoardReset     = 0x0010, //板卡复位
    OBJ_SHIP_BoardInfo      = 0x0011, //板卡信息
    OBJ_SHIP_BoardVersion   = 0x0012, //板卡版本
    OBJ_SHIP_BoardNetwork   = 0x0013, //板卡网络

    OBJ_SHIP_SetID          = 0x1000, //设置标识ID

    OBJ_SHIP_BoardPort      = 0x1010, //板卡端口
    OBJ_SHIP_YawPIDParam    = 0x1011, //航向PID参数
    OBJ_SHIP_SpeedPIDParam  = 0x1012, //速度PID参数
    OBJ_SHIP_DisPIDParam    = 0x1013, //距离PID参数

    OBJ_SHIP_ResetSN        = 0x1030, //重置工控机SN记录
    OBJ_SHIP_LidarParam     = 0x1031, //雷达参数
    OBJ_SHIP_SailParam      = 0x1032, //出航控制参数
    OBJ_SHIP_BackParam      = 0x1033, //返航控制参数
    OBJ_SHIP_PathTrackParam = 0x1034, //轨迹跟踪控制参数
    OBJ_SHIP_EnableSwitches = 0x1035, //使能开关数据
    OBJ_SHIP_ChargeParam    = 0x1036, //充电阈值参数
    OBJ_SHIP_TunableParam   = 0x1037, //上位机可调参数(planner LOS + visual_track)

	OBJ_SHIP_RunCtrl = 0x1101, //运动控制数据
    OBJ_SHIP_PMSCtrl = 0x1102, //PMS控制数据
    OBJ_SHIP_MotorCtrl = 0x1103, //推进器控制数据
    OBJ_SHIP_LightCtrl = 0x1104, //灯光控制数据
    OBJ_SHIP_BerthCtrl = 0x1105, //泊船控制数据
    OBJ_SHIP_ChangeCtrl = 0x1106, //充电控制数据
    

    OBJ_SHIP_StateData = 0x1201, //状态反馈数据
    OBJ_SHIP_LidarData = 0x1202, //雷达负反馈数据
    OBJ_SHIP_ACHData   = 0x1203, //里程反馈数据
};


#pragma pack(1)
//OBJ_SHIP_BoardReset

//OBJ_SHIP_BoardInfo
typedef struct
{
    char identifiter[32];  //识别码
	char sn[32];           //设备SN
}MDATA_SHIP_BoardInfo_t;

//OBJ_SHIP_BoardVersion
typedef struct
{
    char bootloader[32];  //bootloader版本
    char app[32];         //app版本
    char hardware[32];    //硬件板号
}MDATA_BoardVersion_t;

//OBJ_SHIP_BoardNetwork
typedef struct
{
    uint8_t flag; //操作标志 01H:保存 02H：写入 03H读取 10H:保存失败 11H：保存成功 21H：写入成功 31H：读取成功
	
	uint8_t ip[4];
	uint8_t sn[4];
	uint8_t gw[4];
	uint8_t mac[6];
}MDATA_SHIP_BoardNetwork_t;

//OBJ_SHIP_SetID
typedef struct
{
	uint8_t flag;  //操作标志 01H:保存 02H：写入 03H读取 10H:保存失败 11H：保存成功 21H：写入成功 31H：读取成功
    uint8_t ID;
}MDATA_SHIP_ID_t; 

//OBJ_SHIP_BoardPort
typedef struct
{
    uint8_t flag; //操作标志 01H:保存 02H：写入 03H读取 10H:保存失败 11H：保存成功 21H：写入成功 31H：读取成功
	uint16_t port;
}MDATA_SHIP_BoardPort_t;

//OBJ_SHIP_YawPIDParam && OBJ_SHIP_SpeedPIDParam && OBJ_SHIP_DisPIDParam
typedef struct
{
    uint8_t flag; //操作标志 01H:保存 02H：写入 03H读取 10H:保存失败 11H：保存成功 21H：写入成功 31H：读取成功
	
	bool  OuterLoopEN;  //true:使用外环 false:不使用
    float OutZVal;      //外环死区
    float Pout;
    float Iout;
    float Dout;
    float InZVal;  //内环死区
    float Pin;
    float Iin;
    float Din;
}MDATA_SHIP_PIDParam_t;

//OBJ_SHIP_ResetSN

//OBJ_SHIP_LidarParam
typedef struct
{
	uint8_t flag;

    double X_DISTANCE;   
    double Y_DISTANCE;  
    double Z_DISTANCE;  
    double ClusterTolerance;    // 聚类半径
    int MinClusterSize; // 最小聚类大小，小于则丢弃(噪点)
    int MaxClusterSize; // 最大聚类大小，超过则丢弃
}MDATA_SHIP_LidarParam_t; 

//OBJ_SHIP_SailParam
typedef struct
{
	uint8_t flag;

    double SPEED; //速度调整，最大速度
    double Y; //
    double TOLERANCERADIUS[2]; //容差半径

    double POS[2]; //预留吧 出航预设点
}MDATA_SHIP_SailParam_t; 

//OBJ_SHIP_BackParam
typedef struct
{
	uint8_t flag;

    double MAXSPEED; //速度调整，最大速度
    double MINSPEED; // 设备自动避障模式下分配的最小速度限制
    double LOSDISTANCE; //LOS引导距离，1.5倍体长
    double OBSTACLE_INFLUENCE_DISTANCE;   // 障碍物影响范围（米）
    double SAFE_DISTANCE;                 // 安全区
    double REPULSIVE_GAIN;               // 斥力增益
    double ATTRACTIVE_GAIN;             // 引力增益
    double SIGMA;                      // σ参数
    double TOLERANCERADIUS; //容差半径
    double Y; //
    double POS[2]; //返航预设点
}MDATA_SHIP_BackParam_t; 

//OBJ_SHIP_PathTrackParam
typedef struct
{
	uint8_t flag;

    double MAXSPEED; //速度调整，最大速度
    double MINSPEED; // 设备自动避障模式下分配的最小速度限制
    double LOSDISTANCE; //LOS引导距离，1.5倍体长
    double OBSTACLE_INFLUENCE_DISTANCE;   // 障碍物影响范围（米）
    double SAFE_DISTANCE;                 // 安全区
    double REPULSIVE_GAIN;               // 斥力增益
    double ATTRACTIVE_GAIN ;             // 引力增益
    double SIGMA;                      // σ参数
    double TOLERANCERADIUS; //容差半径
}MDATA_SHIP_PathTrackParam_t; 

//OBJ_SHIP_TunableParam
typedef struct
{
    uint8_t flag;   //01H:保存 03H:读取 11H:保存成功 31H:读取成功

    // ========== usvc_planner 参数 ==========
    double TARGET_SPEED;        // 巡航目标速度(m/s)
    double LOS_DISTANCE;        // LOS预瞄距离
    double WAYPOINT_TOLERANCE;  // 航点到达容差
    double MIN_SPEED_TURN;      // 急弯最低速度
    double ADAPTIVE_LOS_SCALE;  // ALOS自适应系数
    
    // --- 掩码参数 ---
    double SAFE_MARGIN;         // 水侧掩码阈值（沿边安全距离）
    double BOOM_X_MAX;          // 喇叭口最前端离雷达距离
    
    // ========== visual_track 参数 ==========
    double CONF_THRESH;         // 目标置信度阈值
    double MAX_HEADING_OFFSET;  // 最大航向偏移(度)
    double APPROACH_SPEED;      // 接近目标最大速度(m/s)
    double HEADING_ALPHA;       // 航向低通滤波系数
    double REACHED_AREA_THRESH; // 到位判据归一化面积阈值
    double BOTTOM_THRESH;       // 到达盲区阈值
    double SLOW_AREA;           // 减速区阈值
}MDATA_SHIP_TunableParam_t;

//OBJ_SHIP_RunCtrl
typedef struct
{
	uint16_t runMode; //00 一般  01定速  02 定航   03定速定航  04定点  05出航  06返航  07任务  08自定义  
    
    int Y; //Y轴摇杆值 
    int H; //H轴摇杆值 
    float objSpeed;   //目标速度 
    float objHeading;  //目标航向 

	uint8_t isAvoiding;  //避障状态 

    bool has_dock; //有无码头标志 
    uint8_t dock_ip[4];//码头信息ip 
    uint16_t dock_port; //码头端口 
    uint8_t task_state; //任务状态 00停止  01开始  02暂停 
     
    uint8_t task_teyp; //任务类型
    uint8_t ctrl_mode; //控制模式 00_自由控制 01_导航控制
}MDATA_SHIP_RunCtrl_t; //船运动控制;

//OBJ_SHIP_EnableSwitches
typedef struct
{   
    uint8_t flag;
    uint8_t mapping_en;       //建图开关 0  关闭  1开启
    uint8_t back_lowbatt_en;  //低电量返航开关 0 关 1 开
    uint8_t back_linklost_en; //断联返航开关 0 关 1 开
    uint8_t vision_en;        //目标识别+推流+跟踪开关 0 关 1 开
}MDATA_SHIP_EnableSwitches_t; //使能开关

//OBJ_SHIP_ChargeParam
typedef struct
{
    uint8_t flag;

    double LOW_BATTERY;  //低电量返航阈值(%)
    double CHARGE_START; //自动充电阈值(%)
    double CHARGE_STOP;  //充电停止阈值(%)
}MDATA_SHIP_ChargeParam_t; //充电阈值参数

//OBJ_SHIP_PMSCtrl
typedef struct
{   
    uint8_t flag;

    uint8_t Thruster;  //推进器上电
    uint8_t LEDScre;   //LED屏电源控制
    uint8_t unused2;   //
    uint8_t unused3;   //
}MDATA_SHIP_PMSCtrl_t; //船电源控制

//OBJ_SHIP_MotorCtrl
typedef struct
{
    uint8_t left_flag;  //  00_自动/无   01_单控
    int32_t L_Motor;

    uint8_t right_flag;  //  00_自动/无   01_单控
    int32_t R_Motor;
}MDATA_SHIP_MotorCtrl_t; //船推进器控制

//OBJ_SHIP_LightCtrl
typedef struct
{
    uint8_t LED;
    uint8_t CautionLED;
}MDATA_SHIP_LightCtrl_t; //船灯控制

//OBJ_SHIP_BerthCtrl
typedef struct
{
    uint8_t dock_ip[4];//码头信息ip 
    uint16_t dock_port; //码头端口 

    uint8_t berth; //系泊  00_解开  01_系泊 
}MDATA_SHIP_BerthCtrl_t; //船泊船控制

//OBJ_SHIP_ChangeCtrl
typedef struct
{
    uint8_t dock_ip[4];//码头信息ip 
    uint16_t dock_port; //码头端口 

    uint8_t change; //充电  00_停止充电  01_充电 
}MDATA_SHIP_ChangeCtrl_t; //船充电控制

//OBJ_SHIP_StateData
typedef struct
{
	struct{
        float humi;//湿度
        float temp;//温度
        uint8_t leak[2];//漏水 [0]:左 [1]:右
    } sensor; //传感器

	struct
    {
        float volage;   //电压
        float current;  //电流
    } power;  //电源

	struct
    {
        uint8_t Thruster;  //推进器上电
        uint8_t LEDScre;   //LED屏电源控制
        uint8_t unused2;   //保留
        uint8_t unused3;   //保留
    } pms;

	struct
    {
        float Q;  //电量
        float V;  //电压
        float A;  //电流
        float T;  //时间
    } battery;  //电池

	struct
    {
		uint16_t runMode;  //  下 中 上
			
		int Y; //下中上
		int H; //下中上
		float objSpeed;   //下中上    
		float objHeading; //下中上
        double objPos[2]; //下中上

        uint8_t task_tepy; //任务类型
        uint8_t task_sta;//任务状态

        uint8_t vision_state;  //识别+推流+跟踪 0=关 1=识别推流中 2=跟踪中
        uint8_t mapping_state; //建图 0=未建图 1=建图中
	} run;

    struct
    {
        uint8_t ins_sta;  //组合导航状态 0_初始 1_正常  2_异常

        uint32_t timeTick;  //时间戳

        double gyro_roll;     // 陀螺X(deg/s)
        double gyro_pitch;    // 陀螺Y(deg/s)
        double gyro_heading;  // 陀螺Z(deg/s)

        double roll;     // 横滚角(度)
        double pitch;    // 俯仰角(度)
        double heading;  // 航向角(度) 0~360°

        double acc_x;  // 加速度X(g)
        double acc_y;  // 加速度Y(g)
        double acc_z;  // 加速度Z(g)

        double vel_x;  // 速度x(m/s)
        double vel_y;  // 速度y(m/s)
        double vel_z;  // 速度z(m/s)

        double latitude;   // 纬度(度)
        double longitude;  // 经度(度)
        double altitude;   // 高度(米)

        double vel_n;  // 北向速度(m/s)
        double vel_e;  // 东向速度(m/s)
        double vel_d;  // 地向速度(m/s)

        uint8_t satellite_count;  // 收星数
        uint8_t pos_sta;          // 定位状态 0_初始 1_有定位 2_无定位
        uint8_t heading_sta;      // 定向状态 0_初始 1_有定向 2_无定向

        double position_std_lat;  // 纬度精度(m)
        double position_std_lon;  // 经度精度(m)
    } navi;   //导航

    struct
    {
        int32_t left_Val;
        uint8_t left_sta;
        float   left_duty_scaled;    //占空比
        int32_t left_rpm;            //转速
        float   left_MOST;           //MOS温度
        float   left_total_current;  //总电流
        float   left_current_in;     //输入电流
        //TODO异常状态

        int32_t right_Val;
        uint8_t right_sta;
        float   right_duty_scaled;    //占空比
        int32_t right_rpm;            //转速
        float   right_MOST;           //MOS温度
        float   right_total_current;  //总电流
        float   right_current_in;     //输入电流
        //TODO异常状态
    } thruster;  //推进器

	struct
    {
        uint8_t LED;
        uint8_t CautionLED;
    } light;  //LED灯控制

    struct
    {
        bool connect; //连接状态
        uint8_t dock_ID;

        double presetPos[2][2];//预设位置

		uint8_t berth; //系泊状态 
		uint8_t change; //充电状态
    } dock;//码头
}MDATA_SHIP_StateData_t;  //船状态数据

//OBJ_SHIP_LidarData
typedef struct
{
    uint8_t nums; //数量

    float x[100]; //X轴 左侧位置
    float y[100]; //Y轴 前侧位置
}MDATA_SHIP_LidarData_t; //雷达数据

typedef struct
{
    double total_distance;//总里程 km
    double total_operating_hours;//总作业时长 h
}MDATA_SHIP_ACHData_t; //里程数据


#pragma pack()

#endif
