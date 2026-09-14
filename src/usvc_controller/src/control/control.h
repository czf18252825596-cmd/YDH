#ifndef CONTROL_H
#define CONTROL_H

#include <cstdint>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <Eigen/Core>
#include <ros/ros.h>
#include <std_msgs/Bool.h>


enum RUN_MODE_ENUM
{
    NORMAL = 0,
    KEEP  = 1,
    SAIL  = 2,
    BACK  = 3,
    TASK  = 4,
    USER  = 5,
};


enum TASK_TYPE_ENUM
{
    EMPTY           = 0, //空任务
    SAIL_T          = 1, //出航
    BACK_T          = 2, //返航
    PATH_TRACKING   = 3, //轨迹跟踪
    EDGE_FOLLOWING  = 6, //沿边模式
};

enum TASK_STATUS_ENUM
{
    STOP  = 0,          // 停止
    BEGIN = 1,          // 开始
    PAUSE = 2,          // 暂停
};

enum BERTH_STATUS_ENUM
{
    AWAITING_ARRIVAL = 0, // 空泊待命  解锁 无船
    PREPARING_DOCK   = 1, // 备锁待泊  上锁 无船
    DOCKED_SECURED   = 2, // 泊停锁定  上锁 有船
    PREPARING_DEPART = 3, // 脱锁待航  解锁 有船
};

typedef struct 
{
    double SAIL_SPEED; //速度调整，最大速度
    double SAIL_Y; //
    double SAIL_TOLERANCERADIUS[2]; //容差半径

    double DEFAULT_POS[2]; //预留吧  出航预设点
}Param_Sail_t;

typedef struct 
{
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

    double DEFAULT_POS[2]; //返航预设点
}Param_Back_t;

typedef struct 
{
    double LOW_BATTERY;   //低电量返航阈值(%)
    double CHARGE_START;  //自动充电阈值(%)
    double CHARGE_STOP;   //充电停止阈值(%)
}Param_Charge_t;

typedef struct 
{
    double MAXSPEED; //速度调整，最大速度
    double MINSPEED; // 设备自动避障模式下分配的最小速度限制
    double LOSDISTANCE; //LOS引导距离，1.5倍体长
    double OBSTACLE_INFLUENCE_DISTANCE;   // 障碍物影响范围（米）
    double SAFE_DISTANCE;                 // 安全区
    double REPULSIVE_GAIN;               // 斥力增益
    double ATTRACTIVE_GAIN;             // 引力增益
    double SIGMA;                      // σ参数
    double TOLERANCERADIUS; //容差半径
}Param_PathTracking_t;

typedef struct
{
    uint8_t mapping_en;       //建图开关 0=关 1=开
    uint8_t back_lowbatt_en;  //低电量返航开关 0=关 1=开
    uint8_t back_linklost_en; //断联返航开关 0=关 1=开
    uint8_t vision_en;        //目标识别+推流+跟踪开关 0=关 1=开
}EnableSwitches_t;

typedef struct 
{
    uint16_t runMode; //00 一般  01定速  02 定航   03定速定航  04定点  05出航  06返航  07任务  08自定义
        
    int Y;
    int H;
    float objSpeed;       
    float objHeading;

	uint8_t isAvoiding; 

    bool has_dock; //码头有无标志 
    uint8_t dock_ip[4];//码头信息ip 
    uint16_t dock_port; //码头端口 

    uint8_t task_state; //任务状态 00停止  01开始  02暂停  
    uint8_t task_tepy; //任务类型
    uint8_t ctrl_mode; //控制模式 00_自由控制 01_导航控制 
}RunCtrl_t;

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
		uint16_t runMode;  //
			
		int Y; //
		int H; //
		float objSpeed;   //   
		float objHeading; //
        double objPos[2]; //

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
}StateData_t;

typedef struct 
{
    double roll;     // 横滚角(度)
    double pitch;    // 俯仰角(度)
    double heading;  // 航向角(度) 0~360°
}NAVIEulerData_t;

typedef struct 
{
    double latitude;   // 纬度(度)
    double longitude;  // 经度(度)
}NAVIGnssData_t;

typedef struct 
{
    bool has_dock; //码头有无标志 
    uint8_t dock_ip[4];//码头信息ip 
    uint16_t dock_port; //码头端口 

    bool connect; //连接状态
    uint8_t dock_ID;

    double presetPos[2][2];//预设位置

    uint8_t berth; //系泊状态 
    uint8_t change; //充电状态
}Dock_Info_t;

//沿边结构体  
typedef struct
{
    float objSpeed;
    float objHeading;
    bool  isAvoiding;
    bool  reached;
}Edge_ctrl;

//单个任务：task_type +沿边水向+ 航点集 
typedef struct
{
    int task_type;
    bool water_on_left = false;  // 仅沿边模式有意义：true=水在船左侧
    std::map<int, std::vector<double>> task_val;
}CtlTask_t;

typedef struct 
{
    bool ctrl_finished; 

    uint8_t ctrl_mode; //控制模式 00_自由控制 01_导航控制 
    int Y;
    int H;
    float objSpeed;       
    float objHeading;

	bool isAvoiding; 

    uint8_t dock_ip[4];//码头信息ip 
    uint16_t dock_port; //码头端口 

}Ctrl_Result_t;


class Control 
{

public:
    // 构造函数
    Control();
    
    // 析构函数
    ~Control();


//任务文件
private:
    mutable std::mutex m_task_fileMutex;    
    std::vector<CtlTask_t> m_task_file; //任务文件
    //std::vector<...>  任务序列数组
        //<CtlTask_t> task_type:任务类型  water_on_left:沿边水向  task_val:航点集(序号→经纬度) 
public:
    bool getNewTaskFile(const std::string& file_path);
    void setTaskFile(std::vector<CtlTask_t> task_file);
    std::vector<CtlTask_t>  getTaskFile(void) const;

private:
    std::mutex m_Param_SailMutex; 
    Param_Sail_t m_Param_Sail;
public: 
    void setParam_Sail(Param_Sail_t param);
    Param_Sail_t getParam_Sail();

private:
    std::mutex m_Param_BackMutex; 
    Param_Back_t m_Param_Back;
public: 
    void setParam_Back(Param_Back_t param);
    Param_Back_t getParam_Back();

private:
    std::mutex m_Param_PathTrackingMutex; 
    Param_PathTracking_t m_Param_PathTracking;
public: 
    void setParam_PathTracking(Param_PathTracking_t param);
    Param_PathTracking_t getParam_PathTracking();

private:
    std::mutex m_Param_ChargeMutex;
    Param_Charge_t m_Param_Charge;
public:
    void setParam_Charge(Param_Charge_t param);
    Param_Charge_t getParam_Charge();

//运动模式
private:
    std::mutex m_RUN_MODEMutex; 
    RUN_MODE_ENUM m_RUN_MODE;
public:
    void changeRUN_MODE(RunCtrl_t ctrl);
    void setRUN_MODE(RUN_MODE_ENUM run);
    RUN_MODE_ENUM getRUN_MODE();

//任务状态
private:
    std::mutex m_TASK_STATUSMutex; 
    TASK_STATUS_ENUM m_TASK_STATUS;
public:
    void changeTASK_STATUS(RunCtrl_t ctrl);
    void setTASK_STATUS(TASK_STATUS_ENUM sta);
    TASK_STATUS_ENUM getTASK_STATUS();

//运动控制数据
private:
    std::mutex m_RunCtrlMutex; 
    RunCtrl_t m_RunCtrl;
public:
    void setRunCtrl(RunCtrl_t ctrl);
    RunCtrl_t getRunCtrl(void);

//使能开关数据
private:
    std::mutex m_EnableSwitchesMutex;
    EnableSwitches_t m_EnableSwitches;
public:
    void setEnableSwitches(EnableSwitches_t enables);
    EnableSwitches_t getEnableSwitches(void);
    void checkAutoReturn(const EnableSwitches_t& enables, bool upper_connected);

//状态数据
private:
    std::mutex m_StateDataMutex; 
    StateData_t m_StateData; 
public:
    void setStateData(StateData_t data);
    StateData_t getStateData(void);

//导航欧拉角数据
private:
    std::mutex m_NAVIEulerDataMutex; 
    NAVIEulerData_t m_NAVIEulerData;
public:
    void setNAVIEulerData(NAVIEulerData_t data);
    NAVIEulerData_t getNAVIEulerData(void);

//导航Gnss数据
private:
    std::mutex m_NAVIGnssDataMutex; 
    NAVIGnssData_t m_NAVIGnssData;
public:
    void setNAVIGnssData(NAVIGnssData_t data);
    NAVIGnssData_t getNAVIGnssData(void);

//雷达数据
private:
    std::mutex m_LidarDataMutex; 
    std::vector<Eigen::Vector3f> m_LidarData;
public:
    void setLidarData(std::vector<Eigen::Vector3f> data);
    std::vector<Eigen::Vector3f> getLidarData(void);
//沿边
private:
    std::mutex m_EdgeCtrlMutex;
    Edge_ctrl  m_Edge_ctrl;
public:
    void setEdgeCtrl(const Edge_ctrl& c);
    Edge_ctrl getEdgeCtrl(void);
 
private:
    Ctrl_Result_t normal_ctrl();   
    Ctrl_Result_t keep_ctrl();
    Ctrl_Result_t sail_ctrl(const Param_Sail_t& param, const NAVIEulerData_t& euler, const NAVIGnssData_t& gnss, const std::vector<Eigen::Vector3f>& lidar, const Dock_Info_t& dock_info);
    Ctrl_Result_t back_ctrl(const Param_Back_t& param, const NAVIEulerData_t& euler, const NAVIGnssData_t& gnss, const std::vector<Eigen::Vector3f>& lidar, const Dock_Info_t& dock_info);
    Ctrl_Result_t path_tracking_ctrl(const Param_PathTracking_t& param, const NAVIEulerData_t& euler, const NAVIGnssData_t& gnss, const std::vector<Eigen::Vector3f>& lidar, const std::map<int, std::vector<double>>& __O_PATH);
    Ctrl_Result_t user_ctrl();

public:
    RUN_MODE_ENUM executeNormal();
    RUN_MODE_ENUM executeKeep();
    RUN_MODE_ENUM executeSail();
    RUN_MODE_ENUM executeBack();
    RUN_MODE_ENUM executeTask();                                                                                                             
    RUN_MODE_ENUM executeUser();


private:
    int task_index = 0;
    int sail_step = 0; //出航步骤
    int back_step = 0; //返航步骤
    int path_id = 0;   //路径id

// ===== 沿边+视觉识别 状态机 =====
public:
    // EdgeSubState：PATH_TRACKING(轨迹跟踪)任务内部的子状态机，与"沿边任务 EDGE_FOLLOWING"无关。
    // 说明：此状态机位于 control.cpp 的 case PATH_TRACKING 之内，供"轨迹途中顺带清理垃圾"使用。
    // 枚举值 EDGE_FOLLOW 为历史命名，名不副实，贴切语义为 TRACK_NORMAL（正常轨迹航段）：
    //   EDGE_FOLLOW = 0  正常走轨迹（尚未偏离去清理垃圾）→ 可理解为 TRACK_NORMAL
    //   CLEANING        已发现垃圾，从轨迹偏离去清理
    //   CLEAN_DONE      已到垃圾位停车，等待 control_node 恢复轨迹(回到 EDGE_FOLLOW)
    enum EdgeSubState {
        EDGE_FOLLOW = 0,
        CLEANING,
        CLEAN_DONE,
    };

    // ===== 视觉跟踪(visual_track) 数据槽 =====
    // 由 control_node 的 visualCmdCallback(收 /visual_track/cmd) 写入，
    // 供 executeTask 的 PATH_TRACKING 状态机读取。
    EdgeSubState getEdgeSubState() const;
    void setEdgeSubState(EdgeSubState s);
    //触发信号
    void setVisualTrackHasTarget(bool has);
    bool getVisualTrackHasTarget() const;
    void setVisualTrackReached(bool reached);
    bool getVisualTrackReached() const;
    //关键输出
    void setVisualTrackSpeed(float s);
    float getVisualTrackSpeed() const;
    void setVisualTrackHeading(float h);
    float getVisualTrackHeading() const;

    //路径点控制
    void setPathId(int id);
    int getPathId() const;
    int getCurrentTaskType() const;
    void saveTrackPath(const std::map<int, std::vector<double>>& path);
    const std::map<int, std::vector<double>>& getTrackPath() const;
    void setPathTrackingHeading(float h);
    float getPathTrackingHeading() const;

    //恢复路径
    void saveEdgePath(const std::vector<Eigen::Vector2d>& path, bool water_left);
    const std::vector<Eigen::Vector2d>& getResumePath() const;
    bool getResumeWaterLeft() const;

private:
     mutable std::mutex m_VisualTrackMutex;
    // 有目标标志位(速度>0)：EDGE_FOLLOW → CLEANING 切换条件
    bool m_VisualTrackHasTarget = false;
    // 到达标志位：CLEANING → CLEAN_DONE 转移条件
    bool m_VisualTrackReached = false;
    // 船速度(0=停止) —— CLEANING 行驶用
    float m_VisualTrackSpeed   = 0.f;
    // 船航向(沿边基准 + 垃圾偏移) —— CLEANING 行驶用
    float m_VisualTrackHeading = 0.f;

    EdgeSubState m_EdgeSubState = EDGE_FOLLOW;

    std::vector<Eigen::Vector2d> m_ResumePath;
    bool m_ResumeWaterLeft = false;

    std::map<int, std::vector<double>> m_TrackPath;
    float m_PathTrackingHeading = 0.f;
    
};


#endif // CONTROL_H

