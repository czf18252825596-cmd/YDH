#ifndef __ULINKDATASTRUCT_H
#define __ULINKDATASTRUCT_H

#ifdef __cplusplus
extern "C" {
#endif
	
/*******************************************************************************************************/
/*                                                头文件                                               */
/*******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


/*******************************************************************************************************/
/*                                                 枚举                                                */
/*******************************************************************************************************/
enum PACKET_ID_
{
	PACKET_ID_Print  = 0x00, //
	PACKET_ID_Reset  = 0x01, //复位指令, 请求
	PACKET_ID_Info   = 0x02, //信息指令，请求
	PACKET_ID_Vesion = 0x03, //版本指令，请求
	PACKET_ID_Network = 0x04, //网络指令，请求，更改，保存
	
	PACKET_ID_DEVICE_ID = 0x10, //设备ID，请求，更改，保存
	PACKET_ID_Data1 = 0x11, //DataA通道，请求，更改，保存
	PACKET_ID_Data2 = 0x12, //DataB通道，请求，更改，保存
	PACKET_ID_Data3 = 0x13, //DataC通道，请求，更改，保存
	
	PACKET_ID_NAVI  = 0x30, //导航数据
};


/*******************************************************************************************************/
/*                                               类型定义                                               */
/*******************************************************************************************************/
#pragma pack(1)
typedef struct
{
    char identifiter[32];  //识别码
	char sn[32];           //设备SN
}DATA_Info_t;

typedef struct
{
    char bootloader[32];  //bootloader版本
    char app[32];         //app版本
    char hardware[32];    //硬件板号
} DATA_Version_t;

typedef struct
{
    uint8_t flag; //操作标志 01H:保存 02H：写入 03H读取 10H:保存失败 11H：保存成功 21H：写入成功 31H：读取成功
	
	uint8_t ip[4];
	uint8_t sn[4];
	uint8_t gw[4];
	uint8_t mac[6];
} DATA_Network_t;

typedef struct
{
    uint8_t flag; //操作标志位
	
	uint8_t DEVICE_ID;  //设备ID 0~0xff  0xff为广播
} DATA_DEVICE_ID_t;

typedef struct
{
    uint8_t flag;
	
	bool enable; //通道使能  true:使能 false：失能
	uint16_t 	localPort; //本地端口
	uint8_t		desIP[4]; //目标IP
	uint16_t 	desPort;  //目标端口
	uint8_t		protocol; //
} DATA_Data1_t;

typedef struct
{
    uint8_t flag;
	
	bool enable;
	uint16_t 	localPort;
	uint8_t		desIP[4]; //ip
	uint16_t 	desPort; 
	uint8_t		protocol; //
} DATA_Data2_t;

typedef struct
{
    uint8_t flag;
	
	bool enable;
	uint16_t 	localPort;
	uint8_t		desIP[4]; //ip
	uint16_t 	desPort; 
	uint8_t		protocol; //
} DATA_Data3_t;


typedef struct
{
	uint8_t ins_sta; //组合导航状态 0_初始 1_正常  2_异常
	
	uint32_t timeTick; //时间戳
	
	double gyro_roll;              // 陀螺X(deg/s)
    double gyro_pitch;              // 陀螺Y(deg/s)
    double gyro_heading;              // 陀螺Z(deg/s)
	
	double roll;                // 横滚角(度)
    double pitch;               // 俯仰角(度)
    double heading;             // 航向角(度) 0~360°
    
    double acc_x;               // 加速度X(g)
    double acc_y;               // 加速度Y(g)
    double acc_z;               // 加速度Z(g)
	
	double vel_x;               // 速度x(m/s)
    double vel_y;               // 速度y(m/s)
    double vel_z;               // 速度z(m/s)
	
	double latitude;            // 纬度(度)
    double longitude;           // 经度(度)
    double altitude;            // 高度(米)
	
	double vel_n;               // 北向速度(m/s)
    double vel_e;               // 东向速度(m/s)
    double vel_d;               // 地向速度(m/s)
	
	uint8_t satellite_count;    // 收星数
	uint8_t pos_sta;			// 定位状态 0_初始 1_有定位 2_无定位
	uint8_t heading_sta;		// 定向状态 0_初始 1_有定向 2_无定向
	
	double position_std_lat;    // 纬度精度(m)
    double position_std_lon;    // 经度精度(m)
} DATA_NAVI_t;

#pragma pack()

#ifdef __cplusplus
}
#endif

#endif
