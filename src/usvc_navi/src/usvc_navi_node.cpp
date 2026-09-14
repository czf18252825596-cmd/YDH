#include <ros/ros.h>
#include <std_msgs/String.h>
#include <usvc_navi/navi.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include "UDP/udp.h"
extern "C" {
#include "ULink/ULink.h"
}

// 包装函数，用于将成员函数转为带上下文的 C 函数
bool UDP_SendBuffer(void* context, uint8_t* buf, uint16_t len) {
    Udp* mUdp = static_cast<Udp*>(context);
    return mUdp->udp_send(buf, len);
}

bool UDP_RecvBuffer(void* context, uint8_t* buf, uint16_t* len) {
    Udp* mUdp = static_cast<Udp*>(context);
    return mUdp->udp_receive(buf, len);
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "usvc_navi_node");
    
    ros::NodeHandle nh;

    std::string USVC_NAVI_TOPIC; 
    int USVC_NAVI_SPORT;

    // 从参数服务器获取配置
    nh.param<std::string>("USVC_NAVI_TOPIC", USVC_NAVI_TOPIC, "/usvc_navi/navi");
    nh.param<int>("USVC_NAVI_SPORT", USVC_NAVI_SPORT, 6001);

    //设置话题发布器
    ros::Publisher usvc_navi_pub = nh.advertise<usvc_navi::navi>(USVC_NAVI_TOPIC, 10);

    //定义ULink
    ULink_t mULink = {};
    //实例UDP
    Udp mUdp(USVC_NAVI_SPORT);

    //注册ULink收发函数
    ULink_RegSendFunc(&mULink, UDP_SendBuffer, &mUdp);
    ULink_RegReciveFunc(&mULink, UDP_RecvBuffer, &mUdp);

    ros::Rate loop_rate(100); 
    while (ros::ok()) 
    {
        uint8_t DEVICE_ID;
        uint8_t PACKET_ID; 
        uint8_t MS;
        uint8_t *pDATA; 
        uint16_t LENGTH;

        //接收并解析导航
        if(ULink_Receive(&mULink, &DEVICE_ID, &MS, &PACKET_ID,  &pDATA, &LENGTH))
		{
            switch(PACKET_ID)
            { 
                case PACKET_ID_NAVI:
                {
                    usvc_navi::navi navi_msg;

                    // 填充消息头
                    navi_msg.header.stamp = ros::Time::now();
                    navi_msg.header.frame_id = "usvc_navi";
                    
                    // 赋值数据
                    navi_msg.ins_sta = ((DATA_NAVI_t*)pDATA)->ins_sta;
                    navi_msg.timeTick = ((DATA_NAVI_t*)pDATA)->timeTick;
                    navi_msg.gyro_x = ((DATA_NAVI_t*)pDATA)->gyro_roll;
                    navi_msg.gyro_y = ((DATA_NAVI_t*)pDATA)->gyro_pitch;
                    navi_msg.gyro_z = ((DATA_NAVI_t*)pDATA)->gyro_heading;
                    navi_msg.roll = ((DATA_NAVI_t*)pDATA)->roll;
                    navi_msg.pitch = ((DATA_NAVI_t*)pDATA)->pitch;
                    navi_msg.heading = ((DATA_NAVI_t*)pDATA)->heading;
                    navi_msg.acc_x = ((DATA_NAVI_t*)pDATA)->acc_x;
                    navi_msg.acc_y = ((DATA_NAVI_t*)pDATA)->acc_y;
                    navi_msg.acc_z = ((DATA_NAVI_t*)pDATA)->acc_z;
                    navi_msg.vel_x = ((DATA_NAVI_t*)pDATA)->vel_x;
                    navi_msg.vel_y = ((DATA_NAVI_t*)pDATA)->vel_y;
                    navi_msg.vel_z = ((DATA_NAVI_t*)pDATA)->vel_z;
                    navi_msg.latitude = ((DATA_NAVI_t*)pDATA)->latitude;
                    navi_msg.longitude = ((DATA_NAVI_t*)pDATA)->longitude;
                    navi_msg.altitude = ((DATA_NAVI_t*)pDATA)->altitude;
                    navi_msg.vel_n = ((DATA_NAVI_t*)pDATA)->vel_n;
                    navi_msg.vel_e = ((DATA_NAVI_t*)pDATA)->vel_e;
                    navi_msg.vel_d = ((DATA_NAVI_t*)pDATA)->vel_d;
                    navi_msg.satellite_count = ((DATA_NAVI_t*)pDATA)->satellite_count;
                    navi_msg.pos_sta = ((DATA_NAVI_t*)pDATA)->pos_sta;
                    navi_msg.heading_sta = ((DATA_NAVI_t*)pDATA)->heading_sta;
                    navi_msg.position_std_lat = ((DATA_NAVI_t*)pDATA)->position_std_lat;
                    navi_msg.position_std_lon = ((DATA_NAVI_t*)pDATA)->position_std_lon;

                    //发布导航话题
                    usvc_navi_pub.publish(navi_msg);
                    break;
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}