#include <ros/ros.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <ros/package.h>  // 用于获取包路径
#include <unistd.h>       // 用于获取当前路径
#include <sensor_msgs/NavSatFix.h>
#include "usvc_controller/ACHData.h"
#include "control_calc_tool/control_calc_tool.h"

static double total_distance = 0, total_operating_hours = 0;
static double last_latitude = 0, last_longitude = 0;
static int time_cnt = 0;
static std::string filePath;

static ros::Subscriber navi_gnss_sub;
static ros::Publisher ACHData_pub; 


//导航GNSS数据
static void naviGnssCallback(const sensor_msgs::NavSatFix& msg)
{
    if(last_latitude == 0 && last_longitude == 0)
    {
        last_latitude = msg.latitude;
        last_longitude = msg.longitude;
    }
    else
    {
        double current_lat = msg.latitude;
        double current_lon = msg.longitude;

        double distance = ControlCalaTool::get_target_distance(last_latitude, last_longitude, current_lat, current_lon);

        if(distance >=0 && distance < 200.0){
            total_distance += distance / 1000.0; 

            last_latitude = current_lat;
            last_longitude = current_lon;
            
        }
        else
        {
            last_latitude = current_lat;
            last_longitude = current_lon;
        }
    }

    total_operating_hours += 0.00027777; //1s转为h

    usvc_controller::ACHData ach_msg;
    ach_msg.total_distance = total_distance;
    ach_msg.total_operating_hours = total_operating_hours;
    ACHData_pub.publish(ach_msg);

    time_cnt++;
    if(time_cnt >= 60) //一分钟
    {
        time_cnt = 0;
        boost::property_tree::ptree pt;
        // 写入数据
        pt.put("total distance", total_distance); 
        pt.put("total operating hours", total_operating_hours);
        // 输出为 JSON 格式的文件
        boost::property_tree::write_json(filePath, pt);
    }
} 


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ach_node");
    
    ros::NodeHandle nh;
    navi_gnss_sub = nh.subscribe("/sensor/navi_gnss", 1, naviGnssCallback);
    ACHData_pub   = nh.advertise<usvc_controller::ACHData>("/ach/ACHData", 1); //

    filePath = ros::package::getPath("usvc_controller")+"/file/ach_memory.json";

    // 查文件，没有的话，新建一个
    if (boost::filesystem::exists(filePath)) 
    {
        // 创建 property_tree 对象
        boost::property_tree::ptree pt;

        // 从文件读取 JSON 数据
        boost::property_tree::read_json(filePath, pt);
        // 读取 "total distance" 数据
        total_distance = pt.get<double>("total distance");
        total_operating_hours = pt.get<double>("total operating hours");          //总作业时长 h

    } 
    else 
    {
        boost::property_tree::ptree pt;
        // 写入数据
        pt.put("total distance", 0.0);  
        pt.put("total operating hours", 0.0);
        // 输出为 JSON 格式的文件
        boost::property_tree::write_json(filePath, pt);
        total_distance = 0.0;
        total_operating_hours = 0.0;     
    }

    ros::Rate rate(1); // 1Hz
    while (ros::ok()) 
    {
        ros::spinOnce();    
        rate.sleep();
    }
}