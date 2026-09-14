#include <ros/ros.h>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <usvc_navi/navi.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>

#include "usvc_controller/UpdateParams.h"


static float X_DISTANCE = 0.01;   
static float Y_DISTANCE = 0.01;   
static float Z_DISTANCE = 0.01;  
static float ClusterTolerance = 0.1;    // 聚类半径
static int MinClusterSize = 2; // 最小聚类大小，小于则丢弃(噪点)
static int MaxClusterSize = 200000; // 最大聚类大小，超过则丢弃


static ros::Subscriber lidar_sub;
static ros::Subscriber navi_sub;

static ros::Publisher lidar_filter_pub; 
static ros::Publisher navi_euler_pub; 
static ros::Publisher navi_gnss_pub; 

static ros::ServiceServer lidar_param_service;

static bool lidarParamsCallback(usvc_controller::UpdateParams::Request& req, usvc_controller::UpdateParams::Response& res) {
    
    if (req.node_name != "sensor") 
    {
        res.success = false;
        res.message = "Not for sensor node";
        return true;
    }
    
    // 检查数组长度
    if (req.param_names.size() != req.param_values.size())
    {
        res.success = false;
        res.message = "Parameter arrays length mismatch";
        return true;
    }
    
    // ========== 1. 更新内存变量（立即生效）==========
    for (size_t i = 0; i < req.param_names.size(); i++) 
    {
        const std::string& name = req.param_names[i];
        const std::string& value = req.param_values[i];
        
        ROS_DEBUG("  %s = %s", name.c_str(), value.c_str());
        
        if (name == "X_DISTANCE") 
        {
            X_DISTANCE = std::stof(value);
        }
        else if (name == "Y_DISTANCE") 
        {
            Y_DISTANCE = std::stof(value);
        }
        else if (name == "Z_DISTANCE") 
        {
            Z_DISTANCE = std::stof(value);
        }
        else if (name == "ClusterTolerance") 
        {
            ClusterTolerance = std::stof(value);
        }
        else if (name == "MinClusterSize")
        {
            MinClusterSize = std::stoi(value);
        }
        else if (name == "MaxClusterSize") 
        {
            MaxClusterSize = std::stoi(value);
        }
        else 
        {
            ROS_WARN("---[sensor_node.cpp] Unknown parameter: %s", name.c_str());
        }
    }

    ROS_INFO("---[sensor_node.cpp] lidarParams\r\n \
    X_DISTANCE:%f\r\n \
    Y_DISTANCE:%f\r\n \
    Z_DISTANCE:%f\r\n \
    ClusterTolerance:%f\r\n \
    MinClusterSize:%d\r\n \
    MaxClusterSize:%d",
    X_DISTANCE, Y_DISTANCE, Z_DISTANCE, ClusterTolerance, MinClusterSize, MaxClusterSize);
    
    res.success = true;
    res.message = "Parameters updated successfully";
    
    return true;
}

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
    //激光雷达坐标 XYZ 前左上

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<Eigen::Vector3f> pointCloudVector;

    // 1.ros 数据接收点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    // cloud_filtered->reserve(cloud->size()); // 提前分配内存，防止点云太大导致卡顿

    // for (const auto& point : cloud->points) 
    // {
    //     double distance = std::hypot(point.x, point.y);
    //     double angle = std::atan2(point.y, point.x) * 180.0 / M_PI; 

    //     // 1. 前方机械臂 (V型) 
    //     // 角度在 -30 到 30 之间，并且假设距离雷达 1.5 米以内
    //     bool in_front_arm = (angle >= -30.0 && angle <= 30.0) && (distance < 1.5);

    //     // 2. 后方阴影盲区 (并集)
    //     // 角度大于 40度 (40~180) 或者 角度小于 -40度 (-40~-180)
    //     bool in_rear_blind = (angle >= 40.0 || angle <= -40.0);
        
    //     // 【一个小优化】：上面的 in_rear_blind 其实就等价于绝对值大于 40 度
    //     // bool in_rear_blind = (std::abs(angle) >= 40.0);

    //     // 3. 裁切动作
    //     if (in_front_arm || in_rear_blind) 
    //     {
    //         continue; // 切掉！丢弃这些点
    //     }

    //     cloud_filtered->points.push_back(point);
    // }
    // *cloud = *cloud_filtered; 

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filte(new pcl::PointCloud<pcl::PointXYZ>());
    // 创建条件
    pcl::ConditionOr<pcl::PointXYZ>::Ptr xy_cond(new pcl::ConditionOr<pcl::PointXYZ>());

    // 条件1：保留 x ≤ -1*MIN_DISTANCE  和 x ≥ MIN_DISTANCE
    xy_cond->addComparison(
        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, -1 * X_DISTANCE)));
    xy_cond->addComparison(
        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 1 * X_DISTANCE)));

    // 条件2：-MIN_DISTANCE ≤ y ≤ MIN_DISTANCE
    xy_cond->addComparison(
        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, -1 * Y_DISTANCE)));
    xy_cond->addComparison(
        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 1 * Y_DISTANCE)));
    
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addCondition(xy_cond);

    // 条件3:保留z<1
    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, Z_DISTANCE)));

    // 创建条件滤波器
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);

    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);  // 保持点云的组织结构
    condrem.filter(*cloud_filte);

    // 3. 进行降采样 (体素滤波)
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_filte);

    // 体素大小
    voxel.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel.filter(*cloud_filte);
    if(!cloud_filte->empty())
    {
        // 4. 聚类 KD-Tree
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud_filte);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // 0.5 m 内算同一类
        ec.setClusterTolerance(ClusterTolerance);
        // 最小聚类大小，小于则丢弃
        ec.setMinClusterSize(MinClusterSize);
        // 最大聚类大小，超过则丢弃或分割
        ec.setMaxClusterSize(MaxClusterSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filte);
        ec.extract(cluster_indices);

        // for (const auto& cluster : cluster_indices) 
        // {
        //     float min_distance = std::numeric_limits<float>::max();
        //     Eigen::Vector3f closest_point(0, 0, 0); 

        //     // 遍历当前簇中的所有点，找到距离当前位置最近的点
        //     for (int index : cluster.indices) 
        //     {
        //         Eigen::Vector3f point(cloud_filte->points[index].x, 
        //                               cloud_filte->points[index].y, 
        //                               cloud_filte->points[index].z);

        //         // 计算当前位置与当前点的欧几里得距离（2D）
        //         float distance = point.norm();  // 只计算 x 和 y 坐标的距离

        //         // 如果当前点距离当前位置更近，更新最小距离和最近的点
        //         if (distance < min_distance) 
        //         {
        //             min_distance = distance;
        //             closest_point = point;
        //         }
        //     }

        //     // 将距离当前位置最近的点作为该簇的代表点
        //     pointCloudVector.emplace_back(closest_point);
        // }

        static constexpr int CLUSTER_POINTS_TO_KEEP = 4;  // 可调参数

        for (const auto& cluster : cluster_indices) 
        {
            // 1. 收集当前簇中所有点的距离和坐标
            std::vector<std::pair<float, Eigen::Vector3f>> points_with_dist;
            points_with_dist.reserve(cluster.indices.size()); //提前分配空间，避免自动扩容

            for (int index : cluster.indices) 
            {
                Eigen::Vector3f point(cloud_filte->points[index].x, 
                                      cloud_filte->points[index].y, 
                                      cloud_filte->points[index].z);
                float distance = point.norm();  // 到船的距离（2D 欧氏距离）
                points_with_dist.emplace_back(distance, point);
            }

            // 2. 按距离从小到大排序    （Lambda 结构）
            std::sort(points_with_dist.begin(), points_with_dist.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });

            // 3. 取前 N 个最近点（保留宽度信息）
            int keep = std::min(CLUSTER_POINTS_TO_KEEP, (int)points_with_dist.size());
            for (int i = 0; i < keep; ++i) 
            {
                pointCloudVector.emplace_back(points_with_dist[i].second);
            }
        }



    }
    else
    {
        pointCloudVector.clear();
    }

    //创建可视化点云,发布
    // 1. 创建 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 2. 填充点云数据
    for (const auto& point : pointCloudVector) 
    {
        filtered_cloud->points.emplace_back(point.x(), point.y(), point.z());
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;  // 1D 点云
    filtered_cloud->is_dense = true;  // 假设无无效点

    // 3. 转换为 ROS 点云消息s
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = msg->header;  // 继承 LiDAR 原始时间戳
    filtered_cloud_msg.header.frame_id = "base_link";  // 确保 frame_id 正确

    // 4. 发布滤波后的点云
    lidar_filter_pub.publish(filtered_cloud_msg);

    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double, std::milli>(end_time - start_time).count(); // 毫
}


void naviCallback(const usvc_navi::navi& msg) 
{
    // 发布欧拉角话题 (使用geometry_msgs::Vector3)
    geometry_msgs::Vector3 euler_msg;
    euler_msg.x = msg.roll;      // roll
    euler_msg.y = msg.pitch;     // pitch
    euler_msg.z = msg.heading;   // heading/yaw
    
    navi_euler_pub.publish(euler_msg);

    // 发布GPS话题 (使用sensor_msgs::NavSatFix)
    sensor_msgs::NavSatFix gnss_msg;
    gnss_msg.header.stamp = ros::Time::now();  // 或者使用 msg.timeTick 转换
    gnss_msg.header.frame_id = "gnss";
    
    gnss_msg.latitude = msg.latitude;
    gnss_msg.longitude = msg.longitude;
    gnss_msg.altitude = msg.altitude;

    // 设置状态和精度信息
    gnss_msg.status.status = msg.pos_sta;  // 位置状态
    gnss_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    // 设置位置协方差（如果有标准差的估计）
    gnss_msg.position_covariance[0] = msg.position_std_lat * msg.position_std_lat;  // 纬度方差
    gnss_msg.position_covariance[4] = msg.position_std_lon * msg.position_std_lon;  // 经度方差
    gnss_msg.position_covariance[8] = 0;  // 高度方差（如果没有提供）
    
    gnss_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    
    navi_gnss_pub.publish(gnss_msg);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "sensor_node");
    
    ros::NodeHandle nh;

    // 从参数服务器获取配置
    nh.param<float>("sensor/lidar/X_DISTANCE", X_DISTANCE, 0.01);
    nh.param<float>("sensor/lidar/Y_DISTANCE", Y_DISTANCE, 0.01);
    nh.param<float>("sensor/lidar/Z_DISTANCE", Z_DISTANCE, 0.01);
    nh.param<float>("sensor/lidar/ClusterTolerance", ClusterTolerance, 0.1);
    nh.param<int>("sensor/lidar/MaxClusterSize", MaxClusterSize, 200000);
    nh.param<int>("sensor/lidar/MinClusterSize", MinClusterSize, 2);

    lidar_sub = nh.subscribe("/livox/lidar", 1, lidarCallback); //原始雷达订阅
    lidar_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("/sensor/lidar_filter", 10); //新雷达数据发布

    navi_sub = nh.subscribe("/usvc_navi/navi", 1, naviCallback); //导航订阅
    navi_euler_pub = nh.advertise<geometry_msgs::Vector3>("/sensor/navi_euler", 10); //IMU发布
    navi_gnss_pub = nh.advertise<sensor_msgs::NavSatFix>("/sensor/navi_gnss", 10); //导航发布

    lidar_param_service = nh.advertiseService("lidar/update_params", lidarParamsCallback);

    ros::spin();  // 直接使用spin，程序会阻塞在这里，等待回调执行
    
    return 0;
}