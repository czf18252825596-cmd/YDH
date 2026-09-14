#include "control_calc_tool.h"

//转弧度
double ControlCalaTool::to_radians(double degrees) 
{
    return degrees * M_PI / 180.0;
}

//转角度
double ControlCalaTool::to_degrees(double radians) 
{
    return radians * 180.0 / M_PI;
}

//两点间距离计算
double ControlCalaTool::get_target_distance(const double &current_lat, const double &current_lon, const double &target_lat, const double &target_lon) 
{
    // 将经纬度从度转换为弧度
    double lat1 = to_radians(current_lat);
    double lon1 = to_radians(current_lon);
    double lat2 = to_radians(target_lat);
    double lon2 = to_radians(target_lon);

    // Haversine公式
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dLon / 2) * std::sin(dLon / 2);
    double c = 2 * std::asin(std::sqrt(a));

    return EARTH_RADIUS*c;


}

//两点间方位计算
float ControlCalaTool::get_target_azimuth(const double &current_lat, const double &current_lon, const double &target_lat, const double &target_lon) 
{
    // 将输入的角度转换为弧度
    double lat1 = to_radians(current_lat);
    double lon1 = to_radians(current_lon);
    double lat2 = to_radians(target_lat);
    double lon2 = to_radians(target_lon);

    // 计算经度差
    double dlon = lon2 - lon1;

    // 计算方位角
    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    float azimuth = static_cast<float>(std::atan2(y, x) * 180.0 / M_PI);  // 转换为度数

    // 将方位角调整到 [0, 360) 范围
    return std::fmod((azimuth + 360.0), 360.0);

}

//smooth加速处理
float ControlCalaTool::smooth_acceleration(float distance_to_target, float max_speed, float stop_distance) 
{
    float t = std::min(distance_to_target / stop_distance, 1.0f);  // 归一化距离
    return max_speed * (3 * t * t - 2 * t * t * t);  // S 型加速函数
}

//投影位置计算
void ControlCalaTool::project_point(double lat1, double lon1, double lat2, double lon2, double curr_lat, double curr_lon, double &proj_lat, double &proj_lon) 
{
    // 转换为弧度
    double lat1r = to_radians(lat1), lon1r = to_radians(lon1);
    double lat2r = to_radians(lat2), lon2r = to_radians(lon2);
    double latr = to_radians(curr_lat), lonr = to_radians(curr_lon);

    // 计算 3D 直角坐标 (地心坐标)
    double x1 = cos(lat1r) * cos(lon1r), y1 = cos(lat1r) * sin(lon1r), z1 = sin(lat1r);
    double x2 = cos(lat2r) * cos(lon2r), y2 = cos(lat2r) * sin(lon2r), z2 = sin(lat2r);
    double x = cos(latr) * cos(lonr), y = cos(latr) * sin(lonr), z = sin(latr);

    // 计算投影
    double vx = x2 - x1, vy = y2 - y1, vz = z2 - z1;
    double wx = x - x1, wy = y - y1, wz = z - z1;
    double dot_vw = vx * wx + vy * wy + vz * wz;
    double dot_vv = vx * vx + vy * vy + vz * vz;
    double t = dot_vw / dot_vv; // 投影比例

    // 手动限制投影点落在 AB 段内
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    double px = x1 + t * vx, py = y1 + t * vy, pz = z1 + t * vz;
    

    // 转回经纬度
    proj_lat = to_degrees(asin(pz));
    proj_lon = to_degrees(atan2(py, px));
}


//计算一个位置在一个方向一定距离后的位置
void ControlCalaTool::calculateDestination(double lat1, double lon1, double bearing, double distance, double &target_lat, double &target_lon) 
{
    lat1 *=  M_PI / 180.0;;
    lon1 *=  M_PI / 180.0;;
    bearing *=  M_PI / 180.0;;

    double lat2 = std::asin(std::sin(lat1) * std::cos(distance / EARTH_RADIUS) +
                            std::cos(lat1) * std::sin(distance / EARTH_RADIUS) * std::cos(bearing));

    double lon2 = lon1 + std::atan2(std::sin(bearing) * std::sin(distance / EARTH_RADIUS) * std::cos(lat1),
                                    std::cos(distance / EARTH_RADIUS) - std::sin(lat1) * std::sin(lat2));

    target_lat = lat2 * 180.0 / M_PI;
    target_lon = lon2 * 180.0 / M_PI;
}

int ControlCalaTool::findNearestWaypoint(double lat, double lon,
                                         const std::vector<Eigen::Vector2d>& path)
{
    int nearest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < path.size(); ++i) {
        double dlat = path[i].y() - lat;
        double dlon = path[i].x() - lon;
        double dist = dlat * dlat + dlon * dlon;
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = static_cast<int>(i);
        }
    }
    return nearest_idx;
}
