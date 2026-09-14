#include "usvc_planner/geo_util.h"
#include <algorithm>

namespace usvc_planner {

double GeoUtils::to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

double GeoUtils::to_degrees(double radians) {
    return radians * 180.0 / M_PI;
}

// Haversine 公式算两点球面距离
double GeoUtils::get_target_distance(const double &current_lat,
                                       const double &current_lon,
                                       const double &target_lat,
                                       const double &target_lon) {
    double lat1 = to_radians(current_lat);
    double lon1 = to_radians(current_lon);
    double lat2 = to_radians(target_lat);
    double lon2 = to_radians(target_lon);

    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dLon / 2) * std::sin(dLon / 2);
    double c = 2 * std::asin(std::sqrt(a));

    return EARTH_RADIUS * c;
}

// 方位角（0=北，顺时针，0~360）
float GeoUtils::get_target_azimuth(const double &current_lat,
                                    const double &current_lon,
                                    const double &target_lat,
                                    const double &target_lon) {
    double lat1 = to_radians(current_lat);
    double lon1 = to_radians(current_lon);
    double lat2 = to_radians(target_lat);
    double lon2 = to_radians(target_lon);

    double dlon = lon2 - lon1;

    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    float azimuth = static_cast<float>(std::atan2(y, x) * 180.0 / M_PI);

    return std::fmod((azimuth + 360.0), 360.0);
}

// S 型加速函数（smoothstep）
float GeoUtils::smooth_acceleration(float distance_to_target,
                                     float max_speed,
                                     float stop_distance) {
    float t = std::min(distance_to_target / stop_distance, 1.0f);
    return max_speed * (3 * t * t - 2 * t * t * t);
}

// 球面投影：把 curr 投影到 AB 段
void GeoUtils::project_point(double lat1, double lon1,
                               double lat2, double lon2,
                               double curr_lat, double curr_lon,
                               double &proj_lat, double &proj_lon) {
    double lat1r = to_radians(lat1), lon1r = to_radians(lon1);
    double lat2r = to_radians(lat2), lon2r = to_radians(lon2);
    double latr  = to_radians(curr_lat), lonr = to_radians(curr_lon);

    // 地心直角坐标
    double x1 = std::cos(lat1r) * std::cos(lon1r), y1 = std::cos(lat1r) * std::sin(lon1r), z1 = std::sin(lat1r);
    double x2 = std::cos(lat2r) * std::cos(lon2r), y2 = std::cos(lat2r) * std::sin(lon2r), z2 = std::sin(lat2r);
    double x  = std::cos(latr)  * std::cos(lonr),  y  = std::cos(latr)  * std::sin(lonr),  z  = std::sin(latr);

    // 投影到 AB 段
    double vx = x2 - x1, vy = y2 - y1, vz = z2 - z1;
    double wx = x  - x1, wy = y  - y1, wz = z  - z1;
    double dot_vw = vx * wx + vy * wy + vz * wz;
    double dot_vv = vx * vx + vy * vy + vz * vz;
    double t = dot_vw / dot_vv;

    // 限制在 AB 段内
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    double px = x1 + t * vx, py = y1 + t * vy, pz = z1 + t * vz;

    proj_lat = to_degrees(std::asin(pz));
    proj_lon = to_degrees(std::atan2(py, px));
}

// 从起点沿方位角走指定距离，算终点
void GeoUtils::calculate_destination(double lat1, double lon1,
                                       double bearing, double distance,
                                       double &target_lat, double &target_lon) {
    lat1    *= M_PI / 180.0;
    lon1    *= M_PI / 180.0;
    bearing *= M_PI / 180.0;

    double lat2 = std::asin(std::sin(lat1) * std::cos(distance / EARTH_RADIUS) +
                            std::cos(lat1) * std::sin(distance / EARTH_RADIUS) * std::cos(bearing));

    double lon2 = lon1 + std::atan2(std::sin(bearing) * std::sin(distance / EARTH_RADIUS) * std::cos(lat1),
                                    std::cos(distance / EARTH_RADIUS) - std::sin(lat1) * std::sin(lat2));

    target_lat = lat2 * 180.0 / M_PI;
    target_lon = lon2 * 180.0 / M_PI;
}

}  // namespace usvc_planner
