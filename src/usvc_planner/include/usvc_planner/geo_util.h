#ifndef USVC_PLANNER_GEO_UTILS_H
#define USVC_PLANNER_GEO_UTILS_H

#include <cmath>

namespace usvc_planner {

// 地球半径（米）—— 与 USVC ControlCalaTool 保持一致
constexpr double EARTH_RADIUS = 6371393.0;

// 经纬度几何运算工具（拷自 USVC ControlCalaTool，改名 GeoUtils）
// 提供 LOS 制导需要的：距离、方位、投影点、预瞄点
    class GeoUtils {
    public:
        static double to_radians(double degrees);
        static double to_degrees(double radians);

        // Haversine 公式算两点球面距离（米）
        static double get_target_distance(const double &current_lat,
                                        const double &current_lon,
                                        const double &target_lat,
                                        const double &target_lon);

        // 算从 current 到 target 的方位角（度，0=北，顺时针，0~360）
        static float get_target_azimuth(const double &current_lat,
                                        const double &current_lon,
                                        const double &target_lat,
                                        const double &target_lon);

        // S 型速度平滑（用于接近目标时减速）
        static float smooth_acceleration(float distance_to_target,
                                        float max_speed,
                                        float stop_distance);

        // 把 curr 投影到 AB 段上，返回投影点经纬度 proj_lat/lon
        // LOS 制导的核心：算船在路径段上的投影
        static void project_point(double lat1, double lon1,
                                double lat2, double lon2,
                                double curr_lat, double curr_lon,
                                double &proj_lat, double &proj_lon);

        // 从 (lat1,lon1) 沿方位 bearing 走 distance 米，得目标点
        // LOS 制导的核心：从投影点沿路径方向前移 LOS 距离得预瞄点
        static void calculate_destination(double lat1, double lon1,
                                        double bearing, double distance,
                                        double &target_lat, double &target_lon);
    };

}  // namespace usvc_planner

#endif
