#ifndef CONTROL_CALC_TOOL_H
#define CONTROL_CALC_TOOL_H

#include <string>
#include <array>
#include <vector>
#include <map>
#include <cmath>
#include <chrono>
#include <iomanip>    
#include <sstream>   
#include <iostream>
#include <Eigen/Core>

#define EARTH_RADIUS 6371393.0

class ControlCalaTool 
{

public:
    static double to_radians(double degrees);
    static double to_degrees(double radians);

    static double get_target_distance(const double &current_lat, const double &current_lon, const double &target_lat, const double &target_lon);
    static float  get_target_azimuth(const double &current_lat, const double &current_lon, const double &target_lat, const double &target_lon);
    static float smooth_acceleration(float distance_to_target, float max_speed, float stop_distance);
    static void project_point(double lat1, double lon1, double lat2, double lon2, double curr_lat, double curr_lon, double &proj_lat, double &proj_lon); 
    static void calculateDestination(double lat1, double lon1, double bearing, double distance, double &target_lat, double &target_lon);
    static int findNearestWaypoint(double lat, double lon,
                                   const std::vector<Eigen::Vector2d>& path);
};




#endif
