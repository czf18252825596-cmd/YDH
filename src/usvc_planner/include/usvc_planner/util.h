#ifndef USVC_PLANNER_UTILS_H
#define USVC_PLANNER_UTILS_H

#include <cmath>

namespace usvc_planner {

// 角度归一化到 [-pi, pi]
inline double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// 2D 距离（勾股定理）
inline double dist2D(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

// 安全取模（处理负数）
inline int safeMod(int a, int b) {
    return ((a % b) + b) % b;
}

}  // namespace usvc_planner

#endif
