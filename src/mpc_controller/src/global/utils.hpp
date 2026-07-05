#ifndef UTILS_HPP
#define UTILS_HPP

#include "global.hpp"

// ============================= coord utils ==================================
// WGS84 → ECEF
void wgs84ToECEF(double lat, double lon, double h,
                 double& x, double& y, double& z);

// WGS84 → ENU (ref 기준)
void wgs84ToENU(double lat, double lon, double h,
                const CoordinateReference& ref,
                double& x, double& y, double& z);

// quaternion → yaw
double quaternionToYaw(double x, double y, double z, double w);


// ============================= math utils =================================
constexpr double PI       = M_PI;
constexpr double TWO_PI   = 2.0 * M_PI;
constexpr double HALF_PI  = 0.5 * M_PI;

// 각도 [-π, π] 정규화
inline double normalizeAngle(double angle) {
    while (angle >  PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

inline double angleDiff(double a, double b) {
    return normalizeAngle(a - b);
}

template<typename T>
inline T clip(const T& v, const T& lo, const T& hi) {
    return std::max(lo, std::min(v, hi));
}

inline double deg2rad(double d) { return d * PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / PI; }


#endif // MPC_COORD_UTILS_HPP
