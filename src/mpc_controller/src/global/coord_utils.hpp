#ifndef MPC_COORD_UTILS_HPP
#define MPC_COORD_UTILS_HPP

#include "Global.hpp"

// WGS84 → ECEF
void wgs84ToECEF(double lat, double lon, double h,
                 double& x, double& y, double& z);

// WGS84 → ENU (ref 기준)
void wgs84ToENU(double lat, double lon, double h,
                const CoordinateReference& ref,
                double& x, double& y, double& z);

// quaternion → yaw
double quaternionToYaw(double x, double y, double z, double w);

#endif // MPC_COORD_UTILS_HPP
