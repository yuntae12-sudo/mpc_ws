#include "utils.hpp"

// ====================== coord_utils =============================== 

// ========================================
// WGS84 → ECEF
// ========================================
void wgs84ToECEF(double lat, double lon, double h,
                 double& x, double& y, double& z)
{
    constexpr double a  = 6378137.0;
    constexpr double e2 = 6.69437999014e-3;

    double rad_lat = lat * M_PI / 180.0;
    double rad_lon = lon * M_PI / 180.0;
    double N = a / std::sqrt(1.0 - e2 * std::sin(rad_lat) * std::sin(rad_lat));

    x = (N + h) * std::cos(rad_lat) * std::cos(rad_lon);
    y = (N + h) * std::cos(rad_lat) * std::sin(rad_lon);
    z = (N * (1.0 - e2) + h) * std::sin(rad_lat);
}

// ========================================
// WGS84 → ENU
// ========================================
void wgs84ToENU(double lat, double lon, double h,
                const CoordinateReference& ref,
                double& x, double& y, double& z)
{
    double x_ecef, y_ecef, z_ecef;
    wgs84ToECEF(lat, lon, h, x_ecef, y_ecef, z_ecef);

    double dx = x_ecef - ref.x0_ecef;
    double dy = y_ecef - ref.y0_ecef;
    double dz = z_ecef - ref.z0_ecef;

    double rad_lat = ref.lat0 * M_PI / 180.0;
    double rad_lon = ref.lon0 * M_PI / 180.0;

    double t[3][3] = {
        { -std::sin(rad_lon),                std::cos(rad_lon),                0.0 },
        { -std::sin(rad_lat)*std::cos(rad_lon), -std::sin(rad_lat)*std::sin(rad_lon), std::cos(rad_lat) },
        {  std::cos(rad_lat)*std::cos(rad_lon),  std::cos(rad_lat)*std::sin(rad_lon), std::sin(rad_lat) }
    };

    x = t[0][0]*dx + t[0][1]*dy + t[0][2]*dz;
    y = t[1][0]*dx + t[1][1]*dy + t[1][2]*dz;
    z = t[2][0]*dx + t[2][1]*dy + t[2][2]*dz;
}

// ========================================
// quaternion → yaw
// ========================================
double quaternionToYaw(double x, double y, double z, double w)
{
    double siny_cosp = 2.0 * (w*z + x*y);
    double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
    return std::atan2(siny_cosp, cosy_cosp);
}

