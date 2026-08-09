#include "udp_network/gps_transform.hpp"

#include <cmath>

namespace {
constexpr double kPi = M_PI;
constexpr double kWgs84A = 6378137.0;          // WGS-84 타원체 장축 반경 [m]
constexpr double kWgs84E2 = 0.006694379991;    // WGS-84 타원체 이심률의 제곱

// WGS84 위/경도/고도 -> ECEF(x,y,z). lat/lon은 라디안.
void LlaToEcef(double lat_rad, double lon_rad, double alt_m,
               double& x, double& y, double& z) {
    const double sin_lat = std::sin(lat_rad);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    x = (n + alt_m) * std::cos(lat_rad) * std::cos(lon_rad);
    y = (n + alt_m) * std::cos(lat_rad) * std::sin(lon_rad);
    z = (n * (1.0 - kWgs84E2) + alt_m) * sin_lat;
}
}  // namespace

void GpsToEnu(double lat_deg, double lon_deg, double alt_m,
              double ref_lat_deg, double ref_lon_deg, double ref_alt_m,
              double& out_e, double& out_n, double& out_u) {
    const double lat_rad = lat_deg * kPi / 180.0;
    const double lon_rad = lon_deg * kPi / 180.0;
    const double ref_lat_rad = ref_lat_deg * kPi / 180.0;
    const double ref_lon_rad = ref_lon_deg * kPi / 180.0;

    double x, y, z;
    LlaToEcef(lat_rad, lon_rad, alt_m, x, y, z);
    double ref_x, ref_y, ref_z;
    LlaToEcef(ref_lat_rad, ref_lon_rad, ref_alt_m, ref_x, ref_y, ref_z);

    const double dx = x - ref_x;
    const double dy = y - ref_y;
    const double dz = z - ref_z;

    const double sin_lat = std::sin(ref_lat_rad), cos_lat = std::cos(ref_lat_rad);
    const double sin_lon = std::sin(ref_lon_rad), cos_lon = std::cos(ref_lon_rad);

    // ECEF -> ENU 회전행렬. 세 축 모두 기준점(ref_lat/ref_lon)을 기준으로
    // 일관되게 적용한다(Study_ITS 원본의 Up 축 불일치 버그 수정).
    out_e = -sin_lon * dx + cos_lon * dy;
    out_n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    out_u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
}

void GpsToUtm52N(double lat_deg, double lon_deg,
                 double& out_easting, double& out_northing) {
    constexpr double kScale = 0.9996;
    constexpr double kFalseEasting = 500000.0;
    constexpr double kCentralMeridianDeg = 129.0;  // UTM zone 52

    const double lat = lat_deg * kPi / 180.0;
    const double lon = lon_deg * kPi / 180.0;
    const double lon0 = kCentralMeridianDeg * kPi / 180.0;
    const double ep2 = kWgs84E2 / (1.0 - kWgs84E2);
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double tan_lat = std::tan(lat);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    const double t = tan_lat * tan_lat;
    const double c = ep2 * cos_lat * cos_lat;
    const double a = cos_lat * (lon - lon0);

    const double e4 = kWgs84E2 * kWgs84E2;
    const double e6 = e4 * kWgs84E2;
    const double m = kWgs84A *
        ((1.0 - kWgs84E2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * lat
         - (3.0 * kWgs84E2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0)
               * std::sin(2.0 * lat)
         + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * std::sin(4.0 * lat)
         - (35.0 * e6 / 3072.0) * std::sin(6.0 * lat));

    out_easting = kFalseEasting + kScale * n *
        (a + (1.0 - t + c) * a * a * a / 6.0
         + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2)
               * a * a * a * a * a / 120.0);
    out_northing = kScale *
        (m + n * tan_lat *
             (a * a / 2.0
              + (5.0 - t + 9.0 * c + 4.0 * c * c) * a * a * a * a / 24.0
              + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2)
                    * a * a * a * a * a * a / 720.0));
    if (lat_deg < 0.0) out_northing += 10000000.0;
}

double QuaternionToYaw(double w, double x, double y, double z) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
