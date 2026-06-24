#ifndef MPC_MATH_UTILS_HPP
#define MPC_MATH_UTILS_HPP

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

namespace math_utils {

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

inline double computeDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1, dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

inline double computeBearing(double x1, double y1, double x2, double y2) {
    return normalizeAngle(std::atan2(y2 - y1, x2 - x1));
}

template<typename T>
inline T clip(const T& v, const T& lo, const T& hi) {
    return std::max(lo, std::min(v, hi));
}

inline double deg2rad(double d) { return d * PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / PI; }

// 글로벌 → 로컬 (자차 좌표계)
inline void globalToLocal(
    double gx, double gy,
    double ex, double ey, double eyaw,
    double& lx, double& ly)
{
    double dx = gx - ex, dy = gy - ey;
    double c = std::cos(eyaw), s = std::sin(eyaw);
    lx =  c*dx + s*dy;
    ly = -s*dx + c*dy;
}

// 로컬 → 글로벌
inline void localToGlobal(
    double lx, double ly,
    double ex, double ey, double eyaw,
    double& gx, double& gy)
{
    double c = std::cos(eyaw), s = std::sin(eyaw);
    gx = ex + c*lx - s*ly;
    gy = ey + s*lx + c*ly;
}

// 가장 가까운 점 인덱스 찾기 (시작 인덱스 옵션, 부분 탐색)
inline size_t findClosestPoint(
    double ex, double ey,
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    size_t start = 0,
    size_t end   = static_cast<size_t>(-1))
{
    if (xs.empty()) return 0;
    size_t n = xs.size();
    if (end > n) end = n;
    if (start >= end) start = 0;

    size_t best = start;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = start; i < end; ++i) {
        double dx = xs[i] - ex, dy = ys[i] - ey;
        double d2 = dx*dx + dy*dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best = i;
        }
    }
    return best;
}

} // namespace math_utils

#endif // MPC_MATH_UTILS_HPP
