#ifndef FRENET_PLANNER_PKG_UTILS_HPP
#define FRENET_PLANNER_PKG_UTILS_HPP

#include <algorithm>
#include <cmath>

// mpc_controller/src/global/utils.hpp의 축소판. 원본은 MPC 레이어의 global.hpp를
// 끌어왔지만 실제로 쓰는 건 이 파일의 범용 수학 유틸뿐이라, 패키지 분리하면서
// 그 의존을 끊고 <cmath>/<algorithm>만 직접 포함한다.

constexpr double PI      = M_PI;
constexpr double TWO_PI  = 2.0 * M_PI;
constexpr double HALF_PI = 0.5 * M_PI;

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

#endif  // FRENET_PLANNER_PKG_UTILS_HPP
