#ifndef FRENET_REF_LINE_HPP
#define FRENET_REF_LINE_HPP

#include "global/global.hpp"

// =========================================================
// Center line 한 점의 정보
// Global path Waypoint를 arc length로 매개화한 결과
// =========================================================

struct RefPoint {
    double x, y;
    double s;
    double theta;       // heading
    double kappa;       // 곡률
    double d_kappa;     // 곡률 미분한 거
};

// =========================================================
// Center line 전체
// =========================================================

struct RefLine {

    std::vector<RefPoint> points;

};

// =========================================================
// 생성
// Waypoints (x, y Array) 받아 RefLine 구성
// heading, kappa, d_kappa 수치 미분으로 계산
// =========================================================

RefLine BuildRefLine(const std::vector<double>& wx,
                     const std::vector<double>& wy,
                     double max_curvature);

// =========================================================
// s* 탐색 (Cartesian -> Frenet First Step)
// 차량 위치 (x, y)에서 Center Line 위 가장 가까운 s 반환
// Newton Iteration으로 수치 풀이
// =========================================================

double FindClosestS(const RefLine& ref, double x, double y);

// =========================================================
// 보간, 임의의 s에서 RefPoint 반환(선형 보간)
// frenet_converter가 heading_r, kappa_r, d_kappa_r을 가져갈 때 사용
// =========================================================

RefPoint Interpolate(const RefLine& ref, double s);

#endif