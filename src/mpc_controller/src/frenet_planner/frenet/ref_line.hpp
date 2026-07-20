#ifndef FRENET_REF_LINE_HPP
#define FRENET_REF_LINE_HPP

#include "frenet_planner/global/global.hpp"

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

// s_hint가 주어지면(nullptr이 아니면), 전체 경로가 아니라 s_hint ± window
// 구간의 점들 중에서만 "가장 가까운 초기값"을 찾는다. 트랙이 자기 자신과
// 가까이 지나가는 구간(헤어핀, 루프 등)에서 매 사이클 전체 탐색을 하면
// 실제로는 몇 m밖에 안 움직였는데도 물리적으로 가깝지만 s로는 수백~수천m
// 떨어진 다른 구간의 점으로 최근접점이 순간적으로 튀어버리는 문제가 있다
// (실측 재현: 한 틱 사이 s가 2176->2048, d가 1->720m로 폭주). 이전 사이클의
// s 근방으로 탐색을 제한하면(한 제어 틱에 차량이 이동 가능한 거리는 window에
// 비해 훨씬 작으므로) 이 문제가 생기지 않는다.
double FindClosestS(const RefLine& ref, double x, double y,
                     const double* s_hint = nullptr, double window = 30.0);

// =========================================================
// 보간, 임의의 s에서 RefPoint 반환(선형 보간)
// frenet_converter가 heading_r, kappa_r, d_kappa_r을 가져갈 때 사용
// =========================================================

RefPoint Interpolate(const RefLine& ref, double s);

#endif