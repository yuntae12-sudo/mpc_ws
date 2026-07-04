#ifndef FRENET_FRENET_TYPES_HPP
#define FRENET_FRENET_TYPES_HPP

#include "global/global.hpp"

// =========================================================
// Ego 차량의 현재 Frenet 상태
// MakeQuintic/MakeQuartic의 Start 조건이 나오는 곳
// -> 후보 궤적을 만들기 위한 Start 조건
//==========================================================

struct FrenetState {
    // Longitudinal (along center line)
    double s;       // arc length
    double s_d;     // s_dot: speed
    double s_dd;    // s_ddot: accelration

    // Lateral (Perpendicular to center line)
    double d;       // Lateral Offset
    double d_d;     // d_dot: speed
    double d_dd;    // d_ddot: accelration

};

// =========================================================
// 후보 궤적 하나 (Frenet 공간)
// PathGenerator가 생성, Cost/CollisionChecker가 읽음
// =========================================================

struct FrenetPath {
    // Time Sample Array
    std::vector<double> t;

    // Lateral
    std::vector<double> d;
    std::vector<double> d_d;
    std::vector<double> d_dd;

    // Longitudinal
    std::vector<double> s;
    std::vector<double> s_d;
    std::vector<double> s_dd;

    // Cost (우선 논문 기반 식으로 작성, C_tot = k_lat * C_lat + k_lon * C_lon)
    double cost_lat;
    double cost_lon;
    double cost_total;

    // --- cost.hpp가 비용을 조립하는 데 필요한 재료 ---
    // Jt = ∫(p''')^2 dt (Prop.1의 jerk 적분항). path_generator가 다항식 계수를
    // 갖고 있는 시점(생성 직후)에 닫힌 형태(해석적 공식)로 미리 계산해서 채워넣는다.
    // 샘플링된 d/s 배열만으로는 원본 다항식 계수를 복원할 수 없어서 사후에는 계산 불가.
    double jerk_cost_lat;   // Jt(d(t))
    double jerk_cost_lon;   // Jt(s(t))

    // 종료조건 오프셋 (Sec.V-A/B의 [s1-sd] = Δsi, [s_dot1-s_dot_d] = Δs_dot_i).
    // Following/Merging/Stopping 후보는 delta_s만, Velocity Keeping 후보는
    // delta_s_dot만 채우고 나머지는 0으로 둔다 (둘 다 0이면 자동으로 그 항이 사라짐).
    double delta_s;
    double delta_s_dot;

    // Available Flag
    bool valid;

};

// =========================================================
// Frenet -> Cartesian 변환 결과 (Tracking Controller 입력)
// 이 구조체가 MPC Controller로 넘어감.
// =========================================================

struct CartesianPath {

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> kappa;
    std::vector<double> v;
    std::vector<double> a;

};

#endif