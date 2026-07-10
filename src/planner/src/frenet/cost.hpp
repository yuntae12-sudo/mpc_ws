#ifndef FRENET_COST_HPP
#define FRENET_COST_HPP

#include "global/global.hpp"
#include "frenet/path_generator.hpp"

struct CostWeights {
    double kj;      // jerk 적분항 가중치 (공통)
    double kt;      // 시간항 가중치 (공통, g(T)=T)
    double kd;      // lateral 종료위치 가중치 (kd*d1^2)
    double ks;      // longitudinal 목표위치 가중치 (following/merging/stopping, ks*delta_s^2)
    double ks_dot;  // longitudinal 목표속도 가중치 (velocity keeping, ks_dot*delta_s_dot^2)
    double klat;    // Sec.VI: C_tot = klat*C_lat + klon*C_lon
    double klon;
};

double ComputeLateralCost(const FrenetPath& path, const CostWeights& w);
double ComputeLongitudinalCost(const FrenetPath& path, const CostWeights& w);
double ComputeTotalCost(double cost_lat, double cost_lon, const CostWeights& w);
void EvaluateCosts(std::vector<FrenetPath>& candidates, const CostWeights& w);

const FrenetPath* SelectBestPath(const std::vector<FrenetPath>& candidates);

// =========================================================
// 경로 연속성(히스테리시스) — 사이클마다 거의 동률인 후보 사이에서
// 이유 없이 갈아타는(flapping) 걸 막는다. mpc_controller 재통합 전,
// controller 없이도 재현됐던 "직선 구간 발산"의 근본 원인(사이클마다
// 선택이 튀어서 목표 경로 자체가 끊김)에 대한 처방 (2026-07 설계).
//
// switch_margin은 절대값이 아니라 cost_total에 대한 비율로 잡는다 -
// cost_weights_emergency.kt(20.0)가 일반 모드(1.0)의 20배라 절대값
// 마진은 모드마다 스케일이 안 맞음.
// =========================================================

struct HysteresisConfig {
    double margin_fraction_velocity_keeping = 0.10;  // LANE_KEEPING/LANE_CHANGE_*/TURN_*/FOLLOWING
    double margin_fraction_stopping = 0.05;           // STOP/INTERSECTION_WAIT
    double margin_fraction_emergency = 0.0;           // EMERGENCY/AVOID - 즉시 반응, 히스테리시스 비활성
};

// 직전 사이클에서 선택된 후보를 식별하는 최소 파라미터. FrenetPath의
// delta_s/delta_s_dot이 모드별로 안 쓰는 쪽은 0으로 남는 것과 같은 방식으로,
// d1/T는 모드 무관 공통, delta_s/delta_s_dot 중 실제 쓰인 쪽만 의미를 가짐.
struct LastBestParams {
    bool valid = false;
    BehaviorState mode = LANE_KEEPING;
    double d1 = 0.0;
    double delta_s = 0.0;
    double delta_s_dot = 0.0;
    double T = 0.0;
};

// last_best를 갱신하며(in-out) 히스테리시스를 적용해 최종 선택을 반환한다.
// mode가 직전과 다르면(behavior 전환) 히스테리시스 없이 즉시 SelectBestPath 결과를 채택.
const FrenetPath* SelectBestPathWithHysteresis(const std::vector<FrenetPath>& candidates,
                                                BehaviorState mode,
                                                const PathGeneratorConfig& path_cfg,
                                                const HysteresisConfig& hyst_cfg,
                                                LastBestParams& last_best);

#endif
