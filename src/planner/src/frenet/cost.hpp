#ifndef FRENET_COST_HPP
#define FRENET_COST_HPP

#include "frenet/frenet_types.hpp"

// =========================================================
// 비용 가중치 (parameter.yaml에서 로드될 예정, 지금은 코드에서 직접 채워도 됨)
//
// 논문 Prop.1의 일반형 C = kj*Jt + kt*g(T) + kp*h(p1) 이 이 프로젝트에서
// 세 번 구체화된다 (kp 자리에 오는 가중치만 lateral/longitudinal에 따라
// kd, ks, ks_dot으로 나뉨):
//   Cd (Sec.IV-A 식(2)) = kj*Jt(d(t)) + kt*T + kd*d1^2
//   Ct (Sec.V-A)        = kj*Jt(s(t)) + kt*T + ks*[s1-sd]^2      (Δs = s1-sd)
//   Cv (Sec.V-B)        = kj*Jt(s(t)) + kt*T + ks_dot*[s_dot1-s_dot_d]^2
// kj, kt는 lateral/longitudinal 공통으로 재사용 (논문도 같은 문자를 씀).
// =========================================================

struct CostWeights {
    double kj;      // jerk 적분항 가중치 (공통)
    double kt;      // 시간항 가중치 (공통, g(T)=T)
    double kd;      // lateral 종료위치 가중치 (kd*d1^2)
    double ks;      // longitudinal 목표위치 가중치 (following/merging/stopping, ks*delta_s^2)
    double ks_dot;  // longitudinal 목표속도 가중치 (velocity keeping, ks_dot*delta_s_dot^2)
    double klat;    // Sec.VI: C_tot = klat*C_lat + klon*C_lon
    double klon;
};

// =========================================================
// [Sec.IV-A 식(2)] Cd = kj*Jt(d(t)) + kt*T + kd*d1^2
//
// T = path.t.back(), d1 = path.d.back() (path_generator가 이미 종료조건을
// d1으로 정확히 맞춰 생성했으므로 그대로 사용 가능).
// Jt(d(t))는 path.jerk_cost_lat에 미리 계산되어 있음 (path_generator 참고).
// =========================================================

double ComputeLateralCost(const FrenetPath& path, const CostWeights& w);

// =========================================================
// [Sec.V-A / Sec.V-B] Clon = kj*Jt(s(t)) + kt*T + ks*delta_s^2 + ks_dot*delta_s_dot^2
//
// Following/Merging/Stopping 후보는 delta_s만 채워져 있고 delta_s_dot=0,
// Velocity Keeping 후보는 반대이므로 두 항을 그냥 더해도 항상 해당 모드에
// 맞는 항 하나만 살아남는다 (분기 불필요).
// =========================================================

double ComputeLongitudinalCost(const FrenetPath& path, const CostWeights& w);

// =========================================================
// [Sec.VI] Ctot = klat*Clat + klon*Clon
// =========================================================

double ComputeTotalCost(double cost_lat, double cost_lon, const CostWeights& w);

// =========================================================
// candidates 전체에 대해 cost_lat/cost_lon/cost_total 필드를 채운다.
// valid 여부와 무관하게 계산해둔다 (디버그 시각화에서 invalid 후보의 비용도
// 확인할 수 있게 하기 위함 — 논문 Fig.3~6의 회색 궤적들도 비용값 자체는 존재).
// =========================================================

void EvaluateCosts(std::vector<FrenetPath>& candidates, const CostWeights& w);

// =========================================================
// [Sec.VII] valid한 후보 중 cost_total이 가장 작은 것을 선택.
// candidates가 비어있거나 valid한 후보가 하나도 없으면 nullptr 반환.
//
// 주의(스코프 제한): 논문 Sec.VII는 "여러 활성 모드를 동시에 만들어서
// 그중 초기 저크가 가장 작은 것"까지 비교하는 override 로직인데, 지금은
// PathGenerator::ResolveManeuver가 FSM으로부터 이미 단일 모드만 받는
// 전제라 이 함수는 "하나의 모드 안에서의 최적 후보 선택"까지만 담당한다.
// TODO(추후 개발 필요): FSM이 여러 활성 모드를 동시에 넘기도록 확장되면,
// 여기서 모드별 결과를 다시 비교하는 override 단계가 추가로 필요함.
// =========================================================

const FrenetPath* SelectBestPath(const std::vector<FrenetPath>& candidates);

#endif
