#ifndef FRENET_COST_HPP
#define FRENET_COST_HPP

#include "frenet_planner/global/global.hpp"

struct CostWeights {
    double kj;      // jerk 적분항 가중치 (공통)
    double kt;      // 시간항 가중치 (공통, g(T)=T)
    double kd;      // lateral 종료위치 가중치 (kd*d1^2) - "끝나는 지점"만 봄
    // kd*d1^2는 T 끝에서 d가 0으로 돌아오기만 하면 0이라, 그 사이(중간 궤적)에
    // 얼마나 옆으로 벌어졌는지는 전혀 안 본다. kj(jerk)가 작을 때는 "끝에서만
    // 살짝 중앙으로 붙고 중간엔 넓게 돌아서 부드럽게(=jerk 적은) 가는" 경로가
    // kd 비용은 그대로 0이면서 jerk 비용까지 싸져서 최적이 되어버린다(실측
    // 재현: kj=0.1일 때 d가 -0.03에서 -2.7까지 부풀었다가 다시 돌아옴 - 도착
    // 조건 d1≈0만 만족하면 되니 중간 이탈에는 비용이 안 걸림). 이걸 막으려고
    // "경로 전체에 걸친 d^2 적분" 항을 추가한다 - 중간에 멀리 벗어나 있는
    // 시간이 길수록 비용이 커져서, jerk가 조금 늘어도 중앙에 더 붙어있는
    // 경로를 선호하게 만든다.
    double kd_path;
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

#endif
