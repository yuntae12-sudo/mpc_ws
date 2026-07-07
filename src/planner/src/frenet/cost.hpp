#ifndef FRENET_COST_HPP
#define FRENET_COST_HPP

#include "global/global.hpp"

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

#endif
