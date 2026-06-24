#ifndef MPC_CONSTRAINTS_HPP
#define MPC_CONSTRAINTS_HPP

#include "../Global/Global.hpp"
#include <vector>

// box constraint 클리핑
void clipSteering(MPCControl& u, double max_steering);
void clipAcceleration(MPCControl& u, double a_min, double a_max);
void clipControl(MPCControl& u, const MPCParams& params);

// 조향 변화율 제한
void clipSteeringRate(MPCControl& u_cur, const MPCControl& u_prev,
                      double max_rate, double dt);

// 가용성 체크 (정보용)
bool isControlFeasible(const MPCControl& u, const MPCParams& params);
bool isCollisionDetected(const MPCState& s, const CostmapInfo& costmap,
                          double lethal_threshold);

#endif // MPC_CONSTRAINTS_HPP
