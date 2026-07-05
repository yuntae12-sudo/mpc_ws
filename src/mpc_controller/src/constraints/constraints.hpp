#ifndef MPC_CONSTRAINTS_HPP
#define MPC_CONSTRAINTS_HPP

#include "../global/global.hpp"

// box constraint 클리핑
void clipSteering(MPCControl& u, double max_steering);
void clipAcceleration(MPCControl& u, double a_min, double a_max);
void clipControl(MPCControl& u, const MPCParams& params);

// 조향 변화율 제한
void clipSteeringRate(MPCControl& u_cur, const MPCControl& u_prev,
                      double max_rate, double dt);

#endif // MPC_CONSTRAINTS_HPP
