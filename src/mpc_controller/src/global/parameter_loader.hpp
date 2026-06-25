#ifndef MPC_PARAMETER_LOADER_HPP
#define MPC_PARAMETER_LOADER_HPP

#include "global.hpp"

// ros parameter server 에서 MPC 파라미터 로드 (mpc_params 채움)
void loadMPCParameters(ros::NodeHandle& pnh);

// CSV 에서 waypoints + reference origin 로드 (PlanningControl 호환)
bool loadWaypoints();

// 로드된 waypoints 에 곡률 정보 계산 (3점 원 근사)
void computeWaypointCurvatures();

#endif // MPC_PARAMETER_LOADER_HPP
