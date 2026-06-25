#ifndef MPC_VEHICLE_MODEL_HPP
#define MPC_VEHICLE_MODEL_HPP

#include "../global/global.hpp"

// 자전거 모델 한 스텝 적분 (Runge-Kutta 미적용, Euler)
MPCState updateVehicleState(
    const MPCState& state,
    const MPCControl& control,
    double dt,
    double wheelbase,
    double v_min,
    double v_max);

// horizon 스텝 forward sim
//   결과: predicted_states.size() == control_sequence.size() + 1
std::vector<MPCState> predictTrajectory(
    const MPCState& initial_state,
    const std::vector<MPCControl>& control_sequence,
    double dt,
    double wheelbase,
    double v_min,
    double v_max);

#endif // MPC_VEHICLE_MODEL_HPP
