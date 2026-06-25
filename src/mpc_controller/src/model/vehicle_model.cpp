#include "vehicle_model.hpp"
#include "../global/utils.hpp"

// ========================================
// Kinematic Bicycle Model (rear axle reference)
//   x_dot   = vx * cos(yaw)
//   y_dot   = vx * sin(yaw)
//   yaw_dot = vx / L * tan(delta)
//   vx_dot  = a
// (vy 는 kinematic 모델에서 별도 상태가 아님)
// ========================================
MPCState updateVehicleState(
    const MPCState& state,
    const MPCControl& control,
    double dt,
    double wheelbase,
    double v_min,
    double v_max)
{
    MPCState next;
    double yaw   = state.yaw;
    double vx    = state.vx;
    double delta = control.delta;
    double a     = control.accel;

    // 위치
    next.x = state.x + vx * dt * std::cos(yaw);
    next.y = state.y + vx * dt * std::sin(yaw);

    // yaw (vx 가 너무 작을 때는 회전 거의 없음 - 분모 보호)
    if (std::fabs(vx) > 1e-3) {
        next.yaw = normalizeAngle(yaw + (vx / wheelbase) * std::tan(delta) * dt);
    } else {
        next.yaw = yaw;
    }

    // 종속도 (양/최대 클리핑)
    next.vx = clip(vx + a * dt, v_min, v_max);

    return next;
}

std::vector<MPCState> predictTrajectory(
    const MPCState& initial_state,
    const std::vector<MPCControl>& control_sequence,
    double dt,
    double wheelbase,
    double v_min,
    double v_max)
{
    std::vector<MPCState> traj;
    traj.reserve(control_sequence.size() + 1);
    traj.push_back(initial_state);

    MPCState cur = initial_state;
    for (const auto& u : control_sequence) {
        cur = updateVehicleState(cur, u, dt, wheelbase, v_min, v_max);
        traj.push_back(cur);
    }
    return traj;
}
