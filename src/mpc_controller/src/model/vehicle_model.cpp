#include "vehicle_model.hpp"
#include "../global/utils.hpp"

// ========================================
// Kinematic Bicycle Model 한 스텝 적분 (Euler)
//   x_dot   = vx * cos(yaw)
//   y_dot   = vx * sin(yaw)
//   yaw_dot = vx / L * tan(delta)
//   vx_dot  = a
// ========================================
MPCState updateVehicleState(
    const MPCState&   state,
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

    next.x = state.x + vx * std::cos(yaw) * dt;
    next.y = state.y + vx * std::sin(yaw) * dt;

    if (std::fabs(vx) > 1e-3) {
        next.yaw = normalizeAngle(yaw + (vx / wheelbase) * std::tan(delta) * dt);
    } else {
        next.yaw = yaw;
    }

    next.vx = clip(vx + a * dt, v_min, v_max);
    return next;
}

// ========================================
// horizon 스텝 비선형 forward sim
// ========================================
std::vector<MPCState> predictTrajectory(
    const MPCState&                initial_state,
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

// ========================================
// 동작점 (x̄, ū) 에서 1차 Taylor 선형화
//
// bicycle model:
//   f0: x'   = x   + vx * cos(yaw) * dt
//   f1: y'   = y   + vx * sin(yaw) * dt
//   f2: yaw' = yaw + vx/L * tan(delta) * dt
//   f3: vx'  = vx  + a * dt
//
// 상태 인덱스: x=0, y=1, yaw=2, vx=3
// 입력 인덱스: delta=0, accel=1
//
// A[i][j] = ∂f_i/∂x_j  (동작점에서 편미분)
// B[i][j] = ∂f_i/∂u_j
// c       = f(x̄,ū) - A*x̄ - B*ū
// ========================================
LinearizedModel linearizeBicycle(
    const MPCState&   state,
    const MPCControl& control,
    double dt,
    double wheelbase)
{
    LinearizedModel lm;

    double yaw   = state.yaw;
    double vx    = state.vx;
    double delta = control.delta;
    double L     = wheelbase;

    // ---- A 행렬 (4×4): 단위행렬에서 시작 ----
    // 대각선: 자기 자신에 대한 편미분 = 1
    for (int i = 0; i < 4; ++i) lm.A[i][i] = 1.0;

    // f0(x') = x + vx*cos(yaw)*dt
    //   ∂f0/∂yaw = -vx * sin(yaw) * dt
    //   ∂f0/∂vx  =  cos(yaw) * dt
    lm.A[0][2] = -vx * std::sin(yaw) * dt;   // ∂x'/∂yaw
    lm.A[0][3] =  std::cos(yaw) * dt;         // ∂x'/∂vx

    // f1(y') = y + vx*sin(yaw)*dt
    //   ∂f1/∂yaw =  vx * cos(yaw) * dt
    //   ∂f1/∂vx  =  sin(yaw) * dt
    lm.A[1][2] =  vx * std::cos(yaw) * dt;   // ∂y'/∂yaw
    lm.A[1][3] =  std::sin(yaw) * dt;         // ∂y'/∂vx

    // f2(yaw') = yaw + vx/L * tan(delta) * dt
    //   ∂f2/∂vx  = tan(delta) / L * dt
    //   (vx 작을 때 보호: 동작점에서만 쓰므로 그냥 계산)
    lm.A[2][3] = std::tan(delta) / L * dt;    // ∂yaw'/∂vx

    // f3(vx') = vx + a*dt  → 대각 1 이미 설정됨, 추가 항 없음

    // ---- B 행렬 (4×2): 입력에 대한 편미분 ----
    // f2(yaw') = yaw + vx/L * tan(delta) * dt
    //   ∂f2/∂delta = vx/L * (1/cos²(delta)) * dt
    //   1/cos²(delta) = sec²(delta)
    double cos_d = std::cos(delta);
    double sec2  = 1.0 / (cos_d * cos_d + 1e-6);  // 분모 보호
    lm.B[2][0] = (vx / L) * sec2 * dt;            // ∂yaw'/∂delta

    // f3(vx') = vx + a*dt
    //   ∂f3/∂accel = dt
    lm.B[3][1] = dt;                               // ∂vx'/∂accel

    // ---- c 보정항: c = f(x̄,ū) - A*x̄ - B*ū ----
    // f(x̄,ū) 계산 (비선형 모델 직접 계산)
    double fx0 = state.x   + vx * std::cos(yaw) * dt;
    double fx1 = state.y   + vx * std::sin(yaw) * dt;
    double fx2, fx3;
    if (std::fabs(vx) > 1e-3) {
        fx2 = state.yaw + (vx / L) * std::tan(delta) * dt;
    } else {
        fx2 = state.yaw;
    }
    fx3 = vx + control.accel * dt;

    // x̄ = [state.x, state.y, state.yaw, state.vx]
    // ū = [control.delta, control.accel]
    double x_bar[4] = { state.x, state.y, state.yaw, state.vx };
    double u_bar[2] = { control.delta, control.accel };
    double f_bar[4] = { fx0, fx1, fx2, fx3 };

    // A*x̄ 계산
    double Ax[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            Ax[i] += lm.A[i][j] * x_bar[j];

    // B*ū 계산
    double Bu[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 2; ++j)
            Bu[i] += lm.B[i][j] * u_bar[j];

    // c = f(x̄,ū) - A*x̄ - B*ū
    for (int i = 0; i < 4; ++i)
        lm.c[i] = f_bar[i] - Ax[i] - Bu[i];

    return lm;
}

// ========================================
// horizon N 스텝 전체 선형화 모델 계산
//   각 스텝의 동작점: 비선형 forward sim으로 얻은 trajectory
// ========================================
std::vector<LinearizedModel> buildLTVModels(
    const MPCState&                x0,
    const std::vector<MPCControl>& U,
    double dt,
    double wheelbase,
    double v_min,
    double v_max)
{
    std::vector<LinearizedModel> models;
    models.reserve(U.size());

    MPCState cur = x0;
    for (size_t k = 0; k < U.size(); ++k) {
        // k번째 동작점 (cur, U[k]) 에서 선형화
        models.push_back(linearizeBicycle(cur, U[k], dt, wheelbase));
        // 다음 동작점: 비선형 모델로 한 스텝 전진
        cur = updateVehicleState(cur, U[k], dt, wheelbase, v_min, v_max);
    }
    return models;
}