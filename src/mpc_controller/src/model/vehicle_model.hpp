#ifndef MPC_VEHICLE_MODEL_HPP
#define MPC_VEHICLE_MODEL_HPP

#include "../global/global.hpp"

// ========================================
// 1차 Taylor 선형화 결과 구조체
//
//   비선형 bicycle model f(x, u)를
//   동작점 (x̄, ū) 근방에서 선형 근사:
//
//   x_{k+1} ≈ A_k * x_k + B_k * u_k + c_k
//
//   상태 x: [x, y, yaw, vx]  (4차원)
//   입력 u: [delta, accel]   (2차원)
// ========================================
struct LinearizedModel {
    // A: 4×4  (∂f/∂x, 상태에 대한 편미분)
    std::array<std::array<double, 4>, 4> A{};
    // B: 4×2  (∂f/∂u, 입력에 대한 편미분)
    std::array<std::array<double, 2>, 4> B{};
    // c: 4×1  affine 보정항  c = f(x̄,ū) - A*x̄ - B*ū
    std::array<double, 4> c{};
};

// ----------------------------------------
// Kinematic Bicycle Model 한 스텝 적분 (Euler)
//   x_dot   = vx * cos(yaw)
//   y_dot   = vx * sin(yaw)
//   yaw_dot = vx / L * tan(delta)
//   vx_dot  = a
// ----------------------------------------
MPCState updateVehicleState(
    const MPCState&   state,
    const MPCControl& control,
    double dt,
    double wheelbase,
    double v_min,
    double v_max);

// ----------------------------------------
// horizon 스텝 비선형 forward sim
//   결과 size() == control_sequence.size() + 1  (x0 포함)
// ----------------------------------------
std::vector<MPCState> predictTrajectory(
    const MPCState&                initial_state,
    const std::vector<MPCControl>& control_sequence,
    double dt,
    double wheelbase,
    double v_min,
    double v_max);

// ----------------------------------------
// 동작점 (x̄, ū) 에서 bicycle model 1차 Taylor 선형화
//
//   f(x, u) ≈ f(x̄, ū) + A*(x - x̄) + B*(u - ū)
//           = A*x + B*u + c       (c = f(x̄,ū) - A*x̄ - B*ū)
//
//   반환: LinearizedModel { A, B, c }
// ----------------------------------------
LinearizedModel linearizeBicycle(
    const MPCState&   state,    // 동작점 x̄
    const MPCControl& control,  // 동작점 ū
    double dt,
    double wheelbase);

// ----------------------------------------
// horizon N 스텝 전체 선형화 모델 계산
//   각 스텝의 동작점은 비선형 모델로 forward sim한 trajectory 사용
//   반환: size() == N  (k=0..N-1 각 스텝의 A_k, B_k, c_k)
// ----------------------------------------
std::vector<LinearizedModel> buildLTVModels(
    const MPCState&                x0,
    const std::vector<MPCControl>& U,
    double dt,
    double wheelbase,
    double v_min,
    double v_max);

#endif // MPC_VEHICLE_MODEL_HPP