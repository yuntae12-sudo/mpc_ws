#include "cost_function.hpp"
#include "../global/utils.hpp"

// ========================================
// 경로 추종 오차 (수직 거리 ≈ closest point 까지 거리)
// ========================================
double computePathErrorCost(const MPCState& state, const ReferencePath& ref,
                            size_t ref_idx, double weight)
{
    if (ref.empty()) return 0.0;
    if (ref_idx >= ref.size()) ref_idx = ref.size() - 1;
    double dx = state.x - ref.x_ref[ref_idx];
    double dy = state.y - ref.y_ref[ref_idx];
    return weight * (dx*dx + dy*dy);
}

// ========================================
// 헤딩 오차
// ========================================
double computeHeadingErrorCost(const MPCState& state, const ReferencePath& ref,
                               size_t ref_idx, double weight)
{
    if (ref.empty()) return 0.0;
    if (ref_idx >= ref.size()) ref_idx = ref.size() - 1;
    double e = angleDiff(state.yaw, ref.yaw_ref[ref_idx]);
    return weight * e * e;
}

// ========================================
// 속도 오차
// ========================================
double computeSpeedErrorCost(const MPCState& state, double v_ref, double weight)
{
    double e = state.vx - v_ref;
    return weight * e * e;
}

// ========================================
// 제어 입력 크기
//   steer/accel 을 분리해서 weight 적용
// ========================================
double computeControlEffortCost(const MPCControl& u, double w_steer, double w_accel)
{
    return w_steer * u.delta * u.delta + w_accel * u.accel * u.accel;
}

// ========================================
// 제어 변화율
//   delta(rad)와 accel(m/s²)은 단위가 달라서 weight 분리
//   같은 weight로 묶으면 gradient 왜곡 → 흔들림/급정거 원인
// ========================================
double computeControlRateCost(const MPCControl& u_prev, const MPCControl& u_cur,
                              double weight_steer, double weight_accel)
{
    double dd = u_cur.delta - u_prev.delta;
    double da = u_cur.accel - u_prev.accel;
    // steer와 acceleration 변화율은 단위와 튜닝 목적이 달라 독립 가중치를 쓴다.
    return weight_steer * dd*dd + weight_accel * da*da;
}

// ========================================
// trajectory 전체 cost
//
// dt 정규화:
//   mpc_params.yaml의 모든 stage 가중치는 kTunedDt(=0.1, 최초 튜닝 시점의 dt)를
//   기준으로 맞춰져 있다. horizon/dt를 바꿔도 물리적으로 같은 궤적이면 cost 균형이
//   유지되도록, "적분류" 항(path/heading/speed/effort)은 dt에 비례, "미분류" 항
//   (control_rate, Δu/Δt의 이산 근사)은 dt에 반비례로 스케일한다.
//   (dt=kTunedDt인 경우 두 스케일 모두 1.0이라 기존 튜닝 값과 완전히 동일하게 동작)
// ========================================
namespace {
constexpr double kTunedDt = 0.1;
}

CostBreakdown computeCostBreakdown(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const MPCParams&               params)
{
    CostBreakdown bd;
    if (states.empty()) return bd;

    size_t N = controls.size();
    size_t S = states.size();
    size_t R = ref.size();

    const double dt_scale   = params.dt / kTunedDt;   // 적분류 stage cost
    const double rate_scale = kTunedDt / params.dt;   // 미분류 rate cost

    // ref는 main.cpp에서 MPC dt로 시간 정렬되며 ref[i]는 i*dt 시점의 목표다.

    // Stage costs (i = 0 .. N-1)
    for (size_t i = 0; i < N; ++i) {
        const MPCState& st = states[i];
        const size_t ref_idx = (R > 0) ? std::min(i, R - 1) : 0;

        // R==0 분기는 solveMPC가 ref.empty()를 미리 걸러서 실질적으로 도달하지 않음.
        double v_target = (R > 0) ? ref.v_ref[ref_idx] : 0.0;
        bd.path    += dt_scale * computePathErrorCost   (st, ref, ref_idx, params.weight_path_error);
        bd.heading += dt_scale * computeHeadingErrorCost(st, ref, ref_idx, params.weight_heading_error);
        bd.speed   += dt_scale * computeSpeedErrorCost  (st, v_target,     params.weight_speed_error);

        const MPCControl& u_cur = controls[i];
        // steer effort: weight_control / accel effort: weight_control * 0.5
        // accel은 속도 추종을 위해 어느 정도 자유롭게 두어야 급정거 방지
        bd.control += dt_scale * computeControlEffortCost(u_cur,
                                          params.weight_control,
                                          params.weight_control * 0.5);

        const MPCControl& u_prev = (i == 0) ? prev_control : controls[i-1];
        bd.control_rate += rate_scale * computeControlRateCost(u_prev, u_cur,
                                          params.weight_control_rate,
                                          params.weight_control_rate_accel);
    }

    // Terminal cost (마지막 stage와 동일한 시간 인덱스: N-1)
    if (S > 0 && R > 0) {
        const MPCState& st = states.back();
        const size_t ref_idx = std::min(N > 0 ? N - 1 : 0, R - 1);

        double v_target = ref.v_ref[ref_idx];
        bd.terminal += computePathErrorCost   (st, ref, ref_idx, params.weight_terminal);
        bd.terminal += computeHeadingErrorCost(st, ref, ref_idx, params.weight_terminal * 0.5);
        bd.terminal += computeSpeedErrorCost  (st, v_target,     params.weight_terminal * 0.2);
    }

    return bd;
}

double computeTotalCost(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const MPCParams&               params)
{
    return computeCostBreakdown(states, controls, ref, prev_control, params).total();
}
