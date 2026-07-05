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
                              double weight)
{
    double dd = u_cur.delta - u_prev.delta;
    double da = u_cur.accel - u_prev.accel;
    // steer 변화율: weight 그대로 (조향 흔들림 억제)
    // accel 변화율: weight * 0.05 (급정거 억제하되 속도 추종 방해 안 함)
    return weight * dd*dd + (weight * 0.05) * da*da;
}

// ========================================
// trajectory 전체 cost
// ========================================
double computeTotalCost(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const MPCParams&               params)
{
    double total = 0.0;
    if (states.empty()) return 0.0;

    size_t N = controls.size();
    size_t S = states.size();
    size_t R = ref.size();

    size_t ref_idx = 0;

    // Stage costs (i = 0 .. N-1)
    for (size_t i = 0; i < N; ++i) {
        const MPCState& st = states[i];

        // closest ref 탐색 (window 50으로 확장: 고속 주행 시 ref 놓침 방지)
        if (R > 0) {
            double best_d2 = std::numeric_limits<double>::infinity();
            size_t best = ref_idx;
            size_t end = std::min(R, ref_idx + 50);
            for (size_t k = ref_idx; k < end; ++k) {
                double dx = ref.x_ref[k] - st.x;
                double dy = ref.y_ref[k] - st.y;
                double d2 = dx*dx + dy*dy;
                if (d2 < best_d2) { best_d2 = d2; best = k; }
            }
            ref_idx = best;
        }

        double v_target = (R > 0) ? ref.v_ref[ref_idx] : params.target_vel;
        total += computePathErrorCost   (st, ref, ref_idx, params.weight_path_error);
        total += computeHeadingErrorCost(st, ref, ref_idx, params.weight_heading_error);
        total += computeSpeedErrorCost  (st, v_target,     params.weight_speed_error);

        const MPCControl& u_cur = controls[i];
        // steer effort: weight_control / accel effort: weight_control * 0.5
        // accel은 속도 추종을 위해 어느 정도 자유롭게 두어야 급정거 방지
        total += computeControlEffortCost(u_cur,
                                          params.weight_control,
                                          params.weight_control * 0.5);

        const MPCControl& u_prev = (i == 0) ? prev_control : controls[i-1];
        total += computeControlRateCost(u_prev, u_cur, params.weight_control_rate);
    }

    // Terminal cost
    if (S > 0 && R > 0) {
        const MPCState& st = states.back();
        double best_d2 = std::numeric_limits<double>::infinity();
        size_t best = ref_idx;
        size_t end = std::min(R, ref_idx + 50);
        for (size_t k = ref_idx; k < end; ++k) {
            double dx = ref.x_ref[k] - st.x;
            double dy = ref.y_ref[k] - st.y;
            double d2 = dx*dx + dy*dy;
            if (d2 < best_d2) { best_d2 = d2; best = k; }
        }
        ref_idx = best;

        double v_target = ref.v_ref[ref_idx];
        total += computePathErrorCost   (st, ref, ref_idx, params.weight_terminal);
        total += computeHeadingErrorCost(st, ref, ref_idx, params.weight_terminal * 0.5);
        total += computeSpeedErrorCost  (st, v_target,     params.weight_terminal * 0.2);
    }

    return total;
}