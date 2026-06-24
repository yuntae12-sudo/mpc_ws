#include "cost_function.hpp"
#include "../Global/math_utils.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

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
    double e = math_utils::angleDiff(state.yaw, ref.yaw_ref[ref_idx]);
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
// 제어 입력 크기 (가중치는 조향/가속 따로 적용해도 OK; 여기선 같은 가중치)
// ========================================
double computeControlEffortCost(const MPCControl& u, double w_steer, double w_accel)
{
    return w_steer * u.delta * u.delta + w_accel * u.accel * u.accel;
}

// ========================================
// 제어 변화율
// ========================================
double computeControlRateCost(const MPCControl& u_prev, const MPCControl& u_cur,
                              double weight)
{
    double dd = u_cur.delta - u_prev.delta;
    double da = u_cur.accel - u_prev.accel;
    return weight * (dd*dd + da*da);
}

// ========================================
// 장애물 (costmap 0..100 사용, lethal 임계 위는 큰 페널티)
// ========================================
double computeObstacleCost(const MPCState& state,
                           const CostmapInfo& costmap,
                           double weight,
                           double lethal_threshold)
{
    if (!costmap.msg || costmap.resolution <= 1e-9 ||
        costmap.width <= 0 || costmap.height <= 0)
        return 0.0;

    int gx = static_cast<int>(std::floor((state.x - costmap.origin_x) / costmap.resolution));
    int gy = static_cast<int>(std::floor((state.y - costmap.origin_y) / costmap.resolution));
    if (gx < 0 || gx >= costmap.width || gy < 0 || gy >= costmap.height)
        return 0.0;

    int idx = gy * costmap.width + gx;
    if (idx < 0 || idx >= static_cast<int>(costmap.msg->data.size()))
        return 0.0;

    int8_t raw = costmap.msg->data[idx];
    double v;
    if (raw < 0) v = 30.0;          // unknown
    else         v = static_cast<double>(raw);
    if (v < 0.0) v = 0.0;
    if (v > 100.0) v = 100.0;

    // 정상 영역은 부드러운 quadratic, lethal 위는 큰 페널티
    double norm = v / 100.0;          // 0..1
    double soft = norm * norm;
    double hard = (v >= lethal_threshold) ? 100.0 * (v - lethal_threshold + 10.0) : 0.0;
    return weight * (soft + hard);
}

// ========================================
// trajectory 전체 cost
// ========================================
double computeTotalCost(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const CostmapInfo&             costmap,
    const MPCControl&              prev_control,
    const MPCParams&               params)
{
    double total = 0.0;
    if (states.empty()) return 0.0;

    size_t N = controls.size();    // stage cost loop 길이
    size_t S = states.size();      // = N + 1 (terminal 포함)
    size_t R = ref.size();

    // 각 stage 마다 reference 인덱스를 찾는다. 단조 증가 가정.
    size_t ref_idx = 0;

    //if(R>0) {
    //    double best_d2=std::numeric_limits<double>::infinity();
    //    size_t best=0;
    //    for (size_t k=0; k<R;++k) {
    //        double dx =ref.x_ref[k]-states[0].x;
    //        double dy = ref.y_ref[k]-states[0].y;
    //        double d2 = dx*dx+dy*dy;
    //        if(d2<best_d2){
    //            best_d2=d2;
    //            bets=k;
    //        }
    //    }
    //    ref_idx=best;
    //}

    // Stage costs (i = 0 .. N-1)
    for (size_t i = 0; i < N; ++i) {
        const MPCState& st = states[i];

        // closest ref (windowed: start from previous ref_idx)
        if (R > 0) {
            double best_d2 = std::numeric_limits<double>::infinity();
            size_t best = ref_idx;
            // 앞쪽으로만 검색 (역행 방지)
            size_t end = std::min(R, ref_idx + 30);
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
        total += computeObstacleCost    (st, costmap, params.weight_obstacle, params.lethal_cost_threshold);

        // control effort + rate
        const MPCControl& u_cur = controls[i];
        // 조향에 더 큰 weight 를 (안정성을 위해)
        total += computeControlEffortCost(u_cur, params.weight_control, params.weight_control * 0.1);

        const MPCControl& u_prev = (i == 0) ? prev_control : controls[i-1];
        total += computeControlRateCost(u_prev, u_cur, params.weight_control_rate);
    }

    // Terminal cost (마지막 예측 상태 s_N)
    if (S > 0 && R > 0) {
        const MPCState& st = states.back();
        // 마지막 reference 인덱스도 앞으로 진행
        double best_d2 = std::numeric_limits<double>::infinity();
        size_t best = ref_idx;
        size_t end = std::min(R, ref_idx + 30);
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
        total += computeObstacleCost    (st, costmap, params.weight_obstacle, params.lethal_cost_threshold);
    }

    return total;
}
