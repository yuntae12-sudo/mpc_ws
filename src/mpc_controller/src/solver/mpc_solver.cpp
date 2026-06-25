#include "mpc_solver.hpp"
#include "../global/utils.hpp"
#include "../model/vehicle_model.hpp"
#include "../cost/cost_function.hpp"
#include "../constraints/constraints.hpp"

// ========================================
// 헬퍼: control sequence 전체에 box constraint 투영
// ========================================
static void projectControls(std::vector<MPCControl>& controls,
                            const MPCControl& prev_control,
                            const MPCParams& params)
{
    MPCControl u_prev = prev_control;
    for (auto& u : controls) {
        clipControl(u, params);
        clipSteeringRate(u, u_prev, params.steering_rate_max, params.dt);
        u_prev = u;
    }
}

// ========================================
// 헬퍼: cost 평가 (forward sim + total cost)
// ========================================
static double evalCost(
    const MPCState& x0,
    const std::vector<MPCControl>& U,
    const ReferencePath& ref,
    const CostmapInfo& costmap,
    const MPCControl& prev_control,
    const MPCParams& params,
    std::vector<MPCState>* out_traj = nullptr)
{
    std::vector<MPCState> traj = predictTrajectory(
        x0, U, params.dt, params.wheelbase, params.vel_min, params.vel_max);
    double c = computeTotalCost(traj, U, ref, costmap, prev_control, params);
    if (out_traj) *out_traj = std::move(traj);
    return c;
}

// ========================================
// MPC Solver (Projected Gradient Descent with Armijo line search)
// ========================================
MPCResult solveMPC(
    const MPCState&                x0,
    const ReferencePath&           ref,
    const CostmapInfo&             costmap,
    const MPCControl&              prev_control,
    const std::vector<MPCControl>& warm_start,
    const MPCParams&               params)
{
    MPCResult result;
    result.success = false;

    if (ref.empty()) {
        result.solver_msg = "Empty reference path";
        // 안전 명령: 부드러운 감속, 조향 0
        result.control.delta = 0.0;
        result.control.accel = std::max(params.accel_min, -1.0);
        return result;
    }

    const int N = params.horizon;
    if (N <= 0) {
        result.solver_msg = "Invalid horizon";
        return result;
    }

    // ====================================
    // 1) Warm-start: 이전 해를 한 스텝 shift, 부족분은 0으로
    // ====================================
    std::vector<MPCControl> U(N);
    if (static_cast<int>(warm_start.size()) >= N) {
        for (int i = 0; i < N - 1; ++i) U[i] = warm_start[i + 1];
        U[N - 1] = warm_start[N - 1];  // 마지막은 복제
    } else {
        for (int i = 0; i < N; ++i) {
            //U[i].delta = 0.0;
            //U[i].accel = 0.0;
            U[i]=prev_control;
        }
    }
    projectControls(U, prev_control, params);

    // ====================================
    // 2) 최적화 루프
    // ====================================
    double cost_cur = evalCost(x0, U, ref, costmap, prev_control, params);
    double lr = params.lr_init;
    const double eps = 1e-3;  // gradient stencil

    for (int iter = 0; iter < params.max_iterations; ++iter) {
        // ---- 그래디언트 (central diff) ----
        std::vector<MPCControl> grad(N);
        for (int k = 0; k < N; ++k) {
            // d/d_delta
            U[k].delta += eps;
            double c_p = evalCost(x0, U, ref, costmap, prev_control, params);
            U[k].delta -= 2.0 * eps;
            double c_m = evalCost(x0, U, ref, costmap, prev_control, params);
            U[k].delta += eps;
            grad[k].delta = (c_p - c_m) / (2.0 * eps);

            // d/d_accel
            U[k].accel += eps;
            c_p = evalCost(x0, U, ref, costmap, prev_control, params);
            U[k].accel -= 2.0 * eps;
            c_m = evalCost(x0, U, ref, costmap, prev_control, params);
            U[k].accel += eps;
            grad[k].accel = (c_p - c_m) / (2.0 * eps);
        }

        // 그래디언트 노름 (수렴 체크용)
        double grad_norm2 = 0.0;
        for (int k = 0; k < N; ++k)
            grad_norm2 += grad[k].delta * grad[k].delta + grad[k].accel * grad[k].accel;
        if (grad_norm2 < params.convergence_eps * params.convergence_eps) {
            break;  // 충분히 수렴
        }

        // ---- Armijo backtracking line search ----
        double alpha = lr;
        std::vector<MPCControl> U_new(N);
        double cost_new = cost_cur;
        bool accepted = false;
        for (int ls = 0; ls < params.line_search_steps; ++ls) {
            // 후보 업데이트
            for (int k = 0; k < N; ++k) {
                // 초기 step 일수록 더 크게 적용 (시간 중요도)
                double w = 1.0 / (1.0 + 0.1 * k);
                U_new[k].delta = U[k].delta - alpha * w * grad[k].delta;
                U_new[k].accel = U[k].accel - alpha * w * grad[k].accel;
            }
            projectControls(U_new, prev_control, params);
            cost_new = evalCost(x0, U_new, ref, costmap, prev_control, params);

            // Armijo: cost 가 충분히 작아졌으면 accept
            if (cost_new < cost_cur - 1e-4 * alpha * grad_norm2) {
                accepted = true;
                break;
            }
            alpha *= 0.5;
        }

        if (accepted) {
            U = std::move(U_new);
            // step 정확도 향상에 따라 lr 소폭 증가
            lr = std::min(params.lr_init, alpha * 1.5);
            // 개선이 미미하면 종료
            if (cost_cur - cost_new < params.convergence_eps) {
                cost_cur = cost_new;
                break;
            }
            cost_cur = cost_new;
        } else {
            // 어떤 alpha 로도 cost 못 줄임 → lr 더 줄이고 한 번 더
            lr *= 0.5;
            if (lr < params.lr_min) break;
        }
    }

    // ====================================
    // 3) 결과 패키징
    // ====================================
    std::vector<MPCState> traj;
    double final_cost = evalCost(x0, U, ref, costmap, prev_control, params, &traj);

    result.controls         = U;
    result.predicted_states = traj;
    result.cost             = final_cost;
    result.control          = U[0];
    result.success          = true;
    result.solver_msg       = "OK";
    return result;
}
