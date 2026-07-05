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
// 헬퍼: 선형 모델로 trajectory 예측
//   x_{k+1} = A_k * x_k + B_k * u_k + c_k
//   predictTrajectory() 대신 사용 → 행렬 곱셈만이라 훨씬 빠름
// ========================================
static std::vector<MPCState> predictTrajectoryLTV(
    const MPCState&                     x0,
    const std::vector<MPCControl>&      U,
    const std::vector<LinearizedModel>& models)
{
    std::vector<MPCState> traj;
    traj.reserve(U.size() + 1);
    traj.push_back(x0);

    double x[4] = { x0.x, x0.y, x0.yaw, x0.vx };

    for (size_t k = 0; k < U.size(); ++k) {
        const auto& lm = models[k];
        double u[2] = { U[k].delta, U[k].accel };

        // x_next = A*x + B*u + c
        double x_next[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; ++i) {
            x_next[i] = lm.c[i];
            for (int j = 0; j < 4; ++j) x_next[i] += lm.A[i][j] * x[j];
            for (int j = 0; j < 2; ++j) x_next[i] += lm.B[i][j] * u[j];
        }

        MPCState st;
        st.x   = x_next[0];
        st.y   = x_next[1];
        st.yaw = normalizeAngle(x_next[2]);
        st.vx  = clip(x_next[3], 0.0, 100.0);
        traj.push_back(st);

        x[0] = st.x; x[1] = st.y; x[2] = st.yaw; x[3] = st.vx;
    }
    return traj;
}

// ========================================
// 헬퍼: cost 평가
//   models 있으면 선형 모델로 trajectory 예측 (빠름)
//   없으면 비선형 predictTrajectory() 사용 (fallback)
// ========================================
static double evalCost(
    const MPCState&                     x0,
    const std::vector<MPCControl>&      U,
    const ReferencePath&                ref,
    const MPCControl&                   prev_control,
    const MPCParams&                    params,
    const std::vector<LinearizedModel>* models   = nullptr,
    std::vector<MPCState>*              out_traj = nullptr)
{
    std::vector<MPCState> traj;
    if (models && models->size() == U.size()) {
        // 선형 모델로 trajectory 예측
        traj = predictTrajectoryLTV(x0, U, *models);
    } else {
        // fallback: 비선형 forward sim
        traj = predictTrajectory(x0, U, params.dt, params.wheelbase,
                                 params.vel_min, params.vel_max);
    }
    double c = computeTotalCost(traj, U, ref, prev_control, params);
    if (out_traj) *out_traj = std::move(traj);
    return c;
}

// ========================================
// 헬퍼: 선형 모델 기반 gradient 계산 (central diff)
//
//   기존과 수식은 동일하지만 evalCost 내부에서
//   predictTrajectory() 대신 행렬 곱(predictTrajectoryLTV)을 사용하므로
//   연산량이 대폭 줄어듦
//
//   기존: evalCost 4N번 호출 → 내부에서 predictTrajectory() 4N번 실행
//   변경: evalCost 4N번 호출 → 내부에서 행렬 곱만 (비선형 적분 없음)
// ========================================
static std::vector<MPCControl> computeGradient(
    const MPCState&                     x0,
    const std::vector<MPCControl>&      U,
    const std::vector<LinearizedModel>& models,
    const ReferencePath&                ref,
    const MPCControl&                   prev_control,
    const MPCParams&                    params)
{
    const int N   = static_cast<int>(U.size());
    const double eps = 1e-3;
    std::vector<MPCControl> grad(N);

    for (int k = 0; k < N; ++k) {
        // d/d_delta
        std::vector<MPCControl> U_p = U, U_m = U;
        U_p[k].delta += eps;
        U_m[k].delta -= eps;
        grad[k].delta = (evalCost(x0, U_p, ref, prev_control, params, &models)
                       - evalCost(x0, U_m, ref, prev_control, params, &models))
                       / (2.0 * eps);

        // d/d_accel
        U_p = U; U_m = U;
        U_p[k].accel += eps;
        U_m[k].accel -= eps;
        grad[k].accel = (evalCost(x0, U_p, ref, prev_control, params, &models)
                       - evalCost(x0, U_m, ref, prev_control, params, &models))
                       / (2.0 * eps);
    }
    return grad;
}

// ========================================
// MPC Solver
//   - LTV 1차 Taylor 선형화 기반 gradient 계산
//   - Projected Gradient Descent
//   - Armijo backtracking line search
// ========================================
MPCResult solveMPC(
    const MPCState&                x0,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const std::vector<MPCControl>& warm_start,
    const MPCParams&               params)
{
    MPCResult result;
    result.success = false;

    if (ref.empty()) {
        result.solver_msg = "Empty reference path";
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
    // 1) Warm-start: 이전 해를 한 스텝 shift, 부족분은 prev_control로
    // ====================================
    std::vector<MPCControl> U(N);
    if (static_cast<int>(warm_start.size()) >= N) {
        for (int i = 0; i < N - 1; ++i) U[i] = warm_start[i + 1];
        U[N - 1] = warm_start[N - 1];
    } else {
        for (int i = 0; i < N; ++i) U[i] = prev_control;
    }
    projectControls(U, prev_control, params);

    // ====================================
    // 2) 현재 U 기반으로 LTV 선형화 모델 계산
    //    warm-start U를 동작점으로 삼아 각 스텝 A_k, B_k, c_k 확보
    //    → 이후 gradient/line search에서 비선형 forward sim 대체
    // ====================================
    std::vector<LinearizedModel> models = buildLTVModels(
        x0, U, params.dt, params.wheelbase, params.vel_min, params.vel_max);

    // ====================================
    // 3) 최적화 루프
    // ====================================
    double cost_cur = evalCost(x0, U, ref, prev_control, params, &models);
    double lr = params.lr_init;

    for (int iter = 0; iter < params.max_iterations; ++iter) {

        // ---- gradient 계산 (선형 모델 기반) ----
        std::vector<MPCControl> grad = computeGradient(
            x0, U, models, ref, prev_control, params);

        // 수렴 체크
        double grad_norm2 = 0.0;
        for (int k = 0; k < N; ++k)
            grad_norm2 += grad[k].delta * grad[k].delta
                        + grad[k].accel * grad[k].accel;
        if (grad_norm2 < params.convergence_eps * params.convergence_eps) break;

        // ---- Armijo backtracking line search ----
        double alpha = lr;
        std::vector<MPCControl> U_new(N);
        double cost_new = cost_cur;
        bool accepted = false;

        for (int ls = 0; ls < params.line_search_steps; ++ls) {
            for (int k = 0; k < N; ++k) {
                double w = 1.0 / (1.0 + 0.1 * k);
                U_new[k].delta = U[k].delta - alpha * w * grad[k].delta;
                U_new[k].accel = U[k].accel - alpha * w * grad[k].accel;
            }
            projectControls(U_new, prev_control, params);

            // U_new 기준으로 선형화 모델 갱신 후 cost 평가
            // U_old 기준 models로 U_new cost 평가 시 선형화 오차 누적 방지
            std::vector<LinearizedModel> models_new = buildLTVModels(
                x0, U_new, params.dt, params.wheelbase, params.vel_min, params.vel_max);
            cost_new = evalCost(x0, U_new, ref, prev_control, params, &models_new);

            if (cost_new < cost_cur - 1e-4 * alpha * grad_norm2) {
                accepted = true;
                models = std::move(models_new);  // accept된 models 반영
                break;
            }
            alpha *= 0.5;
        }

        if (accepted) {
            U = std::move(U_new);
            lr = std::min(params.lr_init, alpha * 1.5);
            // models는 line search에서 이미 U_new 기준으로 갱신 완료

            if (cost_cur - cost_new < params.convergence_eps) {
                cost_cur = cost_new;
                break;
            }
            cost_cur = cost_new;
        } else {
            lr *= 0.5;
            if (lr < params.lr_min) break;
        }
    }

    // ====================================
    // 4) 결과 패키징
    //    최종 trajectory는 비선형 모델로 다시 계산 (근사 오차 제거)
    // ====================================
    std::vector<MPCState> traj;
    evalCost(x0, U, ref, prev_control, params, nullptr, &traj);

    result.controls         = U;
    result.predicted_states = traj;
    result.cost             = cost_cur;
    result.control          = U[0];
    result.success          = true;
    result.solver_msg       = "OK";
    return result;
}