#ifndef MPC_COST_FUNCTION_HPP
#define MPC_COST_FUNCTION_HPP

#include "../global/global.hpp"


// 단일 스텝 비용 (참조 인덱스를 외부에서 결정하여 캐싱)
double computePathErrorCost(const MPCState& state, const ReferencePath& ref,
                            size_t ref_idx, double weight);

double computeHeadingErrorCost(const MPCState& state, const ReferencePath& ref,
                               size_t ref_idx, double weight);

double computeSpeedErrorCost(const MPCState& state, double v_ref, double weight);

double computeControlEffortCost(const MPCControl& u, double weight_steer,
                                 double weight_accel);

double computeControlRateCost(const MPCControl& u_prev, const MPCControl& u_cur,
                              double weight);

// 전체 trajectory 의 cost
double computeTotalCost(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,  // 이전 사이클 마지막 명령 (rate cost용)
    const MPCParams&               params);

// computeTotalCost와 동일한 계산을 항목별로 분리해서 반환 (진단용).
// 급브레이크/급조향 사이클에서 어느 항이 지배적인지 확인하는 데 쓴다.
struct CostBreakdown {
    double path = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double control = 0.0;
    double control_rate = 0.0;
    double terminal = 0.0;
    double total() const { return path + heading + speed + control + control_rate + terminal; }
};

CostBreakdown computeCostBreakdown(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const MPCParams&               params);

#endif // MPC_COST_FUNCTION_HPP
