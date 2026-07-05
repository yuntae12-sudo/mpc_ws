#ifndef MPC_SOLVER_HPP
#define MPC_SOLVER_HPP

#include "../global/global.hpp"

/**
 * @brief MPC 최적화 솔버
 *   - warm-start (이전 해의 한 스텝 shift 사용)
 *   - central-difference gradient
 *   - Armijo backtracking line search
 *   - constraint projection (box clip)
 *
 * @param ego_state     현재 상태 (global frame)
 * @param ref           reference path (현재 위치 주변 window)
 * @param prev_control  직전 사이클에 발행한 제어 (rate cost 의 시작점)
 * @param warm_start    이전 사이클의 최적 control sequence (없으면 비움)
 * @param params        MPC 파라미터
 * @return MPC 결과
 */
MPCResult solveMPC(
    const MPCState&                ego_state,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const std::vector<MPCControl>& warm_start,
    const MPCParams&               params);

#endif // MPC_SOLVER_HPP