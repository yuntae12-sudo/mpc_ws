#ifndef PLANNER_MATH_FRENET_CONVERTER_HPP
#define PLANNER_MATH_FRENET_CONVERTER_HPP

#include "frenet/ref_line.hpp"
#include "frenet/frenet_types.hpp"

// =========================================================
// 한 시점(instant)의 Cartesian 상태.
// CartesianPath(frenet_types.hpp)는 시간 배열(vector)이라
// 매 샘플마다 변환할 때는 이 단일 상태 구조체를 쓰고,
// path_generator가 이걸 CartesianPath의 각 vector에 push_back 한다.
// 필드명은 CartesianPath와 맞춰 x, y, yaw, kappa, v, a로 통일.
// =========================================================

struct CartesianState {
    double x, y;
    double yaw;     // theta_x
    double kappa;   // kappa_x
    double v;       // v_x
    double a;       // a_x
};

// =========================================================
// [핵심 변환] Frenet -> Cartesian
// 논문 Appendix I 식 (1), (6), (9)를 이용한 변환.
//
// d, d', d''는 arc-length s에 대한 미분 (Sec.IV-B, 저속 모드와 동일한 정의)
// 이 형태를 "핵심"으로 삼는 이유: vx = 0 (정지/저속)에서도 특이점 없이
// 항상 계산 가능하기 때문 (논문이 App.I 서두에서 강조하는 부분).
//
// rp: FindClosestS + Interpolate로 얻은 center line 상의 투영점
//     (theta_r, kappa_r, d_kappa_r 포함)
// s, s_dot, s_ddot: 종방향 상태 (투영점의 arc length 및 그 시간 미분)
// d, d_prime, d_pprime: d(s), d'(s), d''(s)
//
// 반환되는 CartesianState.v는 식(7)의 v_x = s_dot*(1-kappa_r*d)/cos(dtheta)
// 로 계산되며, s_dot = 0이면 v = 0으로 처리되어 정지 상태도 안전하게 처리됨.
// =========================================================

CartesianState FrenetToCartesian(const RefPoint& rp,
                                  double s, double s_dot, double s_ddot,
                                  double d, double d_prime, double d_pprime);

// =========================================================
// [고속 어댑터] (d, d_dot, d_ddot) -> (d, d', d'')
// 논문 App.I 마지막 문단: d' = d_dot / s_dot,  d'' = (d_ddot - d'*s_ddot) / s_dot^2
// s_dot != 0 인 고속 구간에서만 유효 (path_generator가 Sec.IV-A 방식,
// 즉 d(t) 다항식으로 후보를 생성했을 때 이 함수를 거쳐 핵심 변환으로 넘긴다).
//
// 호출 측에서 반드시 |s_dot| 이 충분히 큰지 확인 후 호출할 것
// (0 근방이면 저속 모드로 전환해서 d(s) 다항식을 직접 써야 함, Sec.IV-B).
// =========================================================

void TimeDerivToArcDeriv(double s_dot, double s_ddot,
                          double d_dot, double d_ddot,
                          double& d_prime, double& d_pprime);

// =========================================================
// [고속 어댑터의 역방향] (d, d', d'') -> (d_dot, d_ddot)
// 논문 App.I: d_dot = s_dot * d',  d_ddot = d''*s_dot^2 + d'*s_ddot
// =========================================================

void ArcDerivToTimeDeriv(double s_dot, double s_ddot,
                          double d_prime, double d_pprime,
                          double& d_dot, double& d_ddot);

// =========================================================
// [역변환] Cartesian -> Frenet
// 논문 Sec.VI 마지막 문단: "매번 새 center line을 참조할 때마다
// 현재 끝점 (x, theta_x, kappa_x, v_x, a_x)을 새 center line에
// 투영해서 [s, s_dot, s_ddot, d, d', d'']를 구해야 한다."
//
// 내부에서 FindClosestS로 s0을 구하고, 식(4),(6),(9)를 theta_x, kappa_x에
// 대해 역으로 풀어 d, d', d''를 얻는다. s_dot, s_ddot은 식(7)과 그 시간
// 미분을 v_x, a_x에 대해 풀어서 얻는다 (v_x = 0이면 s_dot = 0, s_ddot은
// 저속 특이점이라 App.I에 따로 명시된 바 없어 0으로 둔다 — 구현 시 논의 필요).
//
// 출력 파라미터로 s, s_dot, s_ddot, d, d_prime, d_pprime을 모두 채운다.
// 고속으로 계속 주행 중이면 TimeDerivToArcDeriv의 역인 ArcDerivToTimeDeriv를
// 이미 내부에서 적용해 d_dot, d_ddot도 함께 필요하면 오버로드를 추가할 것.

// 전제: 우선은 고속 모드 전용으로 명시하고 개발하다가 추후에 수정해야함
// =========================================================

void CartesianToFrenet(const RefLine& ref, const CartesianState& cs,
                        double& s, double& s_dot, double& s_ddot,
                        double& d, double& d_prime, double& d_pprime);

// =========================================================
// [배치 변환] FrenetPath 전체 -> CartesianPath
//
// 왜 필요한가:
//   cost.hpp의 SelectBestPath()가 최종적으로 골라주는 건 "가장 비용이 낮은
//   FrenetPath" 하나인데, 이건 여전히 Frenet 좌표(t, d, d_d, d_dd, s, s_d, s_dd
//   의 시간 샘플 배열)로 표현돼 있다. 실제로 트래킹/MPC 컨트롤러에 넘겨줄
//   최종 산출물은 frenet_types.hpp에 이미 정의된 CartesianPath(x, y, yaw,
//   kappa, v, a 배열)인데, 지금까지는 이걸 "채워서 반환"하는 함수가 없었다.
//   즉 비용 계산·충돌 체크·최적 궤적 선택까지 다 해놓고도, 그 결과를 밖으로
//   내보낼 방법이 없는 상태였다 — 이 함수가 그 마지막 연결고리다.
//
// 왜 새 로직이 아니라 "재사용"인가:
//   한 시점(sample)을 Frenet -> Cartesian으로 바꾸는 절차는
//     ① TimeDerivToArcDeriv로 (d_dot, d_ddot) -> (d_prime, d_pprime) 변환
//        (path_generator가 고속 모드(d(t))로 후보를 만들었으므로 항상 필요)
//     ② Interpolate(ref, s)로 그 위치의 center line 정보(RefPoint) 조회
//     ③ FrenetToCartesian(rp, ...)으로 실제 (x,y,yaw,kappa,v,a) 계산
//   이 세 단계는 사실 collision_checker.cpp의 FilterByCollision과
//   path_generator.cpp의 FilterByCurvature 안에 이미 존재한다. 다만 그
//   두 함수는 계산한 CartesianState를 "조건 검사(곡률/충돌)에만 쓰고
//   버린다". 이 함수는 똑같은 세 단계를 그대로 수행하면서, 버리지 않고
//   CartesianPath의 각 vector(x,y,yaw,kappa,v,a)에 순서대로 쌓아서
//   돌려주는 것뿐이다 — 새로운 변환 공식이 추가되는 게 아니라, 이미
//   두 번 쓰인 패턴을 "값을 보존하는 버전"으로 한 번 더 쓰는 것.
//
// 전제(기존 함수들과 동일한 제약을 그대로 물려받음):
//   - path는 이미 FilterByCurvature/FilterByCollision을 통과한
//     valid==true 상태여야 함직하다 (호출 전 보장은 호출 측 책임).
//   - TimeDerivToArcDeriv가 고속 모드 전용이므로, path.s_d[i]가 0에
//     가까운 샘플이 섞여 있으면 그 지점에서 변환이 불안정해진다.
//     TODO(추후 개발 필요, FSM 저속 처리와 함께): 저속 샘플 처리 정책은
//     FilterByCurvature/FilterByCollision과 동일하게 미해결로 남아있음.
// =========================================================

CartesianPath ConvertToCartesianPath(const FrenetPath& path, const RefLine& ref);

#endif
