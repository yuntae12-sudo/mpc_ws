#ifndef FRENET_COLLISION_CHECKER_HPP
#define FRENET_COLLISION_CHECKER_HPP

#include "frenet/ref_line.hpp"
#include "global/global.hpp"
#include "math/frenet_converter.hpp"

// =========================================================
// 아래 두 구조체는 논문 Sec.VI의 개념(안전마진, zero/one 판정,
// 시간에 따라 부풀리는 컨투어)을 구체화하기 위해 이 프로젝트가 내린 설계
// 판단이다. 논문은 "얼마나", "어떤 모양으로"는 규정하지 않는다.
// =========================================================

// 자차 충돌판정용 크기. parameter.yaml에서 로드될 예정.
struct VehicleShape {
    double width;   // [m]
    double length;  // [m]
};

// Sec.VI: "add a certain safety distance to the size of our car" +
// "collision-checked contour is continuously expanded... towards the time horizon"
struct CollisionCheckConfig {
    double safety_margin;       // 기본 안전마진 [m] (자차 폭/길이에 사방으로 더해짐)
    double margin_growth_rate;  // 시간에 따라 마진이 커지는 비율 [m/s], margin(t)=safety_margin+rate*t

    // [Sec.VI/Fig.6] "reactive layer"는 장기목표 후보(Tj 격자, 2~5s)와 별개로
    // 항상 최소 이 시간까지는 충돌체크를 한다 (논문 Fig.6 캡션의 "3.0s lookahead").
    // 실측으로 확인된 문제: Tj가 짧은 후보는 자기 T 안에 장애물이 아예 안 들어와서
    // "충돌 없음"으로 잘못 판정되고, 그게 저크/시간비용이 싸서 계속 선택되다가
    // 정작 회피를 시작해야 할 타이밍을 놓친다. reactive_lookahead까지는 후보의
    // 마지막 상태(속도, 횡오프셋)를 그대로 유지("coast")한다고 가정해 검사를
    // 연장함으로써 이 사각지대를 없앤다.
    double reactive_lookahead;  // [s], 논문 기본값 3.0
};

// =========================================================
// 방향 있는 사각형(Oriented Bounding Box) — 자차/장애물 공통 표현
// =========================================================

struct OrientedBox {
    double cx, cy;      // 중심
    double heading;     // 진행방향 [rad]
    double half_length;
    double half_width;
};

// path의 한 샘플(CartesianState)로부터 자차 OBB를 만든다.
// margin은 이미 시간에 따라 커진 값을 넣어준다 (호출측이 CollisionCheckConfig로 계산).
OrientedBox MakeEgoBox(const CartesianState& cs, const VehicleShape& shape, double margin);

// 장애물의 t초 후 위치를 등속 직선 예측(heading 방향으로 speed만큼 이동)해서 OBB를 만든다.
// ObjectInfo에 가속도 필드가 없어 이 이상의 예측은 할 수 없다 (Sec.V-A의 leader 예측과
// 달리, 여기서는 등속만 가정 — TODO(추후 개발 필요): 가속도 정보가 생기면 등가속 예측으로 교체).
OrientedBox MakeObstacleBox(const ObjectInfo& obj, double t, double margin);

// SAT(분리축 정리) 기반 OBB-OBB 겹침 판정.
bool CheckOBBOverlap(const OrientedBox& a, const OrientedBox& b);

// =========================================================
// [Sec.VI] 충돌 필터링 — combined 후보들을 순회하며, 각 시간 샘플에서
// 자차 OBB와 모든 장애물 OBB를 겹침 검사한다. 하나라도 겹치면 그 궤적은
// valid=false 처리 (path_generator의 Filter* 함수들과 동일한 컨벤션).
//
// combined는 이미 FilterByCurvature까지 거친 상태여야 한다. ego box 방향은
// ComputeGeometricPath(위치 기반, s_dot 무관)로 구하므로 저속(STOP/EMERGENCY
// 등 정지에 수렴하는 후보)에서도 정상적으로 판정된다.
//
// TODO(성능, 기존 한계로 기록 — 지금 당장 손대지 않음): 이 함수가 실시간
// 성능의 병목이다. 오프라인 벤치마크(-O2) 결과, params.yaml 기본 격자
// (후보 약 230개) + 장애물 20개 기준으로는 ~11ms(10Hz 예산의 11%)로 여유
// 있지만, 후보 격자를 2배 촘촘히(1460개)+장애물 20개면 ~70ms(70%)까지
// 올라가고, 4배 촘촘(10188개)이면 ~480ms로 10Hz를 완전히 못 맞춘다.
// 후보수 x 장애물수 x 샘플수에 비례해서 늘어나는 구조라서, 원인은 안다.
// 지금 기본값은 안전하지만, 나중에 FSM을 얹으면서 실제 장애물 개수/모드별
// 격자 크기 요구사항이 바뀔 수 있으므로, 그 시점에 실측하면서 broad-phase
// 거리컷 같은 최적화를 같이 넣는 게 지금 미리 손대는 것보다 낫다고 판단함
// (2026-07 오프라인 벤치마크로 확인 후 보류 결정).
// =========================================================

void FilterByCollision(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const std::vector<ObjectInfo>& obstacles,
                        const VehicleShape& ego_shape,
                        const CollisionCheckConfig& cfg);

#endif
