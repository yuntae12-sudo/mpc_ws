# Planner 3-패키지 통합 아키텍처 (v4 — 열린 결정사항 코드로 검증 완료)

## Context

`behavior_planner`(MGeo 기반 상위 의사결정), `planner`(Frenet 후보경로 생성기), `mpc_controller`(MPC 경로추종)는 각자 독립적으로 동작한다. 설계상 의도된 연결점이 세 개 있는데 전부 미완성이다:

1. `behavior_planner`가 퍼블리시하는 `/behavior/context`를 아무도 구독하지 않음 → `planner`는 항상 `LANE_KEEPING` 고정값으로 동작 (`planner/src/main.cpp:143-145`)
2. `behavior_planner`가 구독을 걸어놓고 기다리는 `/planner/plan_feedback`을 아무도 퍼블리시하지 않음 (`behavior_planner/src/node/behavior_node.cpp:59-60`)
3. `planner`가 퍼블리시하는 `/frenet_planner/trajectory`를 `mpc_controller`가 구독하지 않음

**이 계획은 `behavior_planner` 팀의 공식 핸드오프 문서(`behavior_planner_frenet_handoff_guide.pdf`)를 유일한 기준(source of truth)으로 따른다.** 아래 모든 절은 문서의 몇 절을 근거로 하는지 명시한다. 문서와 어긋나는 판단은 전부 없앴고, 문서가 명시적으로 "V1에서 기대하면 안 되는 것"으로 선언한 항목은 단순화하되 그 사실을 숨기지 않고 계획에 그대로 남긴다(문서 "아직 기대하면 안되는 것" 절, 문서 11절 "현재 한계와 다음 개선 순서").

## 전체 데이터 흐름 (문서 1.1/1.2절 그대로)

```
behavior_planner                 planner (frenet_planner_node)          mpc_controller (mpc_node)
─────────────────                ──────────────────────────────         ─────────────────────────
/behavior/context      ────────▶ CBBehaviorContext
(BehaviorContext.msg)             → BuildCommandFromContext()
                                    → PlannerCommand (기존 하드코딩 대체)
                                  PlanningLoop (기존 파이프라인 그대로)
                                    → best FrenetPath
/planner/plan_feedback ◀──────── BuildFeedback(best, cmd, ctx)
(PlanFeedback.msg)
                                  /frenet_planner/trajectory  ────────▶ CBTrajectory
                                  (Float32MultiArray, 기존 그대로)        → ReferencePath 변환
                                                                         controlLoop: 신선하면 이걸 쓰고
                                                                         아니면 기존 CSV(path.txt) fallback
```

핵심 원칙 (문서 "중요: trajectory 생성과 control command 발행은 하지 않음. Frenet/MPC는 downstream"): **`behavior_planner` 쪽은 코드 변경 없음.** 문서 4.1/4.2절이 정의한 계약만 `planner`가 올바르게 해석해서 소비하고, 스키마에 맞는 `PlanFeedback`을 퍼블리시하면 된다.

## 참고: `planner`의 참조선(RefLine)은 이미 문서 취지와 맞음

문서 2절("Frenet 은 target/current link 중심선을 reference 후보로 사용하면 된다")은 Frenet의 기준선이 MGeo 링크 중심선 기반이어야 한다고 전제한다. `planner`는 지금 MGeo를 직접 읽지 않지만, 이번 세션에서 `path_route1.txt`를 `route_links_1.yaml`(MGeo `link_set.json` 중심선 그대로 이어붙임)로 만들었기 때문에 **현재 주행 중인 route(=current/route_required link 체인)에 대해서는 이미 문서 취지를 만족한다.** 남는 간극은 딱 하나, "차선변경 시 인접 차선(target_link_id)의 중심선"뿐이고, 이건 아래 1.3에서 다룬다.

## 1. `behavior_planner` → `planner`: BehaviorContext 수신 (문서 4.1~4.3절)

**새 파일**: `planner/src/global/behavior_bridge.hpp` / `.cpp`

```cpp
PlannerCommand BuildCommandFromContext(const behavior_planner::BehaviorContext& ctx,
                                        const FrenetState& ego_start);
```

### 1.1 모드 매핑 (문서 4.2절 `selected_behavior` 해석 + 4.3절 모드별 처리 가이드)

문서 4.2절은 "Frenet 이 반드시 읽을 값"에 `front_gap`/`front_relative_speed`/leader 운동학을 포함하지 않는다 — 그 값들은 `/behavior/feature_debug`(디버그 전용)에만 있다. `behavior_planner`가 이미 gap을 반영해 `desired_speed`를 낮춰서 넘겨주는 게 계약이므로, `planner`가 스스로 앞차 운동학을 역산하지 않는다.

| BehaviorContext.selected_behavior | PlannerCommand.mode | 근거(문서 4.3절) |
|---|---|---|
| KEEP(0) | LANE_KEEPING, `target_speed=min(desired_speed,max_speed)` | "기본 lane keeping 후보 생성" |
| FOLLOW(1) | LANE_KEEPING(VelocityKeeping 그대로), `target_speed=min(desired_speed,max_speed)` | "속도 프로파일을 desired_speed 이하로 제한" — 뒤 "비용 강화"는 1.1.1 참고 |
| LEFT_CHANGE(2) | LANE_CHANGE_LEFT, target_lane=-1 | "target link 중심선 또는 d target 을 향한 후보 생성" — 1.3 참고 |
| RIGHT_CHANGE(3) | LANE_CHANGE_RIGHT, target_lane=+1 | "좌측과 동일하되 우측 target" |
| STOP(4) | STOP, `stop_position=ctx.stop_before_s` | "stop_before_s 를 hard longitudinal constraint 로 사용" |
| EMERGENCY_STOP(6) | EMERGENCY | "다른 후보보다 안전 정지 우선" — 1.1.2 참고 |
| YIELD(5) | STOP (방어적 기본값) | 문서 4.2절 `selected_behavior` 해석표에 값 5가 없음 → V1에서 안 오는 값으로 간주 |

`desired_speed`는 STOP/EMERGENCY에서는 안 읽는다 — 문서도 "STOP/E-STOP은 desired_speed=0"이라 명시하는데, `planner`의 `GenerateStoppingCandidates`가 이미 종료속도를 0으로 강제하는 수식(Sec.V-A)이라 필드를 다시 읽을 필요가 없다(중복 소스 방지). `max_speed` 클램프는 표에 있는 모든 모드(KEEP/FOLLOW/LEFT/RIGHT_CHANGE)의 `target_speed`에 동일하게 적용한다 — "속도 후보 상한"이라는 문서 문구가 특정 모드 한정이 아니기 때문.

`behavior_state`(더 세분화된 FSM 상태)는 안 읽는다 — 문서: "V1에서는 PREPARE/STOP/E-STOP 정도로 사용, EXECUTE 전이는 아직 미구현"이라, `selected_behavior`보다 더 얻을 정보가 없음.

#### 1.1.1 FOLLOW의 "앞차 후방 비용 강화" — 미해결로 남김

문서 4.3절 FOLLOW 행: "속도 프로파일을 desired_speed 이하로 제한하고 **앞차 후방 비용 강화**". 뒤쪽 절반(비용 강화)은 지금 `planner`의 `cost.hpp/cpp`에 대응하는 게 없다 — `EvaluateCosts`는 jerk/시간/종료오프셋만 계산하고, 장애물과의 거리를 비용에 넣는 항이 없다(`FilterByCollision`은 통과/무효 이진 판정일 뿐, 비용이 아님). 이번 통합 스코프에서는 속도 캡 + 기존 이진 충돌 필터로 근사하고, "앞차 후방 비용" 자체를 코스트 함수에 추가하는 건 **하지 않는다** — `cost.hpp`에 새 가중치/항을 넣는 별도 작업으로 명시적으로 남겨둔다(숨기지 않음).

#### 1.1.2 EMERGENCY_STOP의 코스트 처리 — 결정 필요

문서: "다른 후보보다 안전 정지 우선. 필요하면 바로 감속 프로파일 생성." 그런데 지금 `SelectBestPath`는 모드와 무관하게 동일한 `CostWeights`(승차감 중심: jerk+시간)로 후보를 고른다 — EMERGENCY 상황에서 승차감 최적 후보를 고르는 게 "안전 정지 우선"과 안 맞을 수 있다. 두 옵션:
- (a) EMERGENCY 전용 `CostWeights`를 하나 더 두고 `kt`(시간비용)를 크게 키워 가장 빨리 정지하는 후보가 이기도록 함
- (b) `SelectBestPath`를 아예 안 쓰고, EMERGENCY일 때는 valid한 후보 중 `stop_position`에 가장 빨리 도달하는 것을 직접 선택

**구현 직전 결정** — 이번 계획에서는 (a)를 기본으로 제안(기존 파이프라인 재사용 범위가 더 커서 리스크 작음).

### 1.2 방어적 검증 (문서 4.2절 "enable=false 후보는 만들거나 선택하지 않는 것이 안전함")

문서 문구가 `enable_keep/follow/left/right/stop/e_stop` 전체를 포괄하므로, 매핑된 모드 전부에 대해 대응하는 `enable_*`를 확인한다(일부만 확인하지 않음):

- `KEEP`→`enable_keep`, `FOLLOW`→`enable_follow`, `LEFT_CHANGE`→`enable_left_change`, `RIGHT_CHANGE`→`enable_right_change`, `STOP`→`enable_stop`, `EMERGENCY_STOP`→`enable_e_stop`
- 각각 해당 `enable_*`가 false인데 `selected_behavior`가 그 모드를 가리키면 → `LANE_KEEPING`으로 강제 폴백 + `ROS_WARN` (behavior_planner의 점수 로직상 정상 상황에서는 안 일어나야 하지만, stale message나 레이스 컨디션에 대한 방어)
- `LEFT_CHANGE`/`RIGHT_CHANGE`는 추가로 `forbid_left/right_change`, `forbid_lane_change`도 확인
- `ctx.force_stop || ctx.emergency_stop`이면 `selected_behavior`와 무관하게 각각 STOP/EMERGENCY를 강제 (문서: "force_stop=true 또는 selected_behavior=STOP 이면...")

### 1.3 차선변경 목표(target_lane) — 문서가 명시적으로 허용하는 단순화

문서 4.3절 LEFT_CHANGE 행이 "target link 중심선 **또는** d target 을 향한 후보 생성"이라고 되어 있어, **d-target(횡오프셋) 방식이 문서에서 이미 명시적으로 허용된 옵션**이다 — 임시 타협이 아니라 정식으로 인정된 두 방법 중 하나를 고르는 것.

`planner`는 MGeo를 안 읽으므로 "target_link_id 중심선" 방식은 MGeo 접근을 새로 붙여야 하는 더 큰 스코프다. 이번엔 d-target 방식을 선택: `path_generator.cpp`의 `ResolveLateralOffset`에 이미 있는 TODO("`lane_width` 설정값이 config에 추가되면 target_lane * lane_width로 계산")를 구현 — `params.yaml`에 `lane_width` 추가, `target_lane`(-1/+1) × `lane_width`를 목표 d 오프셋으로 사용.

**`lane_width` 기본값 확인 완료**: `link_set.json`의 634개 링크 전체가 예외 없이 `width_start=width_end=3.5`. PDF의 "차폭 약 3.5m 근사"는 그림상 근사가 아니라 지도 데이터 자체가 균일한 값 — 링크별로 다르게 읽을 필요 없이 `lane_width: 3.5`로 하드코딩 확정.

`target_link_id`/`candidate_link_ids`는 이번 판단 로직에는 안 쓴다 — `PlanFeedback`에서 `target_link_id`를 되돌려 보내는 용도로만 사용(2번). MGeo 중심선 기반의 더 정밀한 버전은 다음 개선 항목으로 명시.

### 1.4 EMERGENCY_STOP의 `stop_before_s` 유효성 — 확인 완료, fallback 필수로 확정

`hard_rule_filter.cpp` 확인 결과: `stop_before_s`는 신호/정지선 블록(`f.signal.has_stop_line && ...`)에서만 설정되고, TTC 기반 emergency 블록(`f.risk.emergency_risk || f.risk.min_ttc<1.0`)은 `stop_before_s`를 전혀 안 건드린다. `HardConstraint::stop_before_s`의 기본값은 `1e9`(behavior_context.hpp:7) — 정지선 없이 TTC만으로 EMERGENCY_STOP이 뜨면 `stop_before_s`는 sentinel `1e9`로 남는다.

→ **fallback은 선택이 아니라 필수**: `planner`는 `ctx.stop_before_s >= 1e8`(sentinel 판정)이면 `stop_position = ego_start.s + kEmergencyStopBuffer`(파라미터화된 최소 정지거리 상수, 예 10~15m)로 자체 계산. 그 외(신호 기반으로 정상 세팅된 경우)에는 `ctx.stop_before_s` 그대로 사용.

### 1.5 Staleness/fallback

`main.cpp`에 `g_behavior_ctx`, `g_behavior_received`, 수신 타임스탬프 추가. `CBBehaviorContext`는 저장만 하고, `PlanningLoop`은 (a) 한 번도 못 받았거나 (b) 마지막 수신이 임계치(예 0.5s)보다 오래됐으면 기존처럼 `LANE_KEEPING` 하드코딩으로 폴백.

**빌드 의존성**: `planner/CMakeLists.txt`, `planner/package.xml`에 `behavior_planner`를 catkin 컴포넌트로 추가 (메시지 헤더만 필요).

## 2. `planner` → `behavior_planner`: PlanFeedback 발행 (문서 4.1절 + "통합 주의점")

같은 `behavior_bridge.hpp/.cpp`에 추가. 문서 "통합 주의점": "현재 PlanFeedback 은 subscribe 하지만 Behavior Decision 에 실질적으로 반영되지 않는다"(그쪽 개선 과제, 문서 11절 우선순위 3) — 즉 필드 정밀도보다 "스키마를 맞춰서 끊기지 않게 발행"이 우선이라 v1 단순화가 안전하다.

```cpp
behavior_planner::PlanFeedback BuildFeedback(const PlannerCommand& cmd_used,
                                              const FrenetPath* best,
                                              const behavior_planner::BehaviorContext& ctx);
```

- `plan_valid` = `best != nullptr`
- `selected_behavior` = `cmd_used.mode`를 1.1 표의 역방향으로 매핑
- `selected_target_link_id` = `plan_valid ? ctx.target_link_id : ""`
- `selected_cost` = `best ? best->cost_total : 0.0`
- `min_collision_margin` = `1e9`(미측정 sentinel) 고정 — `FilterByCollision`이 지금 pass/fail만 반환하고 실제 여유거리를 안 남김. 의미 있는 값을 채우려면 `collision_checker`에 최소 여유거리 기록하는 작은 확장이 별도로 필요(다음 작업).
- `previous_plan_still_valid = plan_valid`(근사), `lane_change_completed = false`, `lane_change_progress = 0.0` — `lane_width` 구현 이후 재검토.

`main.cpp`의 `PlanningLoop` 끝에 퍼블리셔 추가, 토픽 `/planner/plan_feedback`.

## 3. `planner` → `mpc_controller`: trajectory 구독

(문서가 이 구간을 직접 다루지 않음 — 문서 1.1절 "MPC Controller: 선택 trajectory 를 추종" 원칙만 참고, 나머지는 기존 설계 유지)

**mpc_controller 쪽 변경 파일**: `src/node/mpc_node.cpp`/`.hpp`, `src/planner/path_planner.hpp`/`.cpp`.

- 새 콜백 `CBExternalTrajectory`: `planner`의 `/frenet_planner/trajectory` 레이아웃(`[n, x[n], y[n], yaw[n], kappa[n], v[n], a[n]]`, `planner/src/main.cpp:90-101`)을 파싱해 `ReferencePath{x_ref,y_ref,yaw_ref,v_ref,k_ref}`로 채움.
- 새 함수 `BuildReferenceFromExternalTrajectory(...)` (`path_planner.cpp`).
- `controlLoop()` 수정: 외부 trajectory가 신선하면 그걸 쓰고, 아니면 기존 CSV(`path.txt`) 폴백.

## 열린 결정/확인 사항 — 전부 확정됨

1. **EMERGENCY_STOP의 `stop_before_s` 유효성** — 확인 완료(1.4): sentinel `1e9`로 남는 케이스가 실제로 있어 fallback 필수로 확정.
2. **EMERGENCY 전용 코스트 가중치 도입 여부** — (a) 채택 확정(1.1.2): 기존 파이프라인 재사용 범위가 커서 리스크가 작다.
3. **`lane_width` 기본값** — 확인 완료(1.3): `link_set.json` 전체가 균일하게 3.5m, 하드코딩 확정.
4. **YIELD(5)** — 확인 완료: `BehaviorScore`에 `yield` 필드가 없고 `behavior_decision.cpp`의 6개 후보 배열에도 없어, 현재 코드로는 절대 선택되지 않는다. STOP-폴백 매핑은 방어코드로 유지.
5. **FOLLOW "앞차 후방 비용 강화"** — 스코프 제외 확정(1.1.1): `cost.hpp` 확장은 별도 작업으로 분리.

## 구현 순서 제안

1. **`planner` ↔ `behavior_planner` 브릿지** (1,2번) — `behavior_planner` 코드 변경 0, `planner` 안에서 자기완결적.
2. **`planner` → `mpc_controller`** (3번) — 실시간 제어 루프를 건드리므로 1번 안정화 후 진행.

## 검증 방법 (문서 7~9절 그대로 채용)

- `rostopic hz /Ego_topic`, `/Object_topic`으로 입력 확인 → `rostopic echo /behavior/context`로 Context 정상 여부 확인 → `planner` 로그/`/frenet_planner/markers`(rviz)로 실제 후보 모드 전환 확인 → `rostopic echo /planner/plan_feedback`으로 되돌아오는 값 확인.
- 문서 9절 시나리오 체크리스트 재사용: KEEP(장애물 없음/신호 GREEN), FOLLOW(앞차 존재 시 `desired_speed` 하향 반영), RED STOP(`force_stop=true`→STOP 모드로 정지 후보 생성), GREEN 통과, LEFT/RIGHT CHANGE(활성 시 `d` 오프셋 반영), E-STOP(`emergency_stop=true`→즉시 감속 후보, 1.1.2의 코스트 우선순위 반영 확인).
- 2번(`mpc_controller`) 완료 후: `planner`를 켰다 껐다 하면서 "external trajectory 사용" ↔ "CSV fallback" 전환이 올바른지 로그로 확인.
