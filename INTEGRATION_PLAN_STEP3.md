# 3단계: planner → mpc_controller 연동 (v5 — CSV 폴백 없이 frenet_planner 단일 소스 + 짧은 유예)

## Context

`behavior_planner ↔ planner` 브릿지(1+2단계, `behavior_bridge.hpp/.cpp` 등)는 완료되어 MORAI 실차 시뮬에서 KEEP/FOLLOW/LEFT/RIGHT/E-STOP/RED STOP 전 시나리오 검증까지 끝났다(자세한 내용은 저장소 루트 `INTEGRATION_PLAN.md` 참고). 그 검증 과정에서 `planner` 자체의 버그도 여럿 고쳤다 — waypoint 급커브 스무딩(`ref_line.cpp`), 저속(STOP/EMERGENCY) 후보 전멸 버그(`ComputeGeometricPath` 도입), 정지점 NaN(0/0) 방어, ego kappa 근사(`wheel_angle` 기반) 등.

남은 마지막 단계는 `mpc_controller`가 `planner`의 `/frenet_planner/trajectory`를 실제로 구독해서, 차가 지금처럼 독립적인 CSV 경로가 아니라 **`planner`가 계산한 최적 경로를 실제로 따라 달리게** 만드는 것이다.

**핵심 설계 원칙(v5, 사용자 확정 — v4에서 변경됨)**: 최종 통합 상태에서 `mpc_controller`는 **CSV 주행을 아예 하지 않는다.** `/frenet_planner/trajectory`가 끊길 일이 없도록 만드는 것이 전제이므로, "연결 장애 대비 CSV 폴백"이라는 개념 자체를 두지 않는다. `mpc_controller`는 오직 `planner`의 결과만으로 주행한다.

CSV 관련 코드(로딩, `buildReferenceFromWaypoints`, `closest_waypoint_idx` 등)는 **삭제하지 않고 남겨둔다** — 추후 다른 개발에서 재사용할 가능성이 있기 때문. 다만 `controlLoop()`에서 더 이상 호출하지 않는다(사실상 죽은 코드로 둠).

**빈 사이클(n=0) 정책(사용자 확정)**: planner가 이번 사이클에 유효 경로를 못 내도(`n=0`), 곧바로 급정지하지 않는다. 알고리즘의 순간적 드롭아웃(한두 사이클 씹힘)에 급정지로 반응하면 오히려 뒤차 추돌 등 새 위험을 만들 수 있기 때문. 대신 **아주 짧은 시간(수백 ms) 동안만 직전에 받은 유효 외부 경로를 그대로 유지**해서 계속 달리다가, 그 시간 안에 새 경로가 오면 그걸로 갈아타고, 계속 안 오면 정지한다.

이건 v4의 `EXTERNAL_GRACE`와 메커니즘은 같지만 성격이 다르다 — v4에서는 grace 이후 **CSV로 넘어가는 게 문제**(버그가 CSV 뒤에 숨음)였지, grace 자체(직전 자기 출력을 짧게 유지)가 문제였던 게 아니다. v5는 grace 이후의 목적지가 "CSV" 대신 "정지"로 바뀐 것뿐이다. E-STOP/정지선/Force-Stop처럼 planner가 의도적으로 속도 0에 수렴하는 정지 경로를 준 경우는 그 경로를 그대로 따르면 자연히 멈추므로 이 grace 로직과 무관하다 — 여기서 다루는 건 순수하게 "이번 사이클에 경로 자체가 안 나온" 진짜 빈 사이클 케이스뿐이다.

grace 시간이 길면 고속도로 `desired_speed` 버그(15초+ 연속 빈 사이클) 같은 상황이 다시 오래 감춰지므로, **grace는 짧게(예: 0.3~0.5초, MPC 몇 사이클)** 유지해서 "순간적 드롭아웃 흡수"와 "알고리즘 버그가 뚜렷하게 드러남" 사이의 균형을 잡는다.

## 전체 데이터 흐름

```
planner (frenet_planner_node)                          mpc_controller (mpc_node)
──────────────────────────────                         ─────────────────────────
PlanningLoop (기존 그대로)
  best 있음  → CartesianPath(n>0) 발행
  best 없음  → CartesianPath(n=0, "살아있지만 계획 없음") 발행   ← 이번에 추가
        │
        ▼
/frenet_planner/trajectory (Float32MultiArray)  ────▶  CBExternalTrajectory (신규)
                                                          → mutex 보호 버퍼 + 수신 타임스탬프
                                                        controlLoop 매 사이클:
                                                          1) 외부 trajectory 스냅샷 확인
                                                          2) n > 0 이면: buildReferenceFromExternalTrajectory로
                                                             ReferencePath 구성 → 그걸로 주행, "마지막 유효
                                                             경로"로 저장
                                                          3) n == 0 이면: 짧은 유예(external_empty_grace_s)
                                                             동안은 "마지막 유효 경로"를 그대로 유지해서 주행
                                                          4) 유예 시간도 넘도록 계속 n==0 이면: 정지 명령
                                                             (CSV로 대신 주행하지 않음)
```

CSV 경로(`buildReferenceFromWaypoints`, `closest_waypoint_idx`, waypoint 로딩)는 코드에 그대로 남지만 `controlLoop()`에서 호출되지 않는다.

## 구현

### 1. `planner`: `best`가 없어도 매 사이클 trajectory 발행

**파일**: `src/planner/src/main.cpp` (`PlanningLoop`)

현재:
```cpp
if (!best) {
    ROS_WARN_THROTTLE(1.0, "[FrenetPlanner] No valid candidate this cycle (%zu generated)", candidates.size());
    return;
}
CartesianPath cp = ConvertToCartesianPath(*best, g_ref);
PublishCartesianPath(cp);
```
변경 후:
```cpp
if (!best) {
    ROS_WARN_THROTTLE(1.0, "[FrenetPlanner] No valid candidate this cycle (%zu generated)", candidates.size());
    PublishCartesianPath(CartesianPath{});   // n=0, "살아있지만 계획 없음" 명시적 신호
    return;
}
CartesianPath cp = ConvertToCartesianPath(*best, g_ref);
PublishCartesianPath(cp);
```
`CartesianPath`(`global.hpp`)는 `std::vector<double> x,y,yaw,kappa,v,a`만 있는 순수 aggregate라 `CartesianPath{}`로 안전하게 빈 값 생성 가능(확인됨). `PublishCartesianPath`는 `cp.x.size()==0`이면 이미 `data=[0]`(n=0 헤더만)을 발행하므로 함수 자체는 수정 불필요 — 호출부만 바꾸면 됨. 마커/피드백 발행부는 이미 `best` 유무와 무관하게 항상 실행되므로 그대로 둔다.

### 2. `mpc_controller`: 외부 trajectory 버퍼 + 새 구조체/파라미터

**파일**: `src/mpc_controller/src/global/global.hpp`

```cpp
// 외부(frenet planner) 궤적 원본 파싱 결과
struct ExternalTrajectory {
    std::vector<double> x, y, yaw, k, v;   // a는 MPC가 안 씀 (ReferencePath에 대응 필드 없음)
    bool valid() const { return !x.empty(); }
    void clear() { x.clear(); y.clear(); yaw.clear(); k.clear(); v.clear(); }
};
```

`MPCParams`에 추가:
```cpp
double external_empty_grace_s = 0.4;  // [s] n==0가 몇 사이클 이어져도 이 시간까지는 직전 유효 경로 유지
```

새 extern(기존 `ego`/`ego_mutex` 패턴 그대로):
```cpp
extern ExternalTrajectory external_traj;
extern std::mutex         external_traj_mutex;
extern ReferencePath      external_ref_last_good;       // grace 동안 유지할 직전 유효 ReferencePath
extern ros::Time          external_ref_last_good_stamp;
```
`#include <std_msgs/Float32MultiArray.h>` 추가 필요.

(v4에 있던 `external_trajectory_timeout`/`external_traj_stamp`/`external_traj_received`는 불필요 — "토픽 자체가 끊김"에 대한 연결 장애 폴백이 없으므로 별도 타임아웃/수신여부 추적이 필요 없다. `external_ref_last_good`는 연결 장애 폴백용이 아니라 "순간적 빈 사이클 흡수"용으로만 쓰인다.)

**파일**: `src/mpc_controller/src/global/global.cpp` — 위 extern들의 정의 추가 (기존 `ego`/`ego_mutex` 정의부 옆에).

### 3. `mpc_controller`: 구독 콜백 + 파싱

**파일**: `src/mpc_controller/src/node/mpc_node.cpp` / `.hpp`

`CBEgoState` 옆에 신규 콜백 추가. `/frenet_planner/trajectory`의 `[n, x[n], y[n], yaw[n], kappa[n], v[n], a[n]]` 레이아웃을 파싱해서 `external_traj`에 저장(`a[n]` 구간은 의도적으로 안 씀). `n>0`인데 `data.size()`가 기대치(`1+6n`)보다 작으면 방어적으로 `n=0` 취급(=`clear()`) + 경고. `external_traj_mutex`로 보호.

**파일**: `src/mpc_controller/src/main.cpp` — `ros::Subscriber traj_sub = nh.subscribe("/frenet_planner/trajectory", 1, CBExternalTrajectory);` 추가.

### 4. `mpc_controller`: 외부 trajectory → ReferencePath 변환 함수

**파일**: `src/mpc_controller/src/planner/path_planner.cpp` / `.hpp` (기존 "Planner 분리" 관례 그대로)

```cpp
bool buildReferenceFromExternalTrajectory(
    const ExternalTrajectory& ext,
    const MPCParams&          params,
    ReferencePath&            out_ref);
```
- CSV 버전과 달리 **closest-point 탐색 불필요** — `planner`가 이미 ego 기준 전방(forward-looking) 궤적을 만들어 보내주기 때문.
- `x_ref/y_ref/yaw_ref/v_ref/k_ref`를 그대로 복사(윈도우 트리밍 없음). 근거: `planner`의 `path_generator/time_horizon: {min:2.0, max:5.0}`, `dt:0.1` → 후보 경로는 2.0~5.0초, 즉 약 20~50개 점. `mpc_params.yaml`의 실제 `horizon: 14`(코드 기본값 15가 아님, 항상 `mpc_params.horizon`을 런타임에 읽을 것) `* dt 0.1s = 1.4초` 소비 창보다 항상 더 기니까 패딩/외삽 불필요.
- `v_ref` 재스무딩 안 함 — 이미 다항식 최적화로 나온 부드러운 속도 프로파일이라 CSV 버전의 박스 스무딩을 다시 적용하면 오히려 왜곡됨.
- `ext.x.size() < 2`면 방어적으로 `false` 반환 + `ROS_WARN_THROTTLE`(정상 상황에서는 안 일어나야 함, `planner`쪽 horizon 설정 오류 신호).

### 5. `mpc_controller`: `controlLoop()` — frenet_planner 단일 소스 + 짧은 유예

**파일**: `src/mpc_controller/src/node/mpc_node.cpp`

```cpp
// 외부 trajectory 스냅샷 (CSV 경로는 더 이상 controlLoop에서 호출하지 않음 — 코드는 남겨둠)
ExternalTrajectory ext_snap;
{
    std::lock_guard<std::mutex> lk(external_traj_mutex);
    ext_snap = external_traj;
}

ReferencePath ref;
std::string active_source;   // 로그: EXTERNAL / EXTERNAL_GRACE / STOPPED
static int consecutive_empty_count = 0;

if (ext_snap.valid()) {
    ReferencePath built;
    if (buildReferenceFromExternalTrajectory(ext_snap, mpc_params, built)) {
        ref = built;
        external_ref_last_good = built;
        external_ref_last_good_stamp = ros::Time::now();
        active_source = "EXTERNAL";
        consecutive_empty_count = 0;
    }
}

if (active_source.empty()) {   // 이번 사이클 n==0 (또는 변환 실패)
    consecutive_empty_count++;
    bool have_grace = !external_ref_last_good.empty() &&
        (ros::Time::now() - external_ref_last_good_stamp).toSec() < mpc_params.external_empty_grace_s;
    if (have_grace) {
        ref = external_ref_last_good;
        active_source = "EXTERNAL_GRACE";
        ROS_WARN_THROTTLE(1.0, "[MPC] frenet_planner empty this cycle (%d consecutive) - "
                                "coasting on last valid trajectory (grace %.2fs)",
                                consecutive_empty_count, mpc_params.external_empty_grace_s);
    } else {
        ROS_WARN("[MPC] frenet_planner empty beyond grace(%.2fs, %d consecutive) - STOPPING "
                 "(no CSV fallback by design)", mpc_params.external_empty_grace_s, consecutive_empty_count);
        active_source = "STOPPED";
        // 기존과 동일하게 정지 명령 후 return
        ...
        return;
    }
}
ROS_INFO_THROTTLE(2.0, "[MPC] active reference source: %s", active_source.c_str());
// 이하 solveMPC(...) 등 기존 코드 그대로
```

**정확성 체크포인트**:
- `buildReferenceFromWaypoints`/`closest_waypoint_idx` 관련 호출은 `controlLoop()`에서 완전히 제거 — 함수/전역변수 정의는 남기되 호출부만 삭제.
- `external_ref_last_good`는 오직 `EXTERNAL`(n>0, 변환 성공) 분기에서만 갱신 — `EXTERNAL_GRACE`에서 갱신하면 grace가 매 사이클 리셋되어 무한정 늘어남(반드시 피해야 할 버그).
- grace를 넘기면 `consecutive_empty_count`를 리셋하지 않는다 — 로그에 "몇 연속" 숫자가 계속 누적돼야 고속도로 버그 지속 시간을 로그만으로 가늠 가능.
- 두 경고 문구를 의도적으로 다르게(`coasting on last valid trajectory` / `STOPPING (no CSV fallback by design)`) 만들어서 검증 시 로그 grep만으로 두 상황을 구분 가능하게 함.

### 6. 파라미터 연결

**파일**: `src/mpc_controller/src/config/mpc_params.yaml`
```yaml
external_trajectory:
  empty_grace_s: 0.4
```

**파일**: `src/mpc_controller/src/global/parameter_loader.cpp`
```cpp
pnh.param<double>("external_trajectory/empty_grace_s", p.external_empty_grace_s, p.external_empty_grace_s);
```
로그 라인 하나 추가(`ROS_INFO("[MPC] External trajectory empty grace: %.2fs", ...)`, 기존 스타일 그대로).

### 7. 빌드

새 소스 파일 없음(전부 기존 파일 수정) → `CMakeLists.txt`/`package.xml` 변경 불필요. `std_msgs`는 이미 두 파일 모두에 의존성으로 등록되어 있음(확인됨).

## 검증 방법 (MORAI 실차 시뮬, 기존 rostopic/로그 대조 방식 재사용)

1. **정상 주행 확인**: `rostopic hz /frenet_planner/trajectory`(~10Hz), `mpc_node` 로그에 `active reference source: EXTERNAL`이 뜨는지. 차의 실제 주행 경로가 `planner`가 고른 `best` 후보(rviz 마커)와 실제로 일치하는지 — "계획만 있고 안 따라가던" 이전 상태와 달리 실제로 따라가는지 확인.
2. **`frenet_planner_node` 강제 종료 테스트**(`rosnode kill`): `external_empty_grace_s`(0.4s) 이내에는 `EXTERNAL_GRACE`로 잠깐 버티다가, 이후 `STOPPING (no CSV fallback by design)` 경고와 함께 정지하는지. CSV로 넘어가지 않는지 확인(설계대로).
3. **고속도로 구간(기존 버그) 재주행**: `mpc_node` 로그에 처음 0.4초는 `EXTERNAL_GRACE`, 이후 `STOPPING`이 `frenet_planner_node`의 `No valid candidate` 구간(~15초)과 거의 같은 길이로 나타나는지 — **이 버그를 지금 고치는 게 아니라, CSV 뒤에 숨지 않고 뚜렷하게 드러나는지만 확인**.
4. **순간적 드롭아웃 흡수 확인**: 정상 주행 중 1~2 사이클만 `No valid candidate`가 튀는 짧은 순간(있다면)에 차가 급정지하지 않고 `EXTERNAL_GRACE`로 매끄럽게 넘어가는지.
5. **기존 시나리오(KEEP/FOLLOW/LEFT/RIGHT/STOP/E-STOP) 재검증**: `planner` 결과 기반 주행 상태에서 전 시나리오 재확인.

## 다음 단계(이번 스코프 밖, 3단계 검증 후 필요하면 진행)

- 고속도로 `desired_speed` 불일치 버그의 실제 수정: `BuildCommandFromContext`/`GenerateVelocityKeepingCandidates`에서 `target_speed`를 가속도 한계(`ego_speed ± max_longitudinal_accel × T_max`) 내로 클램핑. v5 설계에서는 이 버그가 발생하는 동안(grace 0.4초 이후) 차가 정지하므로, 수정 전까지는 고속도로 구간에서 정지가 반복될 수 있음을 감안해야 함.
