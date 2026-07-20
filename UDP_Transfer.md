# mpc_ws → UDP 단독 동작 재작성 청사진 (v1, 아직 통합 미실행)

> 목표: **ROS 완전 제거**, C++ 유지, MPC + planner + behavior 전부 UDP 기반 단일 실행으로 재작성.
> 알고리즘 코드(solver/model/cost/frenet/decision 등)는 **그대로 재사용**, 통신 껍데기만 교체.
> 기준 구조: `MORAI-DriveExample_UDP` (`network/` 통신 격리 + 중립 데이터 클래스 + 단일 `main`).

---

## 0. 핵심 발견 — 왜 이 재작성이 생각보다 할 만한가

세 노드의 **알고리즘 계층이 ROS로부터 이미 거의 완전히 분리**되어 있음. 조사 결과:

| 패키지 | 알고리즘 계층 (ROS 없음, 무수정 재사용) | ROS 껍데기 (교체 대상) |
|---|---|---|
| mpc_controller | `solver/`, `model/`, `cost/`, `constraints/`, `planner/`, `parameter_loader` | `main.cpp`, `node/mpc_node.*`, `global.hpp`(헤더 3줄 + `cmd_pub`) |
| planner | `frenet/`, `math/`, `visualization/` | `main.cpp`, `global/behavior_bridge.*`(ROS msg 타입 의존) |
| behavior_planner | `decision/`, `feature/`, `signal/`, `context/`(코어), `map/`(로직) | `node/behavior_node.*`, `node/morai_msg_converter.*`, `map/mgeo_loader.cpp`(ROS param), `context/msg_converter.cpp` |

즉 재작성은 "알고리즘을 다시 짜는" 게 아니라 **각 노드의 얇은 껍데기(콜백/pub-sub/spin/param)를 UDP + 일반 함수 호출로 갈아끼우는** 작업.

---

## 1. 목표 아키텍처

UDP 예제의 구조를 그대로 채택하되, 세 알고리즘을 하나의 실행 파일에서 순차 호출:

```
              [UDP RECEIVER THREADS]                    [ALGORITHM PIPELINE]              [UDP SENDER]
  MORAI SIM ──▶ EgoInfoReceiver   ─┐
  (UDP out)     ObjectInfoReceiver ─┼─▶ 중립 스냅샷 ─▶ behavior.decide() ─▶ ctx
                TrafficReceiver    ─┘   (VehicleState,   ─▶ planner.plan(ctx) ─▶ trajectory
                                        ObjectInfo[],    ─▶ mpc.control(trajectory) ─▶ CtrlCmd ──▶ CtrlCmdSender ──▶ MORAI SIM
                                        TrafficLight)                                                              (UDP in)
```

핵심 원칙 (UDP 예제 그대로):
- **통신은 `network/`에 완전 격리.** 알고리즘은 UDP를 모름.
- **중립 데이터 클래스**가 통신과 알고리즘 사이 경계. ROS 메시지 타입(`morai_msgs::*`, `behavior_planner::BehaviorContext` 등)을 이 경계 안쪽으로 넘기지 않음.
- **단일 `main` + 고정 주기 루프.** UDP 예제 `_main_loop`처럼 `sampling_time` 주기로 파이프라인 실행.

---

## 2. 새로 만들 것 — `udp_io` 통신 계층 (UDP 예제 이식)

`MORAI-DriveExample_UDP/network/`의 C++ 포팅. UDP 예제는 Python이므로 `struct.pack/unpack` → C++ `memcpy`/캐스팅 또는 직렬화 헬퍼로 옮김.

```
udp_io/
├── sender/
│   ├── sender.hpp            # 추상 Sender (socket + sendto)
│   └── ctrl_cmd_sender.*     # #MoraiCtrlCmd$ 패킷 조립 (예제 ctrl_cmd_sender.py 그대로)
├── receiver/
│   ├── receiver.hpp          # 추상 Receiver (bind + recv 스레드)
│   ├── ego_info_receiver.*   # #MoraiInfo$ 파싱 (예제 ego_info_receiver.py 필드 오프셋 그대로)
│   ├── object_info_receiver.*# #MoraiObjInfo$ 파싱
│   └── traffic_receiver.*    # #TrafficLight$ 파싱
├── datatypes.hpp             # 중립 데이터 클래스 (아래 3절)
└── config.(json|yaml)        # 예제 config.json의 포트/IP 그대로
```

**필드 오프셋은 UDP 예제 파서에서 그대로 가져온다** (검증된 값):
- Ego: `pos_x/y` = raw_data[77:89], `yaw` = raw_data[89:101]의 세번째, `vel_x` = raw_data[101:113]의 첫째, `front_steer` = raw_data[137:145]의 첫째. (예제 `ego_info_receiver.py` 참조)
- CtrlCmd 송신: `#MoraiCtrlCmd$` + mode=2(auto) + gear=4(drive) + cmd_type=1(throttle) + accel/brake/front_steer. (예제 `ctrl_cmd_sender.py` 그대로)

---

## 3. 중립 데이터 클래스 — 통신↔알고리즘 경계

UDP 예제의 `VehicleState`/`ObjectInfo` 역할. **세 노드가 각자 다르게 하던 Ego 해석을 여기 한 곳으로 통일** (이게 "패키지마다 다르다" 문제의 실제 해결점).

```cpp
// udp_io/datatypes.hpp
struct VehicleState {   // 예제 VehicleState 대응
    double x, y;        // MORAI 월드좌표
    double yaw;         // [rad]  ← 여기서 deg→rad 변환 단일화
    double v;           // [m/s]  ← 여기서 km/h 처리 단일화
    double front_steer; // [rad]  ← planner의 wheel_angle TODO를 여기서 확정
    double accel;       // [m/s²]
};

struct ObjectInfo {     // 예제 ObjectInfo 대응
    int id, type;       // 0:person 1,2:vehicle 3:traffic light
    double x, y;
    double heading;     // [rad]
    double speed;       // [m/s]
    double length, width;
};

struct TrafficLightState { std::string index; int status; };
```

> **결정 필요 지점 A — 단위/부호 확정.** 지금 planner는 `wheel_angle`(deg 가정, 부호 미검증)을, UDP 예제는 `front_steer`(rad)를 준다. 재작성 시 **`front_steer`(rad)를 정본으로 삼고**, 기존 알고리즘이 기대하던 형태로 이 경계에서 한 번만 변환. 실측으로 부호(좌회전=+) 확인 필요.

---

## 4. 알고리즘 계층 어댑터 — 각 노드의 "껍데기"만 재작성

기존 알고리즘 함수는 대부분 전역 상태(`ego`, `g_ego_cs`, `g_obstacles` 등) + ROS 콜백으로 값이 채워지는 구조. 재작성 시 **전역 상태를 중립 데이터 클래스로 채우는 어댑터**만 새로 씀. 알고리즘 함수 시그니처는 최대한 유지.

### 4.1 behavior 어댑터
- 버릴 것: `behavior_node.*`(spin/pub-sub), `morai_msg_converter.*`(morai_msgs→EgoState), `mgeo_loader`의 `ros::param` 로딩, `context/msg_converter.cpp`.
- 새로 쓸 것: `VehicleState`/`ObjectInfo[]` → 기존 `EgoState`/내부 타입으로 넣는 변환 (morai_msg_converter의 좌표/타입 매핑 로직 **재사용**, ROS 타입만 제거). MGeo 경로는 `ros::param` 대신 config 파일 경로로 로드.
- 산출: `decision/`이 만들어내던 `BehaviorContext`에 해당하는 **중립 struct** (ROS 메시지 아님).

### 4.2 planner 어댑터
- 버릴 것: `main.cpp`의 spin/pub-sub, `/behavior/context` 구독, `/frenet_planner/trajectory` 발행.
- **주의 — `behavior_bridge.*`**: 지금 `behavior_planner::BehaviorContext`(ROS msg)에 직접 의존(`#include <behavior_planner/BehaviorContext.h>`). 재작성 시 이 include를 4.1의 **중립 struct로 교체**해야 함. 함수 본체(BuildCommandFromContext 등)는 필드 접근만 바꾸면 재사용 가능.
- 산출: 기존 `Float32MultiArray` 레이아웃(`[n, x[n], y[n], yaw[n], kappa[n], v[n], a[n]]`) 대신 **중립 trajectory struct**를 mpc에 직접 전달.

### 4.3 mpc 어댑터
- 버릴 것: `main.cpp`(init/spin/timer), `node/mpc_node.cpp`의 `CBEgoState`/`publishCtrlCmd`, `global.hpp`의 ROS include 3줄 + `cmd_pub`/`ego_mutex`.
- 새로 쓸 것: `VehicleState` → `MPCState ego` 채우기 (기존 `CBEgoState` 본문 로직 재사용). 4.2의 trajectory struct → `ReferencePath` 변환 (INTEGRATION_PLAN 3번이 원래 하려던 것을 함수 호출로).
- 산출: `MPCControl` → `CtrlCmdSender`로 (기존 `publishCtrlCmd`의 clip/정규화 로직 재사용).

> **결정 필요 지점 B — INTEGRATION_PLAN 3번 미구현.** 현재 mpc는 planner의 trajectory를 **아직 구독하지 않음**(CSV `path.txt` fallback만 있음). UDP 재작성과 별개로 "planner→mpc trajectory 연결"이 먼저 완성돼야 파이프라인이 닫힘. 재작성 시 이 연결을 함수 호출로 자연스럽게 넣을 수 있어 오히려 기회.

---

## 5. 메인 루프 — UDP 예제 `_main_loop` 대응

```cpp
// main.cpp (새 단일 실행)
int main() {
    load_config();                    // 예제 config.json 대응
    set_protocol();                   // receiver/sender 소켓 오픈 (예제 _set_protocol)
    behavior.init(mgeo_path);
    planner.init(params);
    mpc.init(mpc_params);

    while (running) {                 // 예제 _main_loop
        auto t0 = now();
        if (have_ego()) {             // 예제 if self.vehicle_state
            auto ego  = latest_vehicle_state();      // receiver 스냅샷
            auto objs = latest_objects();
            auto tl   = latest_traffic();

            auto ctx  = behavior.decide(ego, objs, tl);   // 4.1
            auto traj = planner.plan(ego, objs, ctx);     // 4.2
            auto cmd  = mpc.control(ego, traj);           // 4.3

            ctrl_cmd_sender.send(cmd);                    // UDP 송신
        }
        sleep_to_maintain_rate(t0);   // 예제 sampling_time 유지
    }
}
```

> **결정 필요 지점 C — 주기(rate) 단일화.** 지금 세 노드가 서로 다른 주기(behavior `ros::Rate`, planner/mpc `ros::Timer`)로 돌았음. 단일 루프로 합치면 **가장 느린 단계가 전체 주기를 결정**. behavior(MGeo/FSM)가 무거우면 mpc 제어 주기가 떨어질 수 있음 → behavior만 별도 스레드에서 낮은 주기로 돌리고 최신 ctx를 스냅샷하는 구조(예제의 receiver 콜백 패턴과 동일)로 분리하는 것을 권장.

---

## 6. 빌드 — catkin/ROS 제거

- `catkin` 대신 **순수 CMake**(또는 유지하되 `roscpp`/`morai_msgs` 의존만 제거). `find_package(catkin ...)`, `add_message_files`, `generate_messages` 전부 삭제.
- `morai_msgs` 메시지 생성 불필요 (UDP 바이트 직접 파싱).
- 남는 외부 의존: 없음에 가까움 (소켓은 표준 라이브러리). Eigen 등 알고리즘이 쓰던 수치 라이브러리만 유지.

---

## 7. 제거/유지 파일 최종 목록

**제거 (ROS 껍데기):**
- `mpc_controller/src/main.cpp`, `node/mpc_node.*`
- `planner/src/main.cpp`
- `behavior_planner/src/node/*`, `context/msg_converter.cpp`
- 세 패키지의 `.launch`, `package.xml`의 ROS 의존, `msg/` 정의 전부
- `integration_launch` 전체

**유지 (무수정 또는 include만 교체):**
- `mpc_controller/src/{solver,model,cost,constraints,planner}/*` — 무수정
- `planner/src/{frenet,math}/*` — 무수정, `behavior_bridge.*`는 include만 교체
- `behavior_planner/src/{decision,feature,signal,map(로직),context(코어)}/*` — ROS param 로딩만 config 로딩으로
- `mpc_controller/src/global/global.hpp` — ROS include 3줄 + `cmd_pub`/`ego_mutex`/`ros.h`만 제거

---

## 8. 권장 실행 순서 (재작성 시)

1. **`udp_io` 통신 계층부터** (2절). 알고리즘 없이 UDP 예제 그대로 C++ 포팅 → `rostopic` 없이도 시뮬레이터와 Ego 수신 / CtrlCmd 송신만 단독 검증.
2. **mpc 단독 UDP 동작** (4.3 + CSV path.txt fallback). behavior/planner 없이 UDP로 받은 ego + 기존 CSV 경로로 제어가 도는지 확인 (가장 단순한 닫힌 루프).
3. **planner 붙이기** (4.2). trajectory struct로 mpc에 직접 전달 (결정 지점 B 해소).
4. **behavior 붙이기** (4.1). ctx struct 전달, 필요 시 별도 스레드(결정 지점 C).
5. **주기/스레드 튜닝** 및 알고리즘 코드 정리(추후 예정 부분).

---

## 9. 시작 전 확정할 결정 사항 (요약)

- **A. Ego 단위/부호**: `front_steer`(rad) 정본화, 좌회전 부호 실측 확인.
- **B. planner→mpc trajectory 연결**: 현재 미구현. 재작성에 포함.
- **C. 루프 주기**: 단일 루프 vs behavior 별도 스레드. 무거운 behavior 고려 시 후자 권장.
- **D. config 포맷**: UDP 예제 `config.json` 유지 vs yaml 통일. MGeo 경로 등 ROS param이던 것 여기로.