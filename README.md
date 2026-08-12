# MPC Workspace

MORAI와 UDP로 연결되는 자율주행 Planner/Controller 워크스페이스입니다.
현재 주행 파이프라인은 ROS에 의존하지 않는 두 개의 실행 파일로 구성됩니다.

```text
MORAI GPS/IMU/ObjectInfo
          ↓ UDP
Frenet Planner (20 Hz)
          ↓ PlannedPath UDP :9300
MPC Controller (20 Hz)
          ↓ CtrlCmd UDP
        MORAI
```

## 구성

- `src/frenet_planner`: Frenet 후보 생성, 모드 판단, 충돌검사, 경로 선택
- `src/mpc_controller`: LTV kinematic bicycle MPC 및 MORAI 제어 명령 송신
- `src/behavior_planner`: 팀원 개발 영역. 현재 UDP 주행 파이프라인과 미연동
- `src/object_udp_bridge`: ROS Object/Ego 토픽을 UDP로 중계하는 진단 도구
- `src/MORAI-ROS_morai_msgs`: MORAI ROS 메시지 서브모듈

## 구현된 Planner 기능

- Lane Keeping
- Constant-time-gap Following
- 정적 장애물 Avoid
- 회전교차로 Merge
- 고주로 다중 conflict-zone Merge
- 곡률·종/횡가속도 제한
- CTRV 장애물 예측과 SAT OBB 충돌검사
- 저속 경로 fallback과 MPC 정지 데드락 복구
- GPS/IMU 기반 localization
- 후보/탈락/선택 경로 실시간 시각화

동적 장애물 Avoid와 Behavior Planner 연동은 아직 구현되지 않았습니다.

## 빌드 및 실행

```bash
cd ~/mpc_ws
./run_integration.sh
```

Planner가 UDP 911 포트를 사용하므로 실행 중 `sudo` 비밀번호가 필요할 수 있습니다.
`Ctrl+C`로 MPC와 Planner를 함께 종료합니다.

개별 빌드:

```bash
cmake -S src/frenet_planner -B src/frenet_planner/build
cmake --build src/frenet_planner/build -j

cmake -S src/mpc_controller -B src/mpc_controller/build
cmake --build src/mpc_controller/build -j
```

## 네트워크 설정

Planner 설정:

- `src/frenet_planner/src/udp_network/network.yaml`
- Ego status 수신: `911`
- ObjectInfo 수신: `7505`
- GPS 수신: `1111`
- IMU 수신: `2222`
- PlannedPath 송신: `127.0.0.1:9300`

MPC 설정:

- `src/mpc_controller/src/udp_network/network.yaml`
- PlannedPath 수신: `127.0.0.1:9300`
- CtrlCmd 송신 목적지: 기본값 `172.29.96.1:9094`

WSL2 주소가 바뀌면 MORAI 센서 Destination IP를 현재 WSL 주소에 맞춰야 합니다.

```bash
hostname -I
ip -br addr
ip route
```

현재 localization 입력은 Planner의 `network.yaml`에서 선택합니다.

```yaml
localization:
  source: "gps_imu"  # gps_imu | ego_udp
```

## 주요 파라미터

- Planner: `src/frenet_planner/src/frenet_planner/config/params.yaml`
- MPC: `src/mpc_controller/src/config/mpc_params.yaml`
- Global path: `src/frenet_planner/src/config/2026_molit_comp_global_path.txt`

Merge의 `start_s`, `conflict_s`, `clear_s`는 현재 Global Path에 종속됩니다.
Global Path가 바뀌면 다시 측정해야 합니다.

## 시각화

Planner 실행 중 다른 터미널에서 실행합니다.

```bash
cd ~/mpc_ws
MPLCONFIGDIR=/tmp/matplotlib python3 src/frenet_planner/tools/planner_visualizer.py --view follow
```

- 초록색: 유효 후보
- 주황색: 곡률/횡가속도 탈락
- 빨간색: 충돌 탈락
- 자홍색: 최종 선택 경로

`--view fixed`는 고정 화면, `--view follow`는 Ego 중심 화면입니다.

## 남은 개발 항목

1. 고주로 Merge 반복 시나리오 회귀 검증
2. 동적 장애물 Avoid
3. Behavior Planner와 PlannerCommand 연결
4. 차선 경계/주행 가능 영역 필터
5. GPS 재밍 대응 localization
6. Planner/MPC 자동 회귀 테스트

## 서브모듈

처음 clone할 때 MORAI 메시지를 함께 받으려면 다음을 사용합니다.

```bash
git clone --recurse-submodules <repository-url>
```

이미 clone했다면:

```bash
git submodule update --init --recursive
```
