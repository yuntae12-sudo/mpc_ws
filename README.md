# mpc_ws

## Installation

깃 클론할 때 이렇게 해야 morai_msgs 같이 나와서 개발할 수 있음

```bash
git clone --recurse-submodules https://github.com/yuntae12-sudo/mpc_ws.git
```

이미 clone한 경우:
```bash
git submodule update --init --recursive
```
## 6/25일 경과
k-city 경로,launch 추가 및 경로 추정 완료
현재 k-city 코너 기반하여 속도 가변 주행. but pid기반으로 진행됨

## 실행 
```bash
roslaunch mpc_node mpc_node.launch
```
이때 gps,imu 위치 2,0,0으로 실행해야 경로 추정이 잘됨

## 6/27일 개발 내용
6/27

1. 기존 종방향 PID 제어기 제거 → Only MPC로 주행


기존에 accel/brake를 PID로 따로 제어하던 것을 MPC 단일 출력으로 통합
steer / throttle / brake 모두 MPC 한 곳에서 출력


2. vehicle_model kinematic bicycle model 1차 Taylor 선형화


기존: 비선형 모델 그대로 사용 → solver가 gradient 계산할 때마다 predictTrajectory() 호출 (4N × iter번)
gradient descent가 non-convex cost landscape에서 동작 → 급조향/급가속 시 gradient 폭발 또는 local minimum 수렴 위험


변경: 현재 동작점 (x̄, ū)에서 1차 Taylor 선형화 → A_k, B_k, c_k 행렬 계산
  x_{k+1} ≈ A_k * x_k + B_k * u_k + c_k

이후 gradient / line search에서 비선형 forward sim 대신 행렬 곱만으로 trajectory 예측
연산 부담 감소 + convex 문제로 안정적 수렴
추가된 함수:

linearizeBicycle(): 동작점 하나에서 A, B, c 계산
buildLTVModels(): horizon N 스텝 전체 선형화 모델 계산


3. Planner / Controller 분리


기존: mpc_node.cpp 안에 Planner(경로 생성)와 Controller(MPC 최적화)가 섞여있던 구조
변경:
planner/path_planner.cpp: ReferencePath 생성 담당 (Planner 역할)
controller/mpc_node.cpp: ReferencePath를 받아 solveMPC() 호출만 (Controller 역할)

분리 이유:
MPC solver는 ReferencePath를 입력으로 받아야 동작하므로 Planner가 필요함
하지만 Planner 로직을 mpc_node에서 분리해두면 나중에 Expert/PA/SA Planner로 교체할 때 path_planner.cpp만 바꾸면 됨
추후 Expert Planner, PA, SA 모두 같은 MPC Controller 재사용 가능

## 대회 규정에 맞는 ROS1 msgs 파일 클론
git clone -b beta_drive https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git

## 해야할 일
1. 현재 ROS토픽으로 진행중 이를 UDP로 변환
2. planning 개발
+파라미터 조금씩 하면서 수정 필요
+ROS 버전 나오면 인지 파트에 넘겨주고, UDP 변환 진행
(데이터 수집 자체는 ROS로 진행해야하기 때문, UDP로 데이터 수집 시 오류가 많다고 함)


## 7/4일 개발 내용

현재 Frenet Frame Planner 까지 개발 완료된 상태
아직 리팩토링은 하지 않아서 리팩토링 과정 필요
추가로 Planner & Controller 통합 과정 필요

## 7월 5일 개발 내용

MPC 파일에 남아있던 필요없는 코드들 삭제

## Frenet 후보 경로 실시간 시각화

Planner 실행 중 별도 WSL 터미널에서 다음을 실행한다.

```bash
cd ~/mpc_ws
MPLCONFIGDIR=/tmp/matplotlib python3 src/frenet_planner/tools/planner_visualizer.py
```

왼쪽은 전체 경로와 주행 이력, 오른쪽은 차량 주변 후보 경로 상세 화면이다.
초록색은 유효 후보, 주황색은 곡률 탈락, 빨간색은 충돌 탈락, 굵은 자홍색은
최종 선택 경로를 뜻한다. 설정은
`src/frenet_planner/src/frenet_planner/config/params.yaml`의
`planner.visualization`에서 켜거나 끌 수 있다.

기본 `--view fixed`는 실행 시점의 상세 화면 범위를 고정한다. 차량을 따라가는
화면이 필요하면 `--view follow`를 추가한다. 두 모드 모두 축·범례·전체 경로는
재생성하지 않고 기존 그래픽 객체의 좌표만 갱신한다.
