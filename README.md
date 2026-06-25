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

## 해야할 일
1. PID제어기가 현재 종방향 제어를 하고 있는 상황 이를 완전히 mpc로 바꿔야함
2. 현재 ROS토픽으로 진행중 이를 UDP로 변환
3. planning 개발
+파라미터 조금씩 하면서 수정 필요