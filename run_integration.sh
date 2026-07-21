#!/usr/bin/env bash
# frenet_planner_node + mpc_node를 한 번에 빌드하고 실행한다.
#
# frenet_planner_node가 MORAI의 ego(911)/object(7505) 포트를 직접 bind하는데
# 911이 1024 미만 특권 포트라 sudo가 필요하다. mpc_node는 더 이상 MORAI의
# 포트를 직접 안 들어서(frenet_planner_node가 PlannedPath로 중계) sudo 없이
# 실행 가능하다.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1) 두 패키지 각각 빌드 (서로 독립적인 build/ 디렉터리)
for pkg in frenet_planner mpc_controller; do
  PKG_DIR="$SCRIPT_DIR/src/$pkg"
  mkdir -p "$PKG_DIR/build"
  cmake -S "$PKG_DIR" -B "$PKG_DIR/build" > /dev/null
  make -C "$PKG_DIR/build" -j"$(nproc)"
done

# 2) frenet_planner_node 백그라운드 실행 (911 포트 bind에 sudo 필요)
# 주의: `sudo cmd &`에서 $!는 sudo 래퍼 프로세스의 PID다. sudo는 자식으로
# 실제 frenet_planner_node를 fork하기 때문에, `sudo kill $!`는 래퍼만 죽이고
# 실제 워커는 부모 잃은 채(orphan) 살아남아 911/9300 포트를 계속 붙잡고
# 있었다(실측: PID 다르고, kill 이후에도 계속 동작/로그 출력됨). 그 결과 다음
# run_integration.sh 실행이 새로 띄운 프로세스는 포트 바인드 실패로 죽고,
# 이 좀비가 계속 예전 설정값으로 서빙해서 yaml을 고쳐도 반영이 안 되는
# 문제가 있었다. 프로세스 이름으로 확실히 정리하도록 pkill로 변경.
sudo "$SCRIPT_DIR/src/frenet_planner/build/frenet_planner_node" &
FRENET_PID=$!

cleanup() {
  echo ""
  echo "[run_integration] frenet_planner_node 종료 중..."
  sudo pkill -TERM -f "$SCRIPT_DIR/src/frenet_planner/build/frenet_planner_node" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 1   # frenet_planner_node가 소켓 bind할 시간 확보

# 3) mpc_node 포그라운드 실행 (Ctrl+C로 여기가 끝나면 trap이 frenet_planner_node도 정리)
"$SCRIPT_DIR/src/mpc_controller/build/mpc_node"
