#!/usr/bin/env bash
# frenet_planner_node 빌드 + 단독 실행 (ego 포트 911이 1024 미만이라 sudo 필요)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

mkdir -p "$BUILD_DIR"
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" > /dev/null
make -C "$BUILD_DIR" -j"$(nproc)"

exec sudo "$BUILD_DIR/frenet_planner_node"
