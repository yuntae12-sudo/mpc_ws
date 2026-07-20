#!/usr/bin/env bash
# mpc_node 빌드 + 실행 (포트 908~911대가 1024 미만이라 sudo 필요)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

mkdir -p "$BUILD_DIR"
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" > /dev/null
make -C "$BUILD_DIR" -j"$(nproc)"

exec sudo "$BUILD_DIR/mpc_node"
