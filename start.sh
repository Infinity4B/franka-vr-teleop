#!/bin/bash
# lab 上启动适配器：用 franky_ctl 的 uv 环境，套 low_latency.sh（禁止 CPU 深度休眠）。
# 用法: ./start.sh [--home] [--dry-run] [--csv out.csv] [--host IP] ...（参数原样传给 python -m franka_vr_bridge）
cd "$(dirname "$0")"
FRANKA=${FRANKA_DIR:-$HOME/Documents/Code/franka}
exec "$FRANKA/low_latency.sh" uv run --project "$FRANKA/franky_ctl" python -m franka_vr_bridge "$@"
