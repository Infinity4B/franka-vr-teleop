#!/bin/bash
# 启动适配器：仓库自带的 uv 环境（.venv，第一次会按 uv.lock 自动安装，--frozen 不改写 uv.lock），套 host/low_latency.sh（运行期间禁止 CPU 深度休眠）。
# 用法: ./start.sh [--home] [--dry-run] [--csv out.csv] [--host IP] ...（参数原样传给 python -m franka_vr_bridge）
cd "$(dirname "$0")" || exit 1
command -v uv >/dev/null || { echo "没有 uv，先安装: curl -LsSf https://astral.sh/uv/install.sh | sh"; exit 1; }
for a in "$@"; do
  case $a in --dry-run|-h|--help) exec uv run --frozen python -m franka_vr_bridge "$@" ;; esac   # 不连机器人，不需要低延迟
done
exec host/low_latency.sh uv run --frozen python -m franka_vr_bridge "$@"
