#!/bin/bash
# 跑全部离线测试（不需要头显和机器人）：适配器单元测试、头显单元测试、冒烟测试（主场景连 dry-run 适配器）。
# 需要 Godot 4.6.2（GODOT=...）和带 numpy、PyYAML 的 python3（PYTHON=...）。
set -e
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(dirname "$HERE")
GODOT=${GODOT:-godot}
PY=${PYTHON:-python3}
fail=0

echo "== 适配器单元测试"
(cd "$ROOT" && "$PY" -B -m unittest discover -s tests 2>&1 | tail -n 3) || fail=1

echo "== 导入 Godot 资源"
"$GODOT" --headless --xr-mode off --path "$ROOT/vr-app" --import >/dev/null 2>&1 || true

echo "== 头显单元测试"
out=$("$GODOT" --headless --xr-mode off --path "$ROOT/vr-app" -s res://tests/run_tests.gd 2>&1) || fail=1
echo "$out" | grep -E "✗|项检查|SCRIPT ERROR|Parse Error" || true
echo "$out" | grep -q -E "✗|SCRIPT ERROR|Parse Error" && fail=1
echo "$out" | grep -q "项检查，0 项失败" || fail=1

echo "== 冒烟测试（主场景 + dry-run 适配器）"
PORT=47913
# x 范围用 --ws 固定（smoke_main.gd 检查头显从 HELLO 收到 0.30），不随 config.yaml 调参变
(cd "$ROOT" && exec "$PY" -B -m franka_vr_bridge --dry-run --port $PORT --ws x=-0.25,0.30 >/dev/null 2>&1) &
BRIDGE=$!
sleep 2
out=$("$GODOT" --headless --xr-mode off --path "$ROOT/vr-app" -s res://tests/smoke_main.gd -- --port=$PORT 2>&1) || true
kill -INT $BRIDGE 2>/dev/null || true
echo "$out" | grep -E "✓|✗|SCRIPT ERROR|Parse Error"
echo "$out" | grep -q -E "✗|SCRIPT ERROR|Parse Error" && fail=1
echo "$out" | grep -q SMOKE_DONE || fail=1

[ $fail = 0 ] && echo "== 全部通过" || { echo "== 有失败"; exit 1; }
