#!/bin/bash
# 用 macOS 的 say（婷婷）生成头显语音提示到 vr-app/assets/voice/*.wav。改词后重跑即可。
# 取自 arx-vr-analysis，去掉了 Franka 版用不到的锁定、控制器上下线提示。
set -e
OUT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../vr-app" && pwd)/assets/voice
mkdir -p "$OUT"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

gen() {  # 文件名 文本
  say -v Tingting -r 210 -o "$TMP/$1.aiff" "$2"
  afconvert -f WAVE -d LEI16@22050 -c 1 "$TMP/$1.aiff" "$OUT/$1.wav"
  echo "  $1.wav  $2"
}

gen gate_on "遥操开启"
gen gate_off "遥操关闭"
gen connected "已连接"
gen link_down "连接断开"
gen paused "遥操暂停"
gen resumed "遥操继续，松开扳机再按"
gen aligned "已对齐前方"
