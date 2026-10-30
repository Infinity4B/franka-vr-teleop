#!/bin/bash
# 安装 FrankaTeleop.apk 到 Quest 3S 并启动。取自 arx-vr-analysis，改了 APK 名和包名。
#
#   bash tools/install_apk.sh                     USB 连接（头显里允许 USB 调试）
#   bash tools/install_apk.sh --wifi 192.168.1.50 先 USB 连一次开启无线 adb，之后可拔线
#   bash tools/install_apk.sh --log               安装后跟踪日志（Ctrl+C 退出）
#   bash tools/install_apk.sh --awake             同时关闭佩戴检测（头显挂在脖子上用，见 quest_awake.sh）
#   APK=path/to.apk bash tools/install_apk.sh     安装指定 APK
set -e
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
APK=${APK:-$(dirname "$HERE")/build/FrankaTeleop.apk}
PKG=com.infinity4b.frankateleop
ADB=${ADB:-$(command -v adb || echo "$HOME/Library/Android/sdk/platform-tools/adb")}
WIFI=
LOG=
AWAKE=

while [ $# -gt 0 ]; do
  case $1 in
    --wifi) WIFI=$2; shift 2 ;;
    --log) LOG=1; shift ;;
    --awake) AWAKE=1; shift ;;
    *) echo "未知参数 $1"; exit 1 ;;
  esac
done

[ -x "$ADB" ] || { echo "找不到 adb（brew install --cask android-platform-tools）"; exit 1; }
[ -f "$APK" ] || { echo "找不到 ${APK}：先 bash tools/build_apk.sh，或从 GitHub Actions 下载后用 APK=<路径> 指定"; exit 1; }

if [ -n "$WIFI" ]; then
  if ! "$ADB" devices | grep -q "^$WIFI:5555"; then
    "$ADB" tcpip 5555 && sleep 2
    "$ADB" connect "$WIFI:5555"
  fi
  export ANDROID_SERIAL=$WIFI:5555
fi

n=$("$ADB" devices | awk 'NR>1 && $2=="device"' | wc -l | tr -d ' ')
if [ "$n" = 0 ]; then
  echo "没有可用设备。检查：头显已开开发者模式、USB 线已连接、头显里点了“允许 USB 调试”"
  "$ADB" devices
  exit 1
fi
if [ "$n" -gt 1 ] && [ -z "${ANDROID_SERIAL:-}" ]; then
  echo "有多台设备，请用 ANDROID_SERIAL=<序列号> 指定："
  "$ADB" devices
  exit 1
fi

echo "== 安装 $APK"
if ! "$ADB" install -r "$APK"; then
  echo "覆盖安装失败（多半是签名不同），卸载旧版后重装（会清除头显里的设置）"
  "$ADB" uninstall "$PKG" || true
  "$ADB" install "$APK"
fi
echo "== 启动"
# GodotApp 本身不导出，要从启动器别名进
"$ADB" shell am start -n "$PKG/com.godot.game.GodotAppLauncher" >/dev/null
if [ -n "$AWAKE" ]; then
  ADB=$ADB bash "$HERE/quest_awake.sh"
fi
echo "已启动。戴上头显；设置文件：/sdcard/Android/data/$PKG/files/settings.cfg"

if [ -n "$LOG" ]; then
  "$ADB" logcat -c
  "$ADB" logcat -s godot:V GodotOpenXR:V
fi
