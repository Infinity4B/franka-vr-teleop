#!/bin/bash
# 关闭 / 恢复 Quest 的佩戴检测（距离传感器）。
#
#   bash tools/quest_awake.sh        摘下头显也不休眠，Franka Teleop 不会因此暂停
#   bash tools/quest_awake.sh off    恢复：摘下头显即休眠，应用暂停
#
# 头显重启后自动恢复成默认（摘下即休眠），需要重新执行。
# 关闭后“摘头显 → 遥操暂停”这条保护不再生效：手柄放下前先松开扳机，或长按 B 关闭遥操。
set -e
ADB=${ADB:-$(command -v adb || echo "$HOME/Library/Android/sdk/platform-tools/adb")}
[ -x "$ADB" ] || { echo "找不到 adb（brew install --cask android-platform-tools）"; exit 1; }

n=$("$ADB" devices | awk 'NR>1 && $2=="device"' | wc -l | tr -d ' ')
if [ "$n" = 0 ]; then
  echo "没有可用设备：插上数据线，头显里允许 USB 调试；无线连接用 install_apk.sh --wifi <IP>"
  "$ADB" devices
  exit 1
fi
if [ "$n" -gt 1 ] && [ -z "${ANDROID_SERIAL:-}" ]; then
  echo "有多台设备，请用 ANDROID_SERIAL=<序列号> 指定："
  "$ADB" devices
  exit 1
fi

case ${1:-on} in
  on)
    "$ADB" shell am broadcast -a com.oculus.vrpowermanager.prox_close >/dev/null
    echo "已关闭佩戴检测：摘下头显不休眠。头显重启后失效，需要重新执行。"
    ;;
  off)
    "$ADB" shell am broadcast -a com.oculus.vrpowermanager.automation_disable >/dev/null
    echo "已恢复佩戴检测：摘下头显即休眠、遥操暂停。"
    ;;
  *)
    echo "用法：bash $0 [on|off]"
    exit 1
    ;;
esac
