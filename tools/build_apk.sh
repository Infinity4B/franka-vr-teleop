#!/bin/bash
# 构建 build/FrankaTeleop.apk（Godot 4.6.2 + OpenXR Vendors 5.1.0，Gradle 构建）。取自 arx-vr-analysis，改了名字和签名变量。
#
#   bash tools/build_apk.sh            发布版，用 keystore/franka-teleop.keystore 签名（没有就生成；keystore/ 不进 git）
#   bash tools/build_apk.sh debug      调试版，用 Godot 的 debug.keystore 签名
#
# 环境变量：GODOT（默认 godot）、JAVA_HOME（默认 Homebrew openjdk@17）、
#           FRANKA_KEYSTORE / FRANKA_KEYSTORE_USER / FRANKA_KEYSTORE_PASS（自带发布签名时使用，GitHub Actions 用这组）
set -e
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(dirname "$HERE")
APP=$ROOT/vr-app
OUT=$ROOT/build
MODE=${1:-release}
GODOT=${GODOT:-godot}

if [ -z "${JAVA_HOME:-}" ]; then
  for j in /opt/homebrew/opt/openjdk@17/libexec/openjdk.jdk/Contents/Home /usr/lib/jvm/java-17-openjdk-amd64; do
    [ -d "$j" ] && { export JAVA_HOME=$j; break; }
  done
fi
[ -n "${JAVA_HOME:-}" ] || { echo "需要 JDK 17（brew install openjdk@17），或设置 JAVA_HOME"; exit 1; }
export PATH=$JAVA_HOME/bin:$PATH
command -v "$GODOT" >/dev/null || { echo "找不到 Godot 4.6.2（设置 GODOT=/path/to/godot）"; exit 1; }
[ -f "$APP/addons/godotopenxrvendors/plugin.gdextension" ] || bash "$HERE/fetch_addons.sh"

mkdir -p "$OUT"

# 导出要求编辑器设置里填 Java SDK / Android SDK 路径；为空时自动填上
ES=$(ls "$HOME/Library/Application Support/Godot"/editor_settings-4.*.tres "$HOME/.config/godot"/editor_settings-4.*.tres 2>/dev/null | tail -n1)
if [ -n "$ES" ]; then
  if grep -q '^export/android/java_sdk_path = ""' "$ES"; then
    echo "== 编辑器设置：java_sdk_path = $JAVA_HOME"
    sed -i.bak "s#^export/android/java_sdk_path = \"\"#export/android/java_sdk_path = \"$JAVA_HOME\"#" "$ES"
  fi
  SDK=${ANDROID_HOME:-$HOME/Library/Android/sdk}
  if grep -q '^export/android/android_sdk_path = ""' "$ES" && [ -d "$SDK" ]; then
    echo "== 编辑器设置：android_sdk_path = $SDK"
    sed -i.bak "s#^export/android/android_sdk_path = \"\"#export/android/android_sdk_path = \"$SDK\"#" "$ES"
  fi
fi

# Godot 无头模式退出时偶尔崩溃（EditorHelp 生成扩展文档），所以以产物为准，不看退出码
run_godot() { "$GODOT" --headless --path "$APP" "$@" 2>&1 | grep -v -E "^\s*$|5037|loading_editor_layout" || true; }

echo "== 导入资源"
run_godot --import

# 首次构建时安装 Android 构建模板（vr-app/android/），只能和 --export-* 一起用
TEMPLATE=()
[ -f "$APP/android/build/build.gradle" ] || TEMPLATE=(--install-android-build-template)

APK=$OUT/FrankaTeleop.apk
rm -f "$APK"
if [ "$MODE" = debug ]; then
  echo "== 导出调试版（首次需要下载 Gradle 依赖，约几分钟）"
  run_godot "${TEMPLATE[@]}" --export-debug "Quest 3S" "$APK"
else
  KS=${FRANKA_KEYSTORE:-$ROOT/keystore/franka-teleop.keystore}
  KS_USER=${FRANKA_KEYSTORE_USER:-frankateleop}
  if [ -z "${FRANKA_KEYSTORE_PASS:-}" ]; then
    PASS_FILE=$ROOT/keystore/franka-teleop.pass
    if [ ! -f "$KS" ]; then
      echo "== 生成发布签名 ${KS}（请备份 keystore/ 目录，丢失后无法覆盖安装升级）"
      mkdir -p "$(dirname "$KS")"
      (umask 077; openssl rand -hex 16 >"$PASS_FILE")
      keytool -genkeypair -keystore "$KS" -alias "$KS_USER" -keyalg RSA -keysize 2048 -validity 10000 \
        -storepass "$(cat "$PASS_FILE")" -keypass "$(cat "$PASS_FILE")" -dname "CN=Franka Teleop" >/dev/null
    fi
    FRANKA_KEYSTORE_PASS=$(cat "$PASS_FILE")
  fi
  export GODOT_ANDROID_KEYSTORE_RELEASE_PATH=$KS
  export GODOT_ANDROID_KEYSTORE_RELEASE_USER=$KS_USER
  export GODOT_ANDROID_KEYSTORE_RELEASE_PASSWORD=$FRANKA_KEYSTORE_PASS
  echo "== 导出发布版（首次需要下载 Gradle 依赖，约几分钟）"
  run_godot "${TEMPLATE[@]}" --export-release "Quest 3S" "$APK"
fi

[ -f "$APK" ] || { echo "导出失败，见上方日志"; exit 1; }
echo "== 完成：${APK}（$(du -h "$APK" | cut -f1)）"
