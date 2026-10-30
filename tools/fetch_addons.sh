#!/bin/bash
# 下载 Godot OpenXR Vendors 插件到 vr-app/addons/godotopenxrvendors（48 MB，不进 git）。
# 版本固定 5.1.0-stable，核对 sha256。build_apk.sh 发现缺插件时自动调用，GitHub Actions 也用它。
set -e
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DEST=$(dirname "$HERE")/vr-app/addons
VERSION=5.1.0-stable
SHA256=6a838dbdf4115549e4511ebee0da9a5dcc8f9f6258d4cc2f2ee57a907a3e2911
URL=https://github.com/GodotVR/godot_openxr_vendors/releases/download/$VERSION/godotopenxrvendorsaddon.zip

if [ -f "$DEST/godotopenxrvendors/plugin.gdextension" ]; then
  echo "== 插件已存在：$DEST/godotopenxrvendors"
  exit 0
fi
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
echo "== 下载 OpenXR Vendors $VERSION"
curl -fL --retry 3 -o "$TMP/addon.zip" "$URL"
got=$( (sha256sum "$TMP/addon.zip" 2>/dev/null || shasum -a 256 "$TMP/addon.zip") | cut -d' ' -f1)
[ "$got" = "$SHA256" ] || { echo "sha256 不符：$got（期望 $SHA256）"; exit 1; }
unzip -q "$TMP/addon.zip" -d "$TMP/x"
mkdir -p "$DEST"
mv "$TMP/x/asset/addons/godotopenxrvendors" "$DEST/"
echo "== 完成：$DEST/godotopenxrvendors"
