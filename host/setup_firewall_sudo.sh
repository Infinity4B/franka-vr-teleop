#!/bin/bash
# ufw 默认拒绝入站，会挡住两类包，这里只放行它们（可重复运行，立即生效）：
#   1. FR3 发来的 UDP 状态包（不放行时 libfranka / franky 报 "UDP receive: Timeout"）：
#      FRANKA_IFACE 上、来自 FRANKA_IP 的 UDP，目标端口每次连接随机，所以不限端口
#   2. 头显发给适配器的 UDP VR_PORT（发现、控制共用）：VR_IFACE 上、来自本网段的包
# 参数在 host/host.env。ufw 没开时什么都不做。
# 用法: sudo bash host/setup_firewall_sudo.sh        撤销: sudo bash host/setup_firewall_sudo.sh undo
set -u
[ "$(id -u)" = 0 ] || { echo "请用 sudo 运行: sudo bash $0 $*"; exit 1; }
. "$(cd "$(dirname "$0")" && pwd)/host.env"
command -v ufw >/dev/null || { echo "没有装 ufw，不需要放行"; exit 0; }
if ! timeout 30 ufw status | grep -q 'Status: active'; then echo "ufw 没有开启，不需要放行"; exit 0; fi

SUBNET=
[ -n "$VR_IFACE" ] && SUBNET=$(ip -4 -o addr show "$VR_IFACE" 2>/dev/null | awk '{print $4}' | head -1 \
  | python3 -c "import ipaddress,sys; s=sys.stdin.read().strip(); print(ipaddress.ip_interface(s).network if s else '')")
echo "== FR3：$FRANKA_IFACE 上来自 $FRANKA_IP 的 UDP"
echo "== 头显：${VR_IFACE:-（没找到无线网卡）} 上来自 ${SUBNET:-（没有 IPv4 地址）} 的 UDP $VR_PORT"

if [ "${1:-}" = undo ]; then
  timeout 60 ufw delete allow in on "$FRANKA_IFACE" from "$FRANKA_IP" proto udp || echo "FAILED: 删除 FR3 规则（退出码 $?）"
  if [ -n "$SUBNET" ]; then
    timeout 60 ufw delete allow in on "$VR_IFACE" from "$SUBNET" to any port "$VR_PORT" proto udp || echo "FAILED: 删除头显规则（退出码 $?）"
  fi
else
  timeout 60 ufw allow in on "$FRANKA_IFACE" from "$FRANKA_IP" proto udp comment 'Franka FR3 FCI UDP' \
    || echo "FAILED: 放行 FR3（退出码 $?）"
  if [ -n "$SUBNET" ]; then
    timeout 60 ufw allow in on "$VR_IFACE" from "$SUBNET" to any port "$VR_PORT" proto udp comment 'franka_vr_bridge (Quest)' \
      || echo "FAILED: 放行头显（退出码 $?）"
  else
    echo "FAILED: 头显规则没加。先连上和头显同一个 Wi-Fi，或在 host/host.env 里填 VR_IFACE，再重新运行"
  fi
fi
echo "== 当前 ufw 规则"
timeout 30 ufw status numbered || echo "FAILED: ufw status（退出码 $?）"
