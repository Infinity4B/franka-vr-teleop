#!/bin/bash
# 把网口切到 FR3 的连接（HOST_CIDR，参数见 host/host.env）。
# 要在主机桌面上的终端里运行：通过 ssh 运行 nmcli 会报 "Not authorized to control networking"（polkit 只允许本机桌面会话）。
# 用法: host/net_up.sh          切到 FR3 的连接
#       host/net_up.sh down     断开
#       host/net_up.sh create   第一次用：新建这个连接（静态 IP、不自动连接，不影响这个网口上原有的连接）
. "$(cd "$(dirname "$0")" && pwd)/host.env"
IF=$FRANKA_IFACE
case "${1:-up}" in
  down) exec nmcli con down "$FRANKA_CON" </dev/null ;;
  create)
    if nmcli -t -f NAME con show </dev/null | grep -qx "$FRANKA_CON"; then echo "连接 $FRANKA_CON 已存在"; exit 0; fi
    nmcli con add type ethernet con-name "$FRANKA_CON" ifname "$IF" ipv4.method manual ipv4.addresses "$HOST_CIDR" \
      ipv6.method disabled connection.autoconnect no </dev/null || exit 1
    echo "已新建 $FRANKA_CON（$IF，$HOST_CIDR）。接好网线后运行: host/net_up.sh" ; exit 0 ;;
  up) ;;
  *) echo "用法: $0 [up|down|create]"; exit 1 ;;
esac
[ "$(cat /sys/class/net/$IF/carrier 2>/dev/null)" = 1 ] || { echo "$IF 没有检测到网线（carrier=0），先插好网线"; exit 1; }
nmcli -t -f NAME con show </dev/null | grep -qx "$FRANKA_CON" || { echo "没有连接 $FRANKA_CON，先运行: $0 create"; exit 1; }
nmcli con up "$FRANKA_CON" ifname "$IF" </dev/null || exit 1
ip -br addr show "$IF"
ping -c 2 -W 1 -I "$IF" "$FRANKA_IP" >/dev/null 2>&1 && echo "FR3（$FRANKA_IP）能 ping 通" \
  || echo "FR3（$FRANKA_IP）ping 不通：检查网线是否接在 Control 柜上、Control 是否开机、Desk 里的 IP 是不是 $FRANKA_IP"
