#!/bin/bash
# 只读自检：实时内核和权限、Python 环境、防火墙、FR3 网络、头显网络。不需要 sudo，不改任何东西。
# 用法: bash host/check_host.sh        参数见 host/host.env
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$HERE")
. "$HERE/host.env"
nfail=0
ok()   { echo "  [OK]   $*"; }
warn() { echo "  [WARN] $*"; }
fail() { echo "  [FAIL] $*"; nfail=$((nfail + 1)); }

echo "== 实时内核"
echo "  $(uname -r)"
[ "$(cat /sys/kernel/realtime 2>/dev/null)" = 1 ] && ok "PREEMPT_RT（/sys/kernel/realtime=1）" \
  || fail "当前不是实时内核，开机时在 GRUB 里选 RT 内核（docs/install.md）"

echo "== 实时权限（sudo bash host/setup_rt_sudo.sh）"
id -nG | grep -qw realtime && ok "用户在 realtime 组" || fail "用户不在 realtime 组（运行过脚本的话，要注销重新登录）"
[ "$(ulimit -r)" -ge 99 ] 2>/dev/null && ok "rtprio=$(ulimit -r)" || fail "rtprio=$(ulimit -r)，应为 99"
{ [ "$(ulimit -l)" = unlimited ] || [ "$(ulimit -l)" -ge 102400 ] 2>/dev/null; } && ok "memlock=$(ulimit -l)" || fail "memlock=$(ulimit -l)，应 >= 102400"
gov=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort | uniq -c | tr -s ' ' | tr '\n' ' ')
echo "$gov" | grep -q powersave && fail "governor: $gov" || ok "governor: $gov"
[ -w /dev/cpu_dma_latency ] && ok "/dev/cpu_dma_latency 可写" || fail "/dev/cpu_dma_latency 不可写，start.sh 会拒绝启动（--dry-run 除外）"

echo "== Python 环境（uv sync）"
if command -v uv >/dev/null; then ok "$(uv --version)"; else fail "没有 uv：curl -LsSf https://astral.sh/uv/install.sh | sh"; fi
if [ -x "$ROOT/.venv/bin/python" ]; then
  v=$("$ROOT/.venv/bin/python" -c "import franky, importlib.metadata as m; print(m.version('franky-control'))" 2>&1) \
    && ok "franky-control $v（libfranka 版本要和机器人的 Robot Server 对应，docs/install.md）" || fail "import franky 失败：$v"
else
  fail "还没有 .venv，在 $ROOT 里运行 uv sync"
fi

echo "== 防火墙"
if grep -qs '^ENABLED=yes' /etc/ufw/ufw.conf; then
  warn "ufw 开着：确认运行过 sudo bash host/setup_firewall_sudo.sh（规则要 root 才能看，这里查不到）"
else
  ok "ufw 没开"
fi

echo "== FR3 网络（$FRANKA_IFACE → $FRANKA_IP）"
if [ "$(cat /sys/class/net/$FRANKA_IFACE/carrier 2>/dev/null)" = 1 ]; then
  ok "$FRANKA_IFACE 已插网线，$(cat /sys/class/net/$FRANKA_IFACE/speed 2>/dev/null) Mb/s"
  ip -4 -o addr show "$FRANKA_IFACE" | grep -q " ${HOST_CIDR%/*}/" && ok "$FRANKA_IFACE 上有 ${HOST_CIDR}" \
    || fail "$FRANKA_IFACE 上没有 $HOST_CIDR，在桌面终端运行 host/net_up.sh"
  dev=$(ip route get "$FRANKA_IP" 2>/dev/null | grep -o 'dev [^ ]*' | cut -d' ' -f2)
  [ "$dev" = "$FRANKA_IFACE" ] && ok "到 $FRANKA_IP 的路由走 $dev" || fail "到 $FRANKA_IP 的路由走 ${dev:-（没有）}，不是 $FRANKA_IFACE（VPN 接管了？）"
  ping -c 2 -W 1 "$FRANKA_IP" >/dev/null 2>&1 && ok "ping $FRANKA_IP 通" || fail "ping $FRANKA_IP 不通"
  timeout 2 bash -c "</dev/tcp/$FRANKA_IP/1337" 2>/dev/null && ok "TCP 1337 能连（FCI 端口）" \
    || warn "TCP 1337 连不上：Desk 里还没激活 FCI，或机器人没开机"
else
  warn "$FRANKA_IFACE 没插网线（只测头显时不用管）"
fi

echo "== 头显网络（UDP $VR_PORT）"
if [ -n "$VR_IFACE" ]; then
  addr=$(ip -4 -o addr show "$VR_IFACE" 2>/dev/null | awk '{print $4}' | head -1)
  [ -n "$addr" ] && ok "$VR_IFACE 的地址 ${addr%/*}（发现不了时在头显设置面板里手动填这个 IP）" \
    || fail "$VR_IFACE 没有 IPv4 地址，先连上和头显同一个 Wi-Fi"
else
  warn "没找到无线网卡，在 host/host.env 里填 VR_IFACE"
fi
ss -Hlun "sport = :$VR_PORT" 2>/dev/null | grep -q . && warn "UDP $VR_PORT 已被占用：已经有一个适配器在运行？" || ok "UDP $VR_PORT 空闲"

echo
[ $nfail = 0 ] && echo "全部通过" || echo "$nfail 项 FAIL"
