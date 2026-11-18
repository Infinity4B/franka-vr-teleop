#!/bin/bash
# 实时控制的系统配置（Franka 官方文档的要求），只需运行一次，可重复运行：
#   1. realtime 组：建组、把当前用户加进去，组内 rtprio 99、memlock 100 MB（改完要重新登录）
#   2. RT 内核开机后把所有 CPU 的 governor 设为 performance（关闭降频）
#   3. /dev/cpu_dma_latency 交给 realtime 组，start.sh 运行期间可以禁止 CPU 深度休眠
# 不负责安装 PREEMPT_RT 内核，见 docs/install.md。
# 用法: sudo bash host/setup_rt_sudo.sh          撤销 2、3 和本脚本写的 limits 文件: sudo bash host/setup_rt_sudo.sh undo
set -u
[ "$(id -u)" = 0 ] || { echo "请用 sudo 运行: sudo bash $0 $*"; exit 1; }
USR=${SUDO_USER:-}
SVC=/etc/systemd/system/franka-cpu-performance.service
RULE=/etc/udev/rules.d/99-franka-cpu-dma-latency.rules
LIMITS=/etc/security/limits.d/99-franka-realtime.conf

if [ "${1:-}" = undo ]; then
  echo "== 撤销"
  systemctl disable --now franka-cpu-performance.service 2>&1
  rm -fv "$SVC" "$RULE" "$LIMITS"
  systemctl daemon-reload
  udevadm control --reload 2>&1
  for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo powersave > "$g" || echo "FAILED: $g"; done
  chgrp root /dev/cpu_dma_latency && chmod 600 /dev/cpu_dma_latency
  echo "已恢复 powersave，cpu_dma_latency 恢复为仅 root 可写（realtime 组和组成员保留）"
  exit 0
fi

echo "== 1. realtime 组和 limits"
getent group realtime >/dev/null || groupadd realtime || { echo "FAILED: groupadd realtime"; exit 1; }
if [ -n "$USR" ] && [ "$USR" != root ]; then
  if id -nG "$USR" | grep -qw realtime; then echo "用户 $USR 已在 realtime 组"
  else usermod -aG realtime "$USR" && echo "已把 $USR 加入 realtime 组（重新登录后生效）" || echo "FAILED: usermod -aG realtime $USR"; fi
else
  echo "WARN: 不知道是哪个用户（没有 SUDO_USER），手动运行: usermod -aG realtime <用户名>"
fi
if grep -rqsE '^[[:space:]]*@realtime[[:space:]]+.*rtprio' /etc/security/limits.conf /etc/security/limits.d/ ; then
  echo "已有 @realtime 的 rtprio 配置，不重复写:"
  grep -rsE '^[[:space:]]*@realtime' /etc/security/limits.conf /etc/security/limits.d/
else
  cat > "$LIMITS" <<'LIM' && echo "已写入 $LIMITS" || echo "FAILED: 写 $LIMITS"
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
LIM
fi

echo "== 2. CPU governor：$SVC"
cat > "$SVC" <<'UNIT'
[Unit]
Description=Franka: CPU governor performance (RT kernel only)
ConditionPathExists=/sys/kernel/realtime
After=power-profiles-daemon.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > "$g"; done'
ExecStop=/bin/sh -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo powersave > "$g"; done'

[Install]
WantedBy=multi-user.target
UNIT
systemctl daemon-reload
systemctl enable franka-cpu-performance.service 2>&1
timeout 30 systemctl restart franka-cpu-performance.service || echo "FAILED: 启动 franka-cpu-performance.service（退出码 $?）"
[ -e /sys/kernel/realtime ] || echo "注意：当前不是 RT 内核，服务被跳过；进 RT 内核后自动生效"
echo "governor: $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort | uniq -c | tr -s ' ' | tr '\n' ' ')"

echo "== 3. /dev/cpu_dma_latency 交给 realtime 组：$RULE"
echo 'KERNEL=="cpu_dma_latency", GROUP="realtime", MODE="0660"' > "$RULE" || echo "FAILED: 写 $RULE"
udevadm control --reload || echo "FAILED: udevadm control --reload"
chgrp realtime /dev/cpu_dma_latency && chmod 660 /dev/cpu_dma_latency || echo "FAILED: 修改 /dev/cpu_dma_latency 权限"
ls -l /dev/cpu_dma_latency

echo; echo ">>> 完成。第一次加入 realtime 组的话要注销重新登录（ssh 重新连），然后运行 bash host/check_host.sh 检查"
