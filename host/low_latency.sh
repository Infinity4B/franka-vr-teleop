#!/bin/bash
# 运行期间把 /dev/cpu_dma_latency 设为 0，禁止 CPU 进入深度休眠；命令结束（文件描述符关闭）后自动恢复。
# 用法: host/low_latency.sh <命令> [参数...]      start.sh 已经套了这一层
[ $# -gt 0 ] || { echo "用法: $0 <命令> [参数...]"; exit 1; }
if ! exec 3>/dev/cpu_dma_latency; then
  echo "没有 /dev/cpu_dma_latency 的写权限。先运行一次: sudo bash $(cd "$(dirname "$0")" && pwd)/setup_rt_sudo.sh"
  echo "（只测头显和网络时用 ./start.sh --dry-run，不需要这一步）"
  exit 1
fi
printf '0' >&3
"$@"
