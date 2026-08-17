#!/bin/bash
# NX 硬件资源总览 + 当前(不跑算法时)使用情况
echo "════════ CPU ════════"
echo "── 核数/型号:"
nproc
cat /proc/cpuinfo 2>/dev/null | grep -m1 "model name" || cat /proc/cpuinfo 2>/dev/null | grep -m1 "Hardware"
echo "── 负载:"
uptime

echo "════════ 内存 ════════"
free -h

echo "════════ 磁盘 ════════"
df -h / /SDCARD 2>/dev/null | head -4

echo "════════ GPU ════════"
nvidia-smi 2>/dev/null | head -12 || echo "(无 nvidia-smi)"

echo "════════ 进程 ════════"
echo "── 进程总数:"
ps aux | wc -l
echo "── race_controller 是否在跑:"
ps aux | grep race_controlle[r] | grep -v grep | head -2 || echo "  (没跑 ✅ 纯系统状态)"

echo "════════ CPU 占用 Top 12 ════════"
ps aux --sort=-%cpu | head -13 | awk '{printf "%-5s %6s%% %5s%%  %s\n", $2, $3, $4, $11}'

echo "════════ 汇总 CPU ════════"
top -bn1 2>/dev/null | head -5
