#!/bin/bash
# ============================================================
# build_race.sh — 真机一键编译 (VM 端)
# 流程: 安全同步源码(sync_to_nx.sh, 保护伙伴改动) → NX 编译
# 需要: 狗已开机, 已配 SSH 免密
# 用法(VM): bash ~/Mi/tools/build_race.sh
# 说明:
#   · 自动探测连接: 有线 cyberdog(192.168.44.1) 不通 → 自动切 WiFi cyberdog-wifi
#   · debug_config.hpp 已 #define REAL_DOG → 编译即为真机版
#   · 编译必须在 NX 上 (VM=x86 / NX=aarch64, 架构不同不能交叉用)
#   · 同步走 sync_to_nx.sh: 不带 --delete, 保护 NX 上伙伴改过未提交的文件
# ============================================================
set -e

PKG_DIR="$HOME/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race"

# ── 自动探测 NX 连接: 有线优先, 不通则 WiFi (2026-08-11) ──
NX_HOST="cyberdog"   # 有线 192.168.44.1
if ! timeout 4 ssh -o ConnectTimeout=3 -o BatchMode=yes cyberdog 'true' 2>/dev/null; then
    NX_HOST="cyberdog-wifi"   # WiFi 10.179.x
fi
export NX_HOST

echo "============================================"
echo " CyberDog 真机编译 (安全同步 → NX 编译)"
echo "============================================"

# ── 1. 检查 NX 连通 ──
echo "① 检查 NX 连通 (主机: $NX_HOST)..."
ssh -o ConnectTimeout=5 "$NX_HOST" 'true' || { echo "❌ NX 不可达, 请开机狗并检查网络"; exit 1; }
echo "   ✅ NX 已连通"

# ── 2. 安全同步源码 (sync_to_nx.sh: 不删文件, 跳过伙伴未提交改动) ──
echo "② 安全同步源码 (保护伙伴改动)..."
bash "$PKG_DIR/scripts/sync_to_nx.sh"
echo "   ✅ 源码同步完成 (含 scripts/ 启动脚本)"

# ── 3. NX 编译 (aarch64, 必须在 NX 上) ──
#    全力模式 (2026-08-16): -j4 并行; 内存不足被杀 → 自动降级 -j2 → -j1
echo "③ NX 编译 cyberdog_race (-j4 全力, 自动降级)..."
ssh "$NX_HOST" 'source /etc/mi/ros2_env.conf 2>/dev/null; cd /SDCARD/race_ws && \
    export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic" && \
    export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages" && \
    RC=1; for J in 4 2 1; do \
      echo "═══ 编译 MAKEFLAGS=-j$J ═══"; \
      MAKEFLAGS=-j$J colcon build --merge-install --packages-select cyberdog_race --executor sequential > /tmp/build_out.log 2>&1; RC=$?; \
      tail -8 /tmp/build_out.log; \
      if [ $RC -eq 0 ]; then echo "✅ 编译成功 (-j$J)"; break; fi; \
      if grep -qE "Terminated|Killed" /tmp/build_out.log; then echo "⚠ 内存不足被杀, 降级重试..."; continue; fi; \
      echo "❌ 编译错误(代码问题), 详见 /tmp/build_out.log"; break; \
    done; exit $RC'
echo "   ✅ 编译完成"

echo ""
echo "============================================"
echo " ✅ 完成! 在 NX 上运行:"
echo "   bash /SDCARD/race_ws/src/cyberdog_race/scripts/start_race.sh"
echo "============================================"
