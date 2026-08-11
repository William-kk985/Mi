#!/bin/bash
# ============================================================
# build_race.sh — 真机赛段一键编译 (VM 端)
# 流程: 同步源码(5文件) → 部署 start_race.sh → NX 编译
# 需要: 狗已开机(192.168.44.1), NX 密码(已配免密则无需)
# 用法(VM): bash ~/Mi/tools/build_race.sh
# 说明: debug_config.hpp 已 #define REAL_DOG → 编译即为真机版
# ============================================================
set -e

PKG=cyberdog_sim/src/cyberdog_simulator/cyberdog_race
SRC=$HOME/Mi/$PKG
NX_SRC=/SDCARD/race_ws/src/cyberdog_race

echo "============================================"
echo " CyberDog 真机编译 (同步源码 → NX 编译)"
echo "============================================"

# ── 1. 检查 NX 连通 (免密失败则交互输密码) ──
echo "① 检查 NX 连通..."
ssh -o ConnectTimeout=5 cyberdog 'true' || { echo "❌ NX 不可达, 请开机狗并检查网络"; exit 1; }
echo "   ✅ NX 已连通"

# ── 2. 同步源码 ──
echo "② 同步源码到 NX..."
scp -q "$SRC/CMakeLists.txt"                cyberdog:$NX_SRC/CMakeLists.txt
scp -q "$SRC/src/race_controller.cpp"       cyberdog:$NX_SRC/src/race_controller.cpp
scp -q "$SRC/src/stages/real/stage1_real.cpp"        cyberdog:$NX_SRC/src/stages/real/stage1_real.cpp
scp -q "$SRC/include/cyberdog_race/race_controller.hpp"          cyberdog:$NX_SRC/include/cyberdog_race/race_controller.hpp
scp -q "$SRC/include/cyberdog_race/stages/real/stage1_real.hpp"  cyberdog:$NX_SRC/include/cyberdog_race/stages/real/stage1_real.hpp
echo "   ✅ 源码同步完成 (5 文件)"

# ── 3. 部署运行脚本 ──
echo "③ 部署 start_race.sh 到 NX..."
scp -q "$SRC/scripts/start_race.sh" cyberdog:/home/mi/start_race.sh
echo "   ✅ 已部署 /home/mi/start_race.sh"

# ── 4. NX 编译 ──
echo "④ NX 编译 cyberdog_race (约 1-3 分钟)..."
ssh cyberdog 'source /etc/mi/ros2_env.conf 2>/dev/null; cd /SDCARD/race_ws && \
    export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic" && \
    export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages" && \
    colcon build --merge-install --packages-select cyberdog_race 2>&1 | tail -25'
echo "   ✅ 编译完成"

echo ""
echo "============================================"
echo " ✅ 编译完成! 在 NX 上运行:"
echo "   bash /home/mi/start_race.sh"
echo "   (或 VM: ssh -t cyberdog '\''bash /home/mi/start_race.sh'\'' )"
echo "============================================"
