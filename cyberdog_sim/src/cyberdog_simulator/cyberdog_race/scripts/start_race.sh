#!/bin/bash
# ============================================================
# start_race.sh — 真机比赛一键启动 (NX 端)
# 流程: 防双开 → 激活 D430i(视觉) → 启动 race_controller (Stage1)
# 前置: 狗已开机, cyberdog_bringup 正常, 已用 build_race.sh 编译
# 用法(NX): bash /home/mi/start_race.sh
# ⚠ debug_config.hpp 已 #define REAL_DOG → 运行的是真机赛段
#   Stage1Real: 步高0.15 前进6m 巡线 + IMU 90°转弯
# 停止: Ctrl+C (狗会原地停); 全部关闭: VM 跑 stop_all.sh
# ============================================================
set -e

source /etc/mi/ros2_env.conf 2>/dev/null
source /SDCARD/race_ws/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH="/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

echo "============================================"
echo " CyberDog 真机比赛 (Stage1 石径探路)"
echo "============================================"

# ── 1. 防双开: 清旧 race_controller + 释放 8080 (2026-08-11) ──
pkill -f 'race_controlle[r]' 2>/dev/null || true
for _ in $(seq 1 12); do
    ss -tln 2>/dev/null | grep -q ':8080 ' || break
    sleep 0.5
done
if ss -tln 2>/dev/null | grep -q ':8080 '; then
    echo "  ⚠ 8080 仍被占用, 强制释放..."
    fuser -k 8080/tcp 2>/dev/null || true
    sleep 1
fi

# ── 2. 激活 D430i (比赛视觉巡线需要 RGB/深度) ──
echo "🔴 激活 D430i (红外+深度)..."
for node in camera/camera camera/camera_align; do
    STATE=$(timeout 5 ros2 lifecycle get ${NS}/${node} 2>/dev/null || true)
    case "$STATE" in
        *active*)            echo "  ✅ ${node} 已激活" ;;
        *unconfigured*|*inactive*)
            timeout 5 ros2 lifecycle set ${NS}/${node} configure > /dev/null 2>&1 || true
            timeout 5 ros2 lifecycle set ${NS}/${node} activate > /dev/null 2>&1 \
                && echo "  ✅ ${node} 激活成功" || echo "  ⚠ ${node} 激活失败/超时, 跳过" ;;
        *) echo "  ⚠ ${node} lifecycle 查询失败/超时, 跳过" ;;
    esac
done
sleep 1

# ── 3. 启动比赛 ──
echo "🚀 启动比赛 (Stage1: 前进6m 巡线 + IMU 90°转弯)"
echo "   狗将自动站起开始! 请保持场地空旷"
echo ""
exec /SDCARD/race_ws/install/lib/cyberdog_race/race_controller --ros-args -r __ns:=${NS}
