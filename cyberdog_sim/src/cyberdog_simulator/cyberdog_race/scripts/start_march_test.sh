#!/bin/bash
# start_march_test.sh — 原地踏步测试（需在 debug_config.hpp 定义 TEST_BEHAVIOR=11）
# 流程: 站起 → WALK_USERTROT(303) vel=0 原地踏步5秒 → 停止
set -e
source /etc/mi/ros2_env.conf 2>/dev/null
source /SDCARD/race_ws/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH="/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

# 防双开：清残留 + 等待8080释放（2026-08-07 完善，勿删 || true）
pkill -f 'race_controlle[r]' 2>/dev/null || true
for _ in $(seq 1 12); do
    ss -tln 2>/dev/null | grep -q ':8080 ' || break
    sleep 0.5
done
if ss -tln 2>/dev/null | grep -q ':8080 '; then
    echo "⚠ 8080 被占，强制释放..."
    fuser -k 8080/tcp 2>/dev/null || true
    sleep 1
fi

echo "🐕 原地踏步测试：站起 → 踏步5秒 → 停止"
echo "⚠ 请确认 debug_config.hpp 已定义: DEBUG_TEST_BEHAVIOR + TEST_BEHAVIOR=11"
exec /SDCARD/race_ws/install/lib/cyberdog_race/race_controller --ros-args -r __ns:=${NS}
