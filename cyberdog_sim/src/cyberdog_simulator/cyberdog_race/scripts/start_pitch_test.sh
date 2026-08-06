#!/bin/bash
# start_pitch_test.sh — 低头/抬头测试 (TEST_BEHAVIOR=7)
set -e
source /etc/mi/ros2_env.conf 2>/dev/null
source /SDCARD/race_ws/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH="/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

# 清理残留进程（否则8080端口被占用）
pkill -f race_controller 2>/dev/null
sleep 1

echo "🐕 俯仰角测试：低头15°→回正→抬头15°"
exec /SDCARD/race_ws/install/lib/cyberdog_race/race_controller --ros-args -r __ns:=${NS}
