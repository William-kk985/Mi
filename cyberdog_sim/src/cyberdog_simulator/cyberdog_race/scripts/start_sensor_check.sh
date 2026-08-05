#!/bin/bash
# ============================================================
# start_sensor_check.sh — 逐传感器确认+全链路检查
# 依赖: debug_config.hpp 中 DEBUG_TEST_BEHAVIOR + TEST_BEHAVIOR=9
# ============================================================
set -e

ROS_ENV="/etc/mi/ros2_env.conf"
WS_SETUP="/SDCARD/race_ws/install/setup.bash"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

green() { echo -e "\033[1;32m$1\033[0m"; }
red()   { echo -e "\033[1;31m$1\033[0m"; }

echo "============================================"
echo " CyberDog 传感器逐项检查"
echo "============================================"

source "$ROS_ENV" 2>/dev/null
source "$WS_SETUP" 2>/dev/null

# ── 1. RGB ──
echo "📷 1/8 RGB 相机..."
ros2 service call ${NS}/camera_service protocol/srv/CameraService \
    "{command: 9, args: \"\", width: 640, height: 480, fps: 30}" > /dev/null 2>&1
sleep 1
PUB=$(ros2 topic info ${NS}/image 2>/dev/null | grep "Publisher count" | awk '{print $3}')
[ "$PUB" != "0" ] && green "   ✅ RGB (Pub=$PUB)" || red "   ❌ RGB"
echo ""

# ── 2. LiDAR ──
echo "📡 2/8 LiDAR..."
PUB=$(ros2 topic info ${NS}/scan 2>/dev/null | grep "Publisher count" | awk '{print $3}')
[ "$PUB" != "0" ] && green "   ✅ scan (Pub=$PUB)" || red "   ❌ scan"
echo ""

# ── 3. 里程计 ──
echo "📍 3/8 里程计..."
PUB=$(ros2 topic info ${NS}/odom_out 2>/dev/null | grep "Publisher count" | awk '{print $3}')
[ "$PUB" != "0" ] && green "   ✅ odom_out (Pub=$PUB)" || red "   ❌ odom_out"
echo ""

# ── 4. IMU (D430i内置) ──
echo "🧭 4/8 IMU (D430i camera/imu)..."
PUB=$(ros2 topic info ${NS}/camera/imu 2>/dev/null | grep "Publisher count" | awk '{print $3}')
[ "$PUB" != "0" ] && green "   ✅ camera/imu (Pub=$PUB)" || red "   ❌ camera/imu"
echo ""

# ── 5. D430i 红外 ──
echo "🔴 5/8 D430i 红外..."
PUB=$(ros2 topic info ${NS}/camera/infra1/image_rect_raw 2>/dev/null | grep "Publisher count" | awk '{print $3}')
[ "$PUB" != "0" ] && green "   ✅ infra1 (Pub=$PUB)" || red "   ❌ infra1"
echo ""

# ── 6. D430i 深度 ──
echo "🟦 6/8 D430i 深度..."
PUB=$(ros2 topic info ${NS}/camera/depth/image_rect_raw 2>/dev/null | grep "Publisher count" | awk '{print $3}')
[ "$PUB" != "0" ] && green "   ✅ depth (Pub=$PUB)" || red "   ❌ depth"
echo ""

# ── 7. BMS 电池 ──
echo "🔋 7/8 BMS 电池..."
PUB=$(ros2 topic info ${NS}/bms_status 2>/dev/null | grep "Publisher count" | awk '{print $3}')
[ "$PUB" != "0" ] && green "   ✅ bms_status (Pub=$PUB)" || red "   ❌ bms_status"
echo ""

# ── 8. TOF/Touch/Motion ──
echo "✋ 8/8 TOF+Touch+运动..."
for t in touch_status head_tof_payload rear_tof_payload motion_status; do
    PUB=$(ros2 topic info ${NS}/${t} 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    [ "$PUB" != "0" ] && green "   ✅ $t (Pub=$PUB)" || red "   ❌ $t"
done
echo ""

echo "============================================"
echo " 🔍 运行传感器数据收集测试（5s）..."
echo "============================================"
ros2 run cyberdog_race race_controller --ros-args -r __ns:=${NS}
