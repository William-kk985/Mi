#!/bin/bash
# ============================================================
# start_web.sh — 全传感器 Web 推流
# 浏览器 http://192.168.44.1:8080
# ============================================================
set -e

ROS_ENV="/etc/mi/ros2_env.conf"
WS_SETUP="/SDCARD/race_ws/install/setup.bash"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

echo "============================================"
echo " CyberDog Web 推流"
echo "============================================"

source "$ROS_ENV" 2>/dev/null
source "$WS_SETUP" 2>/dev/null

echo "📷 启动 RGB..."
ros2 service call ${NS}/camera_service protocol/srv/CameraService \
    "{command: 9, args: \"\", width: 640, height: 480, fps: 30}" > /dev/null 2>&1
sleep 1

for t in image scan odom_out camera/imu camera/infra1/image_rect_raw camera/depth/image_rect_raw bms_status touch_status; do
    PUB=$(ros2 topic info ${NS}/${t} 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    [ "$PUB" != "0" ] && echo "   ✅ $t" || echo "   ❌ $t"
done

echo ""
echo "🌐 http://192.168.44.1:8080"
echo ""

ros2 run cyberdog_race race_controller --ros-args -r __ns:=${NS}
