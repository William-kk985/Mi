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

source "$ROS_ENV" 2>/dev/null || { echo "❌ 加载 $ROS_ENV 失败"; exit 1; }
# 必须直接 source（不要加管道，否则子 shell 环境变量传不回来）
# libg2o 警告只是 stderr 提示，不影响环境变量设置
source "$WS_SETUP" 2>/dev/null
export LD_LIBRARY_PATH="/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages"

echo "📷 启动 RGB..."
# ⚠ camera_service 不可用时（充电/未激活）ros2 service call 会无限等待 → 加 timeout 防阻塞
timeout 5 ros2 service call ${NS}/camera_service protocol/srv/CameraService \
    "{command: 9, args: \"\", width: 640, height: 480, fps: 30}" > /dev/null 2>&1 \
    || echo "⚠ camera_service 不可用，跳过 RGB 激活（其余流不受影响）"
sleep 1

for t in image scan odom_out camera/imu camera/infra1/image_rect_raw camera/depth/image_rect_raw bms_status touch_status; do
    PUB=$(ros2 topic info ${NS}/${t} 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    [ "$PUB" != "0" ] && echo "   ✅ $t" || echo "   ❌ $t"
done

echo ""
echo "🌐 http://192.168.44.1:8080"
echo ""

# 直接跑二进制（ros2 run 有 bug 会异常退出）
exec /SDCARD/race_ws/install/lib/cyberdog_race/race_controller --ros-args -r __ns:=${NS}
