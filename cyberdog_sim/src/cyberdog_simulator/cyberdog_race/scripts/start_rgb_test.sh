#!/bin/bash
# ============================================================
# start_rgb_test.sh — RGB 相机实时预览（cv::imshow 弹窗）
# 用法: ssh -X cyberdog 后运行此脚本
# 依赖: debug_config.hpp 中 DEBUG_TEST_BEHAVIOR + TEST_BEHAVIOR=10
# ============================================================
set -e

ROS_ENV="/etc/mi/ros2_env.conf"
WS_SETUP="/SDCARD/race_ws/install/setup.bash"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

echo "============================================"
echo " CyberDog RGB 相机实时预览"
echo "============================================"

# 1. 加载 ROS2 环境
if [ -f "$ROS_ENV" ]; then
    source "$ROS_ENV"
else
    echo "❌ $ROS_ENV 不存在"
    exit 1
fi

# 2. 加载 workspace
if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
else
    echo "❌ $WS_SETUP 不存在，先编译: colcon build --merge-install --packages-select cyberdog_race"
    exit 1
fi

# 3. 启动 RGB 相机推流（⚠ timeout 防 camera_service 不可用时无限等待）
echo "📷 启动 RGB 相机推流..."
timeout 5 ros2 service call ${NS}/camera_service protocol/srv/CameraService \
    "{command: 9, args: \"\", width: 640, height: 480, fps: 30}" \
    > /dev/null 2>&1 || echo "⚠ camera_service 不可用，跳过"
sleep 1

# 4. 验证 image topic
PUB=$(ros2 topic info ${NS}/image 2>/dev/null | grep "Publisher count" | awk '{print $3}')
if [ "$PUB" != "0" ] && [ -n "$PUB" ]; then
    echo "✅ RGB 相机推流中 (Publisher=$PUB)"
else
    echo "❌ RGB 相机未推流"
    exit 1
fi

# 4.5 防双开（清理残留进程，避免 8080 被占）
pkill -f 'race_controlle[r]' 2>/dev/null || true
sleep 1

# 5. 运行 race_controller（TEST_BEHAVIOR=10: cv::imshow 实时预览）
echo "🎥 启动实时预览（按 Ctrl+C 退出）..."
echo "   💡 提示: 需 ssh -X 转发 X11 才能弹窗"
echo ""
# ⚠ 直接跑二进制（ros2 run 有 bug 会异常退出）
exec /SDCARD/race_ws/install/lib/cyberdog_race/race_controller --ros-args -r __ns:=${NS}
