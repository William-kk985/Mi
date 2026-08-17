#!/bin/bash
# 查 /image 官方发布者 QoS 和订阅者情况 (NX 端)
source /etc/mi/ros2_env.conf 2>/dev/null
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

echo "── /image QoS (verbose):"
timeout 8 ros2 topic info $NS/image --verbose 2>/dev/null | head -30
echo "── /image_rgb QoS (对比 stereo_camera):"
timeout 8 ros2 topic info $NS/image_rgb --verbose 2>/dev/null | head -15
