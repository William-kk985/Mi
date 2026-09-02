#!/bin/bash
# 只读诊断: 枚举所有 Image 类型 topic 并测 3 秒帧数 (看还有没有别的 RGB 可用)
source /etc/mi/ros2_env.conf 2>/dev/null
source /SDCARD/race_ws/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages"

echo "── 1. 全部 image/camera 相关 topic:"
timeout 10 ros2 topic list 2>/dev/null | grep -iE "image|camera|rgb" | sort

echo
echo "── 2. 每个 Image 类型 topic 3 秒帧数:"
for t in $(timeout 10 ros2 topic list -t sensor_msgs/msg/Image 2>/dev/null); do
  timeout 8 python3 - "$t" <<'EOF'
import rclpy, time, sys
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n = rclpy.create_node('diag_rgb')
c = {'n': 0}
try:
    n.create_subscription(Image, sys.argv[1],
        lambda m: c.__setitem__('n', c['n'] + 1),
        QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT))
    end = time.time() + 3
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    print('  %-55s %d帧/3s' % (sys.argv[1], c['n']))
except Exception as e:
    print('  %-55s 订阅失败: %s' % (sys.argv[1], e))
rclpy.shutdown()
EOF
done

echo
echo "── 3. stereo_camera 状态:"
timeout 10 ros2 lifecycle get ${NS:-/mi_desktop_48_b0_2d_7b_02_c7}/stereo_camera 2>/dev/null || true
echo "── 4. dmesg VI no reply 计数:"
dmesg 2>/dev/null | grep -c 'no reply' || echo 0
