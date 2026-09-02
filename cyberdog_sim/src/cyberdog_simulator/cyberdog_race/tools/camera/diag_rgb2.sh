#!/bin/bash
# 只读诊断: 显式枚举所有可能的 RGB/图像通道, 测4秒帧数+编码+分辨率
source /etc/mi/ros2_env.conf 2>/dev/null
source /SDCARD/race_ws/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

for t in image image_left image_right image_rgb \
         camera/infra1/image_rect_raw camera/infra2/image_rect_raw \
         camera/depth/image_rect_raw camera/aligned_depth_to_extcolor/image_raw; do
  timeout 10 python3 - "$NS/$t" <<'PY'
import rclpy, time, sys
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init()
n = rclpy.create_node('diag_rgb2')
c = {'n': 0, 'info': ''}
def cb(m):
    c['n'] += 1
    if not c['info']:
        c['info'] = '%s %dx%d' % (m.encoding, m.width, m.height)
n.create_subscription(Image, sys.argv[1], cb,
    QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT))
end = time.time() + 4
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
print('  %-62s %3d帧/4s  %s' % (sys.argv[1], c['n'], c['info'] or '无帧'))
rclpy.shutdown()
PY
done
