#!/bin/bash
# 只读诊断: /image 推流状态 + QoS + 双订阅者并发测试 (不动 controller, 狗不动)
source /etc/mi/ros2_env.conf 2>/dev/null
source /SDCARD/race_ws/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

echo "── 1. camera_server 进程:"
pgrep -af camera_server || echo "  ❌ 无进程"

echo "── 2. /image topic 详情 (QoS profile):"
timeout 10 ros2 topic info ${NS}/image -v 2>&1 | head -40

echo "── 3. 双 BEST_EFFORT 订阅者并发 8s:"
timeout 12 python3 - <<'EOF'
import rclpy,time
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n=rclpy.create_node('diag')
q=QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT)
c={'a':0,'b':0}
n.create_subscription(Image,'/mi_desktop_48_b0_2d_7b_02_c7/image',lambda m:c.__setitem__('a',c['a']+1),q)
n.create_subscription(Image,'/mi_desktop_48_b0_2d_7b_02_c7/image',lambda m:c.__setitem__('b',c['b']+1),q)
end=time.time()+8
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.05)
print('  subA=%d  subB=%d' % (c['a'],c['b']))
EOF

echo "── 4. dmesg VI no reply 计数:"
sudo dmesg 2>/dev/null | grep -c 'no reply' || dmesg 2>/dev/null | grep -c 'no reply' || echo 0

echo "── 5. 磁盘:"
df -h / | tail -1
echo "── 6. 负载:"
uptime
