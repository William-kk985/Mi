#!/bin/bash
# 诊断: /image 是否出帧 + race_controller 状态 (NX 端)
source /etc/mi/ros2_env.conf 2>/dev/null
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

echo "── 1. /image 帧数 (3s):"
timeout 8 python3 -c "
import rclpy,time
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n=rclpy.create_node('chks3')
q=QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT)
c={'n':0}
n.create_subscription(Image, '$NS/image', lambda m: c.__setitem__('n',c['n']+1), q)
end=time.time()+3
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.05)
print('frames=',c['n'])
" 2>/dev/null
echo "── 2. race_controller / camera_server 进程:"
ps aux | grep -E "race_controlle[r]|camera_serve[r]" | grep -v grep | awk '{print "  ", $2, $11}' | head -4
echo "── 3. camera_server 日志 (最近):"
tail -4 /tmp/camera_server.log 2>/dev/null || echo "  (无日志, 可能是 bringup 拉的)"
