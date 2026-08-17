#!/bin/bash
# 快速验证: gc02m1 1280x960 帧率
source /etc/mi/ros2_env.conf 2>/dev/null
export LD_LIBRARY_PATH="/opt/ros2/cyberdog/lib:/opt/ros2/galactic/lib:/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
NS=/mi_desktop_48_b0_2d_7b_02_c7
BIN=/SDCARD/race_ws/install/lib/cyberdog_race/center_cam

pkill -f "cyberdog_race/centercam" 2>/dev/null
pkill -f "cyberdog_race/center_cam" 2>/dev/null
sleep 3
nohup $BIN --ros-args -r __ns:=$NS -p cam_id:=1 -p sync:=false -p width:=1280 -p height:=960 > /tmp/cc_v.log 2>&1 &
sleep 6
timeout 10 python3 - "$NS" <<'PY'
import rclpy,time,sys
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n=rclpy.create_node('v')
q=QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT)
c={'n':0}
n.create_subscription(Image,sys.argv[1]+'/image_center',lambda m: c.__setitem__('n',c['n']+1),q)
end=time.time()+5
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.05)
print('1280x960 5s收帧 =', c['n'])
rclpy.shutdown()
PY
pkill -f "cyberdog_race/centercam" 2>/dev/null
pkill -f "cyberdog_race/center_cam" 2>/dev/null
