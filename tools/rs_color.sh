#!/bin/bash
# 设备已解锁: 重启 realsense2_camera_node → configure → activate → 查 color topic
source /etc/mi/ros2_env.conf 2>/dev/null
export LD_LIBRARY_PATH="/opt/ros2/cyberdog/lib:/opt/ros2/galactic/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
NS=/mi_desktop_48_b0_2d_7b_02_c7

nohup /opt/ros2/cyberdog/lib/realsense2_camera/realsense2_camera_node \
  --ros-args --log-level info --ros-args -r __ns:=${NS}/camera \
  --params-file /tmp/launch_params_x5zwbrrd > /tmp/rs_cam.log 2>&1 &
sleep 6
echo "── camera_node 日志:"
tail -4 /tmp/rs_cam.log
echo "── configure:"
timeout 20 ros2 lifecycle set ${NS}/camera/camera configure 2>&1
echo "── activate:"
timeout 30 ros2 lifecycle set ${NS}/camera/camera activate 2>&1
sleep 5
echo "── color topic:"
timeout 10 ros2 topic list 2>/dev/null | grep -iE "color"
echo "── color 帧数:"
timeout 8 python3 - "$NS" <<'PY'
import rclpy,time,sys
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n=rclpy.create_node('chkcolor')
c={'n':0,'info':''}
def cb(m):
    c['n']+=1
    if not c['info']: c['info']='%s %dx%d'%(m.encoding,m.width,m.height)
n.create_subscription(Image,sys.argv[1]+'/camera/color/image_raw',cb,QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT))
end=time.time()+5
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.05)
print('  %d帧/5s  %s'%(c['n'],c['info'] or '无帧'))
rclpy.shutdown()
PY
