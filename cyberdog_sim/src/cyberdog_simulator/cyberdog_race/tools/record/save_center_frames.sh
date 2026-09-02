#!/bin/bash
# cam_id 2/3 各存一帧到 /tmp/center_2.png /tmp/center_3.png
source /etc/mi/ros2_env.conf 2>/dev/null
export LD_LIBRARY_PATH="/opt/ros2/cyberdog/lib:/opt/ros2/galactic/lib:/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
NS=/mi_desktop_48_b0_2d_7b_02_c7
BIN=/SDCARD/race_ws/install/lib/cyberdog_race/center_cam

for CID in 2 3; do
  pkill -f "cyberdog_race/center_cam" 2>/dev/null
  sleep 3
  nohup $BIN --ros-args -r __ns:=$NS -p cam_id:=$CID -p width:=640 -p height:=480 > /tmp/cc_cid$CID.log 2>&1 &
  sleep 6
  timeout 8 python3 - "$NS" "$CID" <<'PY'
import rclpy,time,cv2,sys
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n=rclpy.create_node('save')
b=CvBridge(); c={'img':None}
def cb(m):
    if c['img'] is None: c['img']=b.imgmsg_to_cv2(m,'bgr8')
n.create_subscription(Image,sys.argv[1]+'/image_center',cb,QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT))
end=time.time()+6
while time.time()<end and c['img'] is None: rclpy.spin_once(n,timeout_sec=0.05)
if c['img'] is not None:
    cv2.imwrite('/tmp/center_%s.png'%sys.argv[2],c['img'])
    print('saved cam_id=%s'%sys.argv[2])
else:
    print('NO FRAME cam_id=%s'%sys.argv[2])
rclpy.shutdown()
PY
done
pkill -f "cyberdog_race/center_cam" 2>/dev/null
