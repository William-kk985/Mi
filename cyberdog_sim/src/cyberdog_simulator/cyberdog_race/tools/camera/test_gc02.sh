#!/bin/bash
# cam_id=1 (gc02m1 bottomright) 测帧+存图, 看是彩色还是黑白
source /etc/mi/ros2_env.conf 2>/dev/null
export LD_LIBRARY_PATH="/opt/ros2/cyberdog/lib:/opt/ros2/galactic/lib:/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
NS=/mi_desktop_48_b0_2d_7b_02_c7
BIN=/SDCARD/race_ws/install/lib/cyberdog_race/center_cam

pkill -f "cyberdog_race/centercam" 2>/dev/null
pkill -f "cyberdog_race/center_cam" 2>/dev/null
sleep 3

for sync in true false; do
  nohup $BIN --ros-args -r __ns:=$NS -p cam_id:=1 -p sync:=$sync -p width:=640 -p height:=480 > /tmp/cc1_$sync.log 2>&1 &
  sleep 6
  echo "── cam_id=1 sync=$sync:"
  tail -2 /tmp/cc1_$sync.log
  timeout 8 python3 - "$NS" <<'PY'
import rclpy,time,cv2,sys
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n=rclpy.create_node('save')
b=CvBridge(); c={'img':None,'n':0}
def cb(m):
    c['n']+=1
    if c['img'] is None: c['img']=b.imgmsg_to_cv2(m,'bgr8')
n.create_subscription(Image,sys.argv[1]+'/image_center',cb,QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT))
end=time.time()+5
while time.time()<end and c['img'] is None: rclpy.spin_once(n,timeout_sec=0.05)
if c['img'] is not None:
    cv2.imwrite('/tmp/gc02.png',c['img'])
    b,g,r = c['img'][:,:,0].mean(),c['img'][:,:,1].mean(),c['img'][:,:,2].mean()
    sat = cv2.cvtColor(c['img'],cv2.COLOR_BGR2HSV)[:,:,1].mean()
    print('  存图 shape=%s BGR=%.0f/%.0f/%.0f S=%.1f (S<5=黑白)'%(c['img'].shape,b,g,r,sat))
else:
    print('  NO FRAME')
rclpy.shutdown()
PY
  pkill -f "cyberdog_race/centercam" 2>/dev/null
  pkill -f "cyberdog_race/center_cam" 2>/dev/null
  sleep 3
done
