#!/bin/bash
# cam_id=0 sync=false 测帧+存图
source /etc/mi/ros2_env.conf 2>/dev/null
export LD_LIBRARY_PATH="/opt/ros2/cyberdog/lib:/opt/ros2/galactic/lib:/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
NS=/mi_desktop_48_b0_2d_7b_02_c7
BIN=/SDCARD/race_ws/install/lib/cyberdog_race/center_cam

pkill -f "cyberdog_race/centercam" 2>/dev/null
pkill -f "cyberdog_race/center_cam" 2>/dev/null
sleep 3
nohup $BIN --ros-args -r __ns:=$NS -p cam_id:=0 -p sync:=false -p width:=640 -p height:=480 > /tmp/cc0b.log 2>&1 &
sleep 6
echo "── 日志:"
tail -3 /tmp/cc0b.log
timeout 10 python3 - "$NS" <<'PY'
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
end=time.time()+6
while time.time()<end and c['img'] is None: rclpy.spin_once(n,timeout_sec=0.05)
if c['img'] is not None:
    cv2.imwrite('/tmp/ov13_640.png',c['img'])
    print('存图 shape=%s 共%d帧'%(c['img'].shape,c['n']))
else:
    print('NO FRAME 共%d帧'%c['n'])
rclpy.shutdown()
PY
