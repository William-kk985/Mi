#!/bin/bash
# 存 infra1/infra2 帧
source /etc/mi/ros2_env.conf 2>/dev/null
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
NS=/mi_desktop_48_b0_2d_7b_02_c7
for T in infra1 infra2; do
  timeout 8 python3 - "$NS" "$T" <<'PY'
import rclpy,time,cv2,sys
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n=rclpy.create_node('save')
b=CvBridge(); c={'img':None,'n':0}
def cb(m):
    c['n']+=1
    if c['img'] is None: c['img']=b.imgmsg_to_cv2(m,'mono8')
n.create_subscription(Image,sys.argv[1]+'/camera/'+sys.argv[2]+'/image_rect_raw',cb,QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT))
end=time.time()+5
while time.time()<end and c['img'] is None: rclpy.spin_once(n,timeout_sec=0.05)
if c['img'] is not None:
    cv2.imwrite('/tmp/%s.png'%sys.argv[2],c['img'])
    print('%s 存图 shape=%s 共%d帧'%(sys.argv[2],c['img'].shape,c['n']))
else:
    print('%s NO FRAME 共%d帧'%(sys.argv[2],c['n']))
rclpy.shutdown()
PY
done
