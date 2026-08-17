#!/bin/bash
# 复现实验: 模拟 race_controller 的订阅集, 变量=绑核
source /etc/mi/ros2_env.conf 2>/dev/null
source /SDCARD/race_ws/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

cat > /tmp/diag_sub.py <<'EOF'
import rclpy,time,os,sys
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image,Imu,LaserScan
rclpy.init()
n=rclpy.create_node('diag_sub')
q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
c={'img':0}
def cb(m): c['img']+=1
n.create_subscription(Image,'/mi_desktop_48_b0_2d_7b_02_c7/image',cb,q)
n.create_subscription(Imu,'/mi_desktop_48_b0_2d_7b_02_c7/camera/imu',lambda m:None,q)
n.create_subscription(LaserScan,'/mi_desktop_48_b0_2d_7b_02_c7/scan',lambda m:None,10)
if len(sys.argv)>1 and sys.argv[1]=='bind':
    os.sched_setaffinity(0,{4,5}); tag='绑核4,5'
else:
    tag='不绑核'
end=time.time()+12
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.05)
print('[%s] /image 12s 收到 %d 帧' % (tag,c['img']))
rclpy.shutdown()
EOF

echo "── A. 模拟controller订阅集 + 绑核4,5 (12s):"
timeout 15 python3 /tmp/diag_sub.py bind 2>/dev/null

echo "── B. 模拟controller订阅集 + 不绑核 (12s):"
timeout 15 python3 /tmp/diag_sub.py 2>/dev/null
