#!/bin/bash
# 循环测试 center_cam 各分辨率出帧数 (cam_id=2 center 相机)
source /etc/mi/ros2_env.conf 2>/dev/null
export LD_LIBRARY_PATH="/opt/ros2/cyberdog/lib:/opt/ros2/galactic/lib:/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
NS=/mi_desktop_48_b0_2d_7b_02_c7
BIN=/SDCARD/race_ws/install/lib/cyberdog_race/center_cam

for wh in "640 480" "800 600" "1280 720" "1280 800" "1280 960" "640 360"; do
  set -- $wh
  W=$1; H=$2
  # 清理旧进程(模式不写本脚本自身命令行里没有 camera 字样? 用 pidfile 方式)
  pkill -f "cyberdog_race/center_cam" 2>/dev/null
  sleep 3
  nohup $BIN --ros-args -r __ns:=$NS -p cam_id:=2 -p width:=$W -p height:=$H > /tmp/cc_$W.log 2>&1 &
  sleep 6
  ALIVE=$(pgrep -f "cyberdog_race/center_cam" | head -1)
  if [ -z "$ALIVE" ]; then
    echo "[${W}x${H}] 进程已死"
    grep -iE "FAILED|ret=|Invalid" /tmp/cc_$W.log | head -2
  else
    N=$(timeout 8 python3 -c "
import rclpy,time
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n=rclpy.create_node('cnt')
c={'n':0}
n.create_subscription(Image,'$NS/image_center',lambda m:c.__setitem__('n',c['n']+1),QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT))
end=time.time()+5
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.05)
print(c['n'])
rclpy.shutdown()
")
    echo "[${W}x${H}] 存活, 5s收到 ${N} 帧"
  fi
done
pkill -f "cyberdog_race/center_cam" 2>/dev/null
