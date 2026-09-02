#!/bin/bash
# 测 gc02m1(cam_id=1) 支持的分辨率
source /etc/mi/ros2_env.conf 2>/dev/null
export LD_LIBRARY_PATH="/opt/ros2/cyberdog/lib:/opt/ros2/galactic/lib:/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
BIN=/SDCARD/race_ws/install/lib/cyberdog_race/center_cam

pkill -f "cyberdog_race/centercam" 2>/dev/null
pkill -f "cyberdog_race/center_cam" 2>/dev/null
sleep 3

for wh in "1280 960" "1600 1200" "1280 720" "800 600" "640 480"; do
  set -- $wh
  nohup $BIN --ros-args -p cam_id:=1 -p sync:=false -p width:=$1 -p height:=$2 > /tmp/cc_res.log 2>&1 &
  sleep 6
  echo "── ${1}x${2}: $(grep -o 'Invalid active Mode Size\|frames per second\|Camera closed\|OpenCamera failed\|ret' /tmp/cc_res.log | sort -u | tr '\n' ' ')"
  pkill -f "cyberdog_race/centercam" 2>/dev/null
  pkill -f "cyberdog_race/center_cam" 2>/dev/null
  sleep 3
done
