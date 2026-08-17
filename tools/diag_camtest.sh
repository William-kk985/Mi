#!/bin/bash
# 测试 camera_test 是否发布 topic + 枚举新 topic
source /etc/mi/ros2_env.conf 2>/dev/null
export LD_LIBRARY_PATH="/opt/ros2/cyberdog/lib:/opt/ros2/galactic/lib:$LD_LIBRARY_PATH"
NS=/mi_desktop_48_b0_2d_7b_02_c7

# 记录现有 topic
timeout 10 ros2 topic list 2>/dev/null | sort > /tmp/topics_before.txt

# 后台跑 camera_test cam_id=2
nohup /opt/ros2/cyberdog/lib/camera_test/camera_test 2 640 480 rgb > /tmp/camera_test_run.log 2>&1 &
CPID=$!
sleep 6

# 新 topic
timeout 10 ros2 topic list 2>/dev/null | sort > /tmp/topics_after.txt
echo "── camera_test 新发布的 topic:"
diff /tmp/topics_before.txt /tmp/topics_after.txt | grep "^>" || echo "  (无新 topic)"

echo "── camera_test 日志帧回调数(最近2s):"
grep -c "frame_callback" /tmp/camera_test_run.log

echo "── 新 topic 帧数:"
NEW=$(diff /tmp/topics_before.txt /tmp/topics_after.txt | grep "^>" | sed 's/^> //')
for t in $NEW; do
  timeout 8 python3 - "$t" <<'PY'
import rclpy, time, sys
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n = rclpy.create_node('chknew')
c = {'n': 0}
n.create_subscription(Image, sys.argv[1], lambda m: c.__setitem__('n', c['n']+1),
    QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT))
end = time.time() + 4
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
print('  %s: %d帧/4s' % (sys.argv[1], c['n']))
rclpy.shutdown()
PY
done

kill $CPID 2>/dev/null
