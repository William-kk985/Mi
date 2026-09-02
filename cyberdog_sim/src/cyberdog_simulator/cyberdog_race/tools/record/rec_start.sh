#!/usr/bin/env bash
# 一键: 启动 center_cam (若没跑) → 等出帧 → 前台跑多段录制
# 用法 (NX):
#   bash /SDCARD/race_bins/rec_start.sh                    # 输出 /SDCARD/recs/rec_001.mp4 ...
#   bash /SDCARD/race_bins/rec_start.sh /SDCARD/recs 1.0 30   # 全分辨率30fps
# 录制控制: Enter=开始/停止一段  q=退出  Ctrl+C=退出
# (2026-08-21 从/tmp迁到/SDCARD/race_bins: /tmp重启会被清空)
set -u

NS=/mi_desktop_48_b0_2d_7b_02_c7
OUT_DIR=${1:-/SDCARD/recs}
SCALE=${2:-0.5}
FPS=${3:-20}

source /etc/mi/ros2_env.conf

# ── 1. center_cam 桥接节点 (cam_id=1 gc02m1 1280x960) ──
if pgrep -f "cyberdog_race/center_cam" > /dev/null; then
    echo "[rec] center_cam 已在跑"
else
    echo "[rec] 启动 center_cam..."
    pkill -f "cyberdog_race/center_cam" 2>/dev/null || true
    nohup /SDCARD/race_ws/install/lib/cyberdog_race/center_cam --ros-args \
        -r __ns:=${NS} -p cam_id:=1 -p sync:=false \
        -p width:=1280 -p height:=960 \
        > /tmp/center_cam.log 2>&1 &
    sleep 5
    CAM_PID=$(pgrep -f "cyberdog_race/center_cam" | head -1)
    if [ -n "$CAM_PID" ]; then
        taskset -pc 2,3 "$CAM_PID" > /dev/null 2>&1 && echo "[rec] center_cam 已绑核 2-3"
    fi
fi

# ── 2. 等 image_center 出帧 (最多20s) ──
echo "[rec] 等待 ${NS}/image_center 出帧..."
FRAMES=0
for i in $(seq 1 20); do
    FRAMES=$(timeout 5 python3 -c "
import rclpy, time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init()
n = rclpy.create_node('chk_frames')
c = {'n': 0}
n.create_subscription(Image, '${NS}/image_center',
    lambda m: c.__setitem__('n', c['n'] + 1),
    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
t0 = time.time()
while time.time() - t0 < 2.0:
    rclpy.spin_once(n, timeout_sec=0.1)
print(c['n'])
rclpy.shutdown()
" 2>/dev/null || echo 0)
    if [ "$FRAMES" -gt 0 ] 2>/dev/null; then
        echo "[rec] 出帧正常 (2s内 ${FRAMES} 帧)"
        break
    fi
    sleep 1
done
if [ "$FRAMES" -le 0 ] 2>/dev/null; then
    echo "[rec] ⚠ 20s 无帧! 查看 /tmp/center_cam.log 或先跑 start_race.sh"
    exit 1
fi

# ── 3. 前台多段录制 ──
echo "[rec] 开始录制 → ${OUT_DIR} (Enter=开始/停止  q=退出)"
python3 /SDCARD/race_bins/rec_video_multi.py -d "$OUT_DIR" -s "$SCALE" -f "$FPS"
echo "[rec] 结束. 拉回本地: scp cyberdog-wifi:${OUT_DIR}/rec_*.mp4 ~/Mi/"
