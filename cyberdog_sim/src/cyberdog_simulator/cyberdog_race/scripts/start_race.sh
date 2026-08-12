#!/bin/bash
# ============================================================
# start_race.sh — 真机比赛一键启动 (NX 端, 源码 scripts/ 目录)
# 流程: 防双开 → 激活 D430i(视觉) → 启动 race_controller (Stage1)
# 前置: 狗已开机, cyberdog_bringup 正常, 已用 build_race.sh 编译
# 用法(NX): bash /SDCARD/race_ws/src/cyberdog_race/scripts/start_race.sh
#   (由 build_race.sh 的 sync_to_nx.sh 同步到 NX)
# ⚠ debug_config.hpp 已 #define REAL_DOG → 运行的是真机赛段
#   Stage1Real: 步高0.15 前进6m 巡线 + IMU 90°转弯
# 停止: Ctrl+C (狗会原地停); 全部关闭: VM 跑 stop_all.sh
# ============================================================
set -e

source /etc/mi/ros2_env.conf 2>/dev/null
source /SDCARD/race_ws/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH="/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

echo "============================================"
echo " CyberDog 真机比赛 (Stage1 石径探路)"
echo "============================================"

# ── 1. 防双开: 清旧 race_controller + 释放 8080 (2026-08-11) ──
pkill -f 'race_controlle[r]' 2>/dev/null || true
for _ in $(seq 1 12); do
    ss -tln 2>/dev/null | grep -q ':8080 ' || break
    sleep 0.5
done
if ss -tln 2>/dev/null | grep -q ':8080 '; then
    echo "  ⚠ 8080 仍被占用, 强制释放..."
    fuser -k 8080/tcp 2>/dev/null || true
    sleep 1
fi

# ── 2. 启动 RGB 推流: camera_service → camera_server 发布 /image (2026-08-13) ──
# ★ stereo_camera 直读 VI 的采集管线已坏 (no reply from camera processor, 0帧);
#   camera_server 推流管线正常 (/image bgr8 640x480 ~21fps) → 改用 /image, 不碰 stereo_camera
echo "🔴 启动 RGB 推流 (camera_service → /image)..."
for attempt in 1 2 3; do
    if timeout 8 ros2 service call ${NS}/camera_service protocol/srv/CameraService \
        "{command: 9, args: \"\", width: 640, height: 480, fps: 30}" > /dev/null 2>&1; then
        echo "  ✅ camera_service 第${attempt}次调用成功"
        break
    fi
    echo "  ⚠ camera_service 第${attempt}次失败, 2s后重试"
    sleep 2
done

# ★ 验证 /image 实际帧数 (2026-08-13: stereo_camera 会假激活0帧, /image 是真实画面源)
check_rgb_frames() {
    timeout 8 python3 -c "
import rclpy,time
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n=rclpy.create_node('chk')
q=QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT)
c={'n':0}
n.create_subscription(Image, '$NS/image', lambda m: c.__setitem__('n',c['n']+1), q)
end=time.time()+3
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.05)
print(c['n'])
" 2>/dev/null
}

# 重启 camera_server 进程再推流 (2026-08-13: 仅调service不够, 采集线程卡死需重启进程)
restart_image() {
    echo "  ⚠ /image 无实际帧, 重启 camera_server 并重新推流..."
    pkill -f "camera_test/camera_server" 2>/dev/null || true
    sleep 4
    nohup ros2 run camera_test camera_server --ros-args -r __ns:=${NS} > /tmp/camera_server.log 2>&1 &
    sleep 6
    timeout 8 ros2 service call ${NS}/camera_service protocol/srv/CameraService \
        "{command: 9, args: \"\", width: 640, height: 480, fps: 30}" > /dev/null 2>&1
    sleep 4
}

RGB_FRAMES=$(check_rgb_frames)
if [ -z "$RGB_FRAMES" ] || [ "$RGB_FRAMES" = "0" ]; then
    restart_image
fi
RGB_FRAMES=$(check_rgb_frames)
if [ -n "$RGB_FRAMES" ] && [ "$RGB_FRAMES" != "0" ]; then
    echo "  ✅ RGB /image 画面正常 (${RGB_FRAMES}帧/3s)"
else
    echo "  ⚠ RGB /image 无帧, Web 可能黑屏"
fi
sleep 1

# ── 3. 相机 watchdog: 后台持续检测 /image 帧率, 无帧自动重启 camera_server (2026-08-13) ──
#   检测: 每8s用3s采样, 连续2次<2帧 → 杀 camera_server 进程重启 + 重新推流
(
    BAD=0
    while true; do
        F=$(check_rgb_frames)
        if [ -z "$F" ] || [ "$F" -lt 2 ]; then
            BAD=$((BAD+1))
            echo "[watchdog] ⚠ /image 帧率过低 (${F:-0}帧/3s), 第${BAD}次"
        else
            [ "$BAD" -gt 0 ] && echo "[watchdog] ✅ 帧率恢复 (${F}帧/3s)"
            BAD=0
        fi
        if [ "$BAD" -ge 2 ]; then
            echo "[watchdog] 🔄 连续低帧, 重新调 camera_service..."
            restart_image
            BAD=0
            sleep 15   # 重启后稳定期
        fi
        sleep 8
    done
) &

# ── 4. 启动比赛 ──
echo "🚀 启动比赛 (Stage1: 前进6m 巡线 + IMU 90°转弯)"
echo "   狗将自动站起开始! 请保持场地空旷"
echo "   Web 可视化: http://192.168.44.1:8080 (有线) 或 http://10.179.102.181:8080 (WiFi)"
echo "   (同一真实画面+巡线标注)"
echo ""
exec /SDCARD/race_ws/install/lib/cyberdog_race/race_controller --ros-args -r __ns:=${NS}
