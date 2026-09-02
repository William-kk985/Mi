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

# ★ (2026-09-02 重构: Stage4 路线由环境变量选, 替代 build_bins 多二进制; 默认正式版)
#   用法: STAGE4_IMPL=test bash start_race.sh   (跑 test 路线)
#        STAGE4_IMPL=formal bash start_race.sh  (跑正式版, 默认)
STAGE4_IMPL="${STAGE4_IMPL:-formal}"

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

# ── 2. 启动 RGB 推流: gc02m1 彩色相机 (2026-08-14 定稿) ──
#   通过官方 camera_api 桥接节点 center_cam 发布 /image_center
#   cam_id=1: gc02m1(bottomright) 低饱和彩色, 即网络资料所述"独立1MP RGB鼻区相机"
#   实测: sync=true/false 均30fps; 1280x960/1600x1200/1280x720 全支持
CENTER_CAM_ID=1         # 1=gc02m1 2MP彩色 (ov13b10=0已弃用: 视角差)
CENTER_SYNC=false      # gc02m1 两种sync均出图, false最稳
CENTER_W=1280
CENTER_H=960
echo "🔴 启动 center 相机 (cam_id=${CENTER_CAM_ID} ${CENTER_W}x${CENTER_H})..."
pkill -f "cyberdog_race/center_cam" 2>/dev/null || true
pkill -f "camera_test/camera_server" 2>/dev/null || true   # 停 bottom 推流, 释放 VI
sleep 3
nohup /SDCARD/race_ws/install/lib/cyberdog_race/center_cam --ros-args \
    -r __ns:=${NS} -p cam_id:=${CENTER_CAM_ID} -p sync:=${CENTER_SYNC} \
    -p width:=${CENTER_W} -p height:=${CENTER_H} \
    > /tmp/center_cam.log 2>&1 &
sleep 5
CAM_PID=$(pgrep -f "cyberdog_race/center_cam" | head -1)
if [ -n "$CAM_PID" ]; then
    taskset -pc 2,3 "$CAM_PID" > /dev/null 2>&1 && echo "  ✅ center_cam(pid=$CAM_PID) 已绑核 2-3"
fi

# ★ 验证 /image 实际帧数 (2026-08-13: stereo_camera 会假激活0帧, /image 是真实画面源)
check_rgb_frames() {
    timeout 8 python3 -c "
import rclpy,time
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n=rclpy.create_node('chk')
q=QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT)
c={'n':0}
n.create_subscription(Image, '$NS/image_center', lambda m: c.__setitem__('n',c['n']+1), q)
end=time.time()+3
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.05)
print(c['n'])
" 2>/dev/null
}

# 重启 center_cam 进程 (2026-08-14: 出帧异常时重启桥接节点)
restart_image() {
    echo "  ⚠ /image_center 无实际帧, 重启 center_cam..."
    pkill -f "cyberdog_race/center_cam" 2>/dev/null || true
    sleep 4
    nohup /SDCARD/race_ws/install/lib/cyberdog_race/center_cam --ros-args \
        -r __ns:=${NS} -p cam_id:=${CENTER_CAM_ID} -p sync:=${CENTER_SYNC} \
        -p width:=${CENTER_W} -p height:=${CENTER_H} \
        > /tmp/center_cam.log 2>&1 &
    sleep 5
    # 绑核 2-3: 采集线程独占CPU, buffer回流及时防VI挂
    NEWCAM=$(pgrep -f "cyberdog_race/center_cam" | head -1)
    if [ -n "$NEWCAM" ]; then
        taskset -pc 2,3 "$NEWCAM" > /dev/null 2>&1 || true
    fi
    sleep 4
}

# ★ taskset 绑核 (2026-08-13 NVIDIA论坛确认): VI "no reply"根因是采集线程抢不到CPU,
#   buffer回流不及时→VI丢帧10-30分钟后挂; center_cam绑核2-3, 与APP隔离
CAM_PID=$(pgrep -f "cyberdog_race/center_cam" | head -1)
if [ -n "$CAM_PID" ]; then
    taskset -pc 2,3 "$CAM_PID" > /dev/null 2>&1 \
        && echo "  ✅ center_cam(pid=$CAM_PID) 已绑核 2-3" \
        || echo "  ⚠ center_cam 绑核失败"
fi

RGB_FRAMES=$(check_rgb_frames)
if [ -z "$RGB_FRAMES" ] || [ "$RGB_FRAMES" = "0" ]; then
    restart_image
fi
RGB_FRAMES=$(check_rgb_frames)
if [ -n "$RGB_FRAMES" ] && [ "$RGB_FRAMES" != "0" ]; then
    echo "  ✅ center /image_center 画面正常 (${RGB_FRAMES}帧/3s)"
else
    echo "  ⚠ center /image_center 无帧, Web 可能黑屏"
fi
sleep 1

# ── 3. 相机 watchdog: 后台持续检测 /image_center 帧率, 无帧自动重启 center_cam (2026-08-14) ──
#   检测: 每8s用3s采样, 连续2次<2帧 → 杀 camera_server 进程重启 + 重新推流
(
    BAD=0
    while true; do
        F=$(check_rgb_frames)
        if [ -z "$F" ] || [ "$F" -lt 2 ]; then
            BAD=$((BAD+1))
            echo "[watchdog] ⚠ /image_center 帧率过低 (${F:-0}帧/3s), 第${BAD}次"
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

# ── 3.5 本地python推理服务 (2026-08-17): 可乐/足球ONNX新opset,
#    C++侧OpenCV4.1.1加载不了 → 由NX本地python(4.12)推理, unix socket回填 ──
if pgrep -f "s4_detect_server.py" > /dev/null 2>&1; then
    echo "🧠 本地推理服务已在运行"
else
    nohup python3 /SDCARD/race_ws/src/cyberdog_race/scripts/s4_detect_server.py \
        > /tmp/s4_detect.log 2>&1 &
    sleep 6   # 等模型加载 (28MB×2)
    if pgrep -f "s4_detect_server.py" > /dev/null 2>&1; then
        echo "🧠 本地推理服务已启动 (日志 /tmp/s4_detect.log)"
        tail -3 /tmp/s4_detect.log
    else
        echo "⚠️ 本地推理服务启动失败, 见 /tmp/s4_detect.log"
    fi
fi

# ── 4. 启动比赛 ──
echo "🚀 启动比赛 (Stage1: 前进6m 巡线 + IMU 90°转弯)"
echo "   狗将自动站起开始! 请保持场地空旷"
echo "   Web 可视化: http://192.168.44.1:8080 (有线) 或 http://10.179.102.181:8080 (WiFi)"
echo "   (同一真实画面+巡线标注)"
echo ""
exec taskset -c 4,5 /SDCARD/race_ws/install/lib/cyberdog_race/race_controller --ros-args -r __ns:=${NS} -p stage4_impl:=${STAGE4_IMPL} 2>&1 | tee /tmp/race_run.log
