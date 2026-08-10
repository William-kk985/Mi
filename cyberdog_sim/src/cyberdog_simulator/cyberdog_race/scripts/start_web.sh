#!/bin/bash
# ============================================================
# start_web.sh — 全传感器 Web 推流
# 浏览器 http://192.168.44.1:8080
# ============================================================
set -e

ROS_ENV="/etc/mi/ros2_env.conf"
WS_SETUP="/SDCARD/race_ws/install/setup.bash"
NS="/mi_desktop_48_b0_2d_7b_02_c7"

echo "============================================"
echo " CyberDog Web 推流"
echo "============================================"

# ★ 防双开保护：彻底清理旧进程 + 等待/强制释放 8080（2026-08-07 完善）
#   'race_controlle[r]' 用括号技巧防止误杀本脚本自身
#   ⚠ 必须 || true：无旧进程时 pkill 返回 1，set -e 会误杀整个脚本（2026-08-07 修复）
pkill -f 'race_controlle[r]' 2>/dev/null || true

# 等待旧进程退出、8080 释放（最多 6 秒）。前次进程被杀/僵死时可能没立即释放端口
echo "  等待 8080 释放..."
for _ in $(seq 1 12); do
    ss -tln 2>/dev/null | grep -q ':8080 ' || break
    sleep 0.5
done

# 若 8080 仍被占用 → 强杀占用它的进程（pkill 可能因进程名/状态匹配不到）
if ss -tln 2>/dev/null | grep -q ':8080 '; then
    echo "  ⚠ 8080 仍被占用，强制释放..."
    fuser -k 8080/tcp 2>/dev/null || true
    sleep 1
fi
if ss -tln 2>/dev/null | grep -q ':8080 '; then
    echo "  ❌ 8080 无法释放，请手动检查: ss -tlnp | grep 8080"
    exit 1
fi
echo "  ✅ 8080 已空闲"

source "$ROS_ENV" 2>/dev/null || { echo "❌ 加载 $ROS_ENV 失败"; exit 1; }
# 必须直接 source（不要加管道，否则子 shell 环境变量传不回来）
# libg2o 警告只是 stderr 提示，不影响环境变量设置
source "$WS_SETUP" 2>/dev/null
export LD_LIBRARY_PATH="/SDCARD/race_ws/install/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="/opt/ros2/cyberdog:/opt/ros2/galactic"
export PYTHONPATH="/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages"

# ── D430i 红外+深度：lifecycle 激活（2026-08-10）──
# ⚠ D430i 是 lifecycle 节点，未 activate 时 infra1/infra2/depth 无 Publisher！
#   相机节点重启/狗重启后回到 unconfigured，必须重新激活
# ⚠ lifecycle get/set 是无限等待的 service 调用 → 全部加 timeout 5 防挂（2026-08-10）
#   camera/camera = realsense 驱动（Web 推流必需）；camera_align = 深度对齐（非必需，卡住就跳过）
echo "🔴 激活 D430i（红外+深度）..."
for node in camera/camera camera/camera_align; do
    STATE=$(timeout 5 ros2 lifecycle get ${NS}/${node} 2>/dev/null || true)
    case "$STATE" in
        *active*)            echo "  ✅ ${node} 已激活" ;;
        *unconfigured*|*inactive*)
            timeout 5 ros2 lifecycle set ${NS}/${node} configure > /dev/null 2>&1 || true
            timeout 5 ros2 lifecycle set ${NS}/${node} activate > /dev/null 2>&1 \
                && echo "  ✅ ${node} 激活成功" || echo "  ⚠ ${node} 激活失败/超时，跳过" ;;
        *) echo "  ⚠ ${node} lifecycle 查询失败/超时，跳过" ;;
    esac
done
sleep 1

echo "📷 启动 RGB..."
# ⚠ camera_service 不可用时（充电/未激活）ros2 service call 会无限等待 → 加 timeout 防阻塞
timeout 5 ros2 service call ${NS}/camera_service protocol/srv/CameraService \
    "{command: 9, args: \"\", width: 640, height: 480, fps: 30}" > /dev/null 2>&1 \
    || echo "⚠ camera_service 不可用，跳过 RGB 激活（其余流不受影响）"
sleep 1

for t in image scan odom_out camera/imu camera/infra1/image_rect_raw camera/depth/image_rect_raw bms_status touch_status; do
    PUB=$(ros2 topic info ${NS}/${t} 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    [ "$PUB" != "0" ] && echo "   ✅ $t" || echo "   ❌ $t"
done

echo ""
echo "🌐 http://192.168.44.1:8080"
echo ""

# 直接跑二进制（ros2 run 有 bug 会异常退出）
exec /SDCARD/race_ws/install/lib/cyberdog_race/race_controller --ros-args -r __ns:=${NS}
