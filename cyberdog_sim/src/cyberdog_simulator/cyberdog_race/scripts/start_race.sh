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

# ── 2. 逐个激活相机: D430i(camera/camera + camera_align) + RGB(stereo_camera) ──
# ⚠ RGB 画面 /image 由 stereo_camera(双目RGB) + camera_server 出流; stereo_camera 未激活 → 黑屏 (2026-08-12)
echo "🔴 逐个激活相机 (D430i 红外/深度 + stereo_camera RGB)..."
# 等相机节点出现(最多 60s; 狗刚开机 bringup 拉起相机较慢 2026-08-11)
# ⚠ lifecycle get 直接查会因 DDS discovery 慢超时 → 先用 node list 判断节点存在
for node in camera/camera camera/camera_align stereo_camera; do
    echo "   等待 ${node} 节点出现..."
    NODE_OK=0
    for i in $(seq 1 30); do
        if timeout 5 ros2 node list 2>/dev/null | grep -q "${NS}/${node}"; then
            NODE_OK=1; break
        fi
        [ $((i % 5)) -eq 0 ] && echo "   ...已等 $((i * 2))s"
        sleep 2
    done
    [ "$NODE_OK" = "1" ] && echo "  ✅ ${node} 节点已出现" || echo "  ⚠ ${node} 60s 未出现, 跳过"
    STATE=$(timeout 8 ros2 lifecycle get ${NS}/${node} 2>/dev/null || true)
    case "$STATE" in
        *active*)            echo "  ✅ ${node} 已激活" ;;
        *unconfigured*|*inactive*)
            timeout 8 ros2 lifecycle set ${NS}/${node} configure > /dev/null 2>&1 || true
            sleep 1
            ACT=0
            for _ in $(seq 1 3); do   # activate 重试3次 (DDS/时序慢会失败)
                if timeout 8 ros2 lifecycle set ${NS}/${node} activate > /dev/null 2>&1; then
                    ACT=1; break
                fi
                sleep 2
            done
            [ "$ACT" = "1" ] && echo "  ✅ ${node} 激活成功" || echo "  ⚠ ${node} 激活失败/超时, 跳过" ;;
        *) echo "  ⚠ ${node} lifecycle 查询失败/超时, 跳过" ;;
    esac
    sleep 1

done
sleep 1

# ── RGB 推流激活: camera_service START_IMAGE_PUBLISH, 重试3次 (防黑屏, 2026-08-12) ──
echo "📷 激活 RGB 推流 (camera_service START_IMAGE_PUBLISH)..."
for attempt in 1 2 3; do
    if timeout 8 ros2 service call ${NS}/camera_service protocol/srv/CameraService \
        "{command: 9, args: \"\", width: 640, height: 480, fps: 30}" > /dev/null 2>&1; then
        echo "  ✅ camera_service 第${attempt}次调用成功"
        break
    fi
    echo "  ⚠ camera_service 第${attempt}次失败, 2s后重试"
    sleep 2
done

# 确认 /image_rgb 有 publisher (最多 30s; stereo_camera active 后才出流, 2026-08-12)
IMAGE_OK=0
for i in $(seq 1 15); do
    PUB=$(timeout 5 ros2 topic info ${NS}/image_rgb 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    if [ -n "$PUB" ] && [ "$PUB" != "0" ]; then IMAGE_OK=1; break; fi
    sleep 2
done
[ "$IMAGE_OK" = "1" ] && echo "  ✅ RGB /image_rgb 推流确认 (Publisher=$PUB)" \
                       || echo "  ⚠ RGB /image_rgb 未确认推流, 继续启动(Web 可能黑屏)"
sleep 1

# ── 3. 启动比赛 ──
echo "🚀 启动比赛 (Stage1: 前进6m 巡线 + IMU 90°转弯)"
echo "   狗将自动站起开始! 请保持场地空旷"
echo "   Web 可视化: http://192.168.44.1:8080 (有线) 或 http://10.179.102.181:8080 (WiFi)"
echo "   (同一真实画面+巡线标注)"
echo ""
exec /SDCARD/race_ws/install/lib/cyberdog_race/race_controller --ros-args -r __ns:=${NS}
