#!/bin/bash
# ============================================================
# stop_all.sh — CyberDog 全部关闭 (在 VM 上运行)
# 关闭:
#   VM: 接收端(scan_receive_v2) + TF(static_transform) + rviz2
#   NX: 转发服务(scan-forward) + 建图(map_builder+stop_mapping) + 相机
# 需要: 输 NX ssh 密码 + sudo 密码
# 用法(VM): bash ~/Mi/tools/stop_all.sh
# ============================================================

echo "============================================"
echo " CyberDog 全部关闭"
echo "============================================"

# ── 1. VM 端 ──
echo "① 关闭 VM 端 (接收端/TF/rviz2)..."
pkill -f scan_receive_v2 2>/dev/null || true
pkill -f static_transform 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
sleep 1
if ps aux | grep -E "scan_receive|static_transform|rviz2" | grep -v grep > /dev/null; then
    echo "   ⚠ 仍有残留进程"
else
    echo "   ✅ VM 端已全部关闭"
fi

# ── 2. NX 端 ──
echo "② 关闭 NX 端 (转发/建图/相机)..."
echo "   (输入 NX 密码 + sudo 密码)"
ssh -t cyberdog 'source /etc/mi/ros2_env.conf 2>/dev/null; NS="/mi_desktop_48_b0_2d_7b_02_c7"; \
    sudo systemctl stop scan-forward 2>/dev/null; \
    timeout 5 ros2 service call ${NS}/stop_mapping visualization/srv/Stop "{}" >/dev/null 2>&1; \
    timeout 5 ros2 lifecycle set ${NS}/map_builder deactivate >/dev/null 2>&1; \
    timeout 5 ros2 lifecycle set ${NS}/camera/camera deactivate >/dev/null 2>&1; \
    echo "   ✅ NX 转发/建图/相机已关闭"; \
    echo "③ 清理残留地图 (删文件 + 重启节点清 DDS 缓存)..."; \
    sudo rm -rf /home/mi/mapping/* 2>/dev/null; \
    sudo systemctl restart cyberdog_sudo.service cyberdog_bringup.service >/dev/null 2>&1; \
    echo "   ✅ 残留地图已清除 (下次可视化无旧地图)"'

echo ""
echo "============================================"
echo " ✅ 全部关闭完成!"
echo " 已清除残留地图 (下次可视化是干净状态)"
echo " 再次启动:"
echo "   1. VM: bash ~/Mi/tools/start_rviz.sh"
echo "   2. NX: bash /home/mi/start_mapping.sh"
echo "============================================"
