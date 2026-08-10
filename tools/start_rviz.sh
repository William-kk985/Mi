#!/bin/bash
# ============================================================
# start_rviz.sh — CyberDog rviz 一键启动 (VM 端)
# 显示: /scan 雷达 + /map 建图 + /point_cloud 3D点云
# 前置(只需一次, 已配置好):
#   NX 端 scan-forward 服务已随 systemd 自启:
#     ssh cyberdog 'sudo systemctl enable --now scan-forward'
#   建图激活(需要时在 NX 上):
#     ros2 lifecycle set ${NS}/map_builder activate
#     ros2 service call ${NS}/start_mapping std_srvs/srv/SetBool "{data: true}"
# ============================================================
set -e

echo "============================================"
echo " CyberDog rviz 一键启动 (雷达+地图+点云)"
echo "============================================"

# 1. 环境
source /opt/ros/galactic/setup.bash 2>/dev/null
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export DISPLAY="${DISPLAY:-:0}"

# 2. 检查 NX 转发服务 (BatchMode + timeout 避免卡密码/卡连接, 失败仅警告)
echo "① 检查 NX scan-forward 服务..."
if timeout 5 ssh -o ConnectTimeout=3 -o BatchMode=yes cyberdog \
    'systemctl is-active scan-forward' 2>/dev/null | grep -q active; then
    echo "   ✅ NX scan-forward 运行中"
else
    echo "   ⚠ 无法确认 NX scan-forward 状态(ssh 需密码或服务未起)"
    echo "     请确认 NX 上已运行: sudo systemctl start scan-forward"
fi

# 3. 启动 VM 接收端 (8001/8002/8003)
echo "② 启动 VM 接收端..."
pkill -f scan_receive_v2.py 2>/dev/null || true
sleep 1
nohup python3 /home/kaka/Mi/tools/scan_receive_v2.py > /tmp/scan_receive.log 2>&1 &
sleep 3
if grep -q "监听" /tmp/scan_receive.log 2>/dev/null; then
    echo "   ✅ 接收端已启动 (8001/8002/8003)"
else
    echo "   ⚠ 接收端可能未启动, 查看: cat /tmp/scan_receive.log"
fi

# 4. 发布静态 TF (laser_frame 挂到 map)
echo "③ 发布静态 TF (map -> laser_frame)..."
pkill -f static_transform_publisher 2>/dev/null || true
sleep 1
nohup ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 map laser_frame > /tmp/tf.log 2>&1 &
sleep 2
echo "   ✅ TF 已发布"

# 5. 启动 rviz2 (预配置)
echo "④ 启动 rviz2..."
pkill -f "rviz2" 2>/dev/null || true
sleep 1
nohup rviz2 -d /home/kaka/Mi/tools/rviz_cyberdog.rviz > /tmp/rviz2.log 2>&1 &
sleep 3
echo "   ✅ rviz2 已启动"

echo ""
echo "============================================"
echo " ✅ 完成! rviz2 窗口应显示:"
echo "   /scan        雷达     (绿色点, Fixed Frame=laser_odom)"
echo "   /map         建图     (黑白栅格)"
echo "   /point_cloud 3D点云  (白色点)"
echo ""
echo " 建图查看步骤:"
echo "   1. NX 上确保建图激活+start_mapping"
echo "   2. 让狗在房间走动/转圈"
echo "   3. rviz2 里地图/点云实时增长"
echo " 日志: /tmp/scan_receive.log /tmp/rviz2.log"
echo "============================================"
