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

# 2. 确保 NX 转发服务 (未运行则自动尝试启动, 需输 NX 密码 + sudo 密码)
echo "① 确保 NX scan-forward 转发服务..."
if timeout 5 ssh -o ConnectTimeout=3 -o BatchMode=yes cyberdog \
    'systemctl is-active scan-forward' 2>/dev/null | grep -q active; then
    echo "   ✅ NX scan-forward 运行中"
else
    echo "   ⚠ NX scan-forward 未运行, 尝试启动 (输入 NX 密码 + sudo 密码)..."
    ssh -t cyberdog 'sudo systemctl start scan-forward && echo "   ✅ NX scan-forward 已启动"' \
        || echo "   ⚠ 启动失败, 请手动: ssh -t cyberdog '\''sudo systemctl start scan-forward'\''"
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

# 4. 发布静态 TF (laser_odom -> laser_frame)
# ⚠ NX 的 /tf 建图未产出时无数据 → VM 自发布静态 TF, 雷达才能显示
echo "③ 发布静态 TF (laser_odom -> laser_frame)..."
pkill -f static_transform_publisher 2>/dev/null || true
sleep 1
nohup ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 laser_odom laser_frame > /tmp/tf.log 2>&1 &
sleep 2
echo "   ✅ TF 已发布"

# 5. 启动 rviz2 (预配置)
# ⚠ VM 无 GPU, 默认 OpenGL 会段错误崩溃 → 必须 LIBGL_ALWAYS_SOFTWARE=1 软件渲染
echo "④ 启动 rviz2 (软件渲染)..."
pkill -f "rviz2" 2>/dev/null || true
sleep 1
LIBGL_ALWAYS_SOFTWARE=1 nohup rviz2 -d /home/kaka/Mi/tools/rviz_cyberdog.rviz > /tmp/rviz2.log 2>&1 &
sleep 3
echo "   ✅ rviz2 已启动"

echo ""
echo "============================================"
echo " ✅ 完成! rviz2 窗口应显示:"
echo "   /scan        雷达     (彩色点, Fixed Frame=laser_odom)"
echo "   /map         建图     (黑白栅格, 需在 Displays 勾选)"
echo "   /point_cloud 3D点云  (彩色点, 默认显示)"
echo ""
echo " 建图查看步骤:"
echo "   1. NX 上确保建图激活+start_mapping"
echo "   2. 让狗在房间走动/转圈"
echo "   3. rviz2 里地图/点云实时增长"
echo " 日志: /tmp/scan_receive.log /tmp/rviz2.log"
echo "============================================"
