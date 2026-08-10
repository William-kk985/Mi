#!/bin/bash
# ============================================================
# start_all.sh — CyberDog 一键启动: 建图 + 转发 + 可视化
# 在 VM 上运行, 自动完成:
#   ① NX:  激活建图(start_mapping.sh) + 启动转发(scan-forward)
#   ② VM:  启动接收端 + TF + rviz2 (默认点云模式)
# 需要: 输 NX 的 ssh 密码 + sudo 密码 (可用 ssh-copy-id 免密)
# 用法(VM): bash ~/Mi/tools/start_all.sh
# ============================================================

echo "============================================"
echo " CyberDog 一键启动 (建图 + 可视化)"
echo "============================================"

# ── ① NX: 建图 + 转发服务 (一次 ssh -t) ──
echo "① 配置 NX (建图 + 转发服务)..."
echo "   (输入 NX 密码, 以及 sudo 密码)"
ssh -t cyberdog 'bash /home/mi/start_mapping.sh && \
    sudo systemctl start scan-forward && \
    echo "  ✅ NX 转发服务已启动"'
if [ $? -ne 0 ]; then
    echo "⚠ NX 配置未完成(密码错误或命令失败), 检查后重试"
    exit 1
fi

# ── ② VM: 接收端 + TF + rviz2 ──
echo "② 启动 VM 可视化 (接收端 + TF + rviz2)..."
bash /home/kaka/Mi/tools/start_rviz.sh

echo ""
echo "============================================"
echo " ✅ 全部启动完成!"
echo "  rviz2 显示: 彩色点云(默认), 雷达/地图可手动勾选"
echo "  让狗走动 → /map 和 /point_cloud 实时增长"
echo "============================================"
echo " 停止建图并保存:"
echo "   ssh -t cyberdog \"ros2 service call /mi_desktop_48_b0_2d_7b_02_c7/stop_mapping visualization/srv/Stop '{}'\""
echo "  地图保存: /home/mi/mapping/"
echo "============================================"
