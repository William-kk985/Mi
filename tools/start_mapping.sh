#!/bin/bash
# ============================================================
# start_mapping.sh — CyberDog 一键建图 (NX 端)
# 流程: 激活相机(IMU) → 激活 map_builder → start_mapping → 自动验证
# 前置: 狗已开机, cyberdog_bringup 正常运行
# 用法(NX 上): bash /home/mi/start_mapping.sh
# 建图: 脚本完成后遥控狗走动/转圈, /map 实时增长
# 停止并保存: ros2 service call ${NS}/stop_mapping visualization/srv/Stop "{}"
# ============================================================
NS="/mi_desktop_48_b0_2d_7b_02_c7"
set -e

source /etc/mi/ros2_env.conf 2>/dev/null

echo "============================================"
echo " CyberDog 一键建图"
echo "============================================"

# ── 1. 激活相机 camera/camera (D430i, IMU 数据源) ──
echo "① 激活相机 camera/camera (IMU)..."
STATE=$(timeout 5 ros2 lifecycle get ${NS}/camera/camera 2>/dev/null || true)
case "$STATE" in
    *active*)            echo "   ✅ 已激活" ;;
    *unconfigured*|*inactive*)
        timeout 5 ros2 lifecycle set ${NS}/camera/camera configure >/dev/null 2>&1 || true
        timeout 5 ros2 lifecycle set ${NS}/camera/camera activate >/dev/null 2>&1 \
            && echo "   ✅ 激活成功" || echo "   ⚠ 激活失败" ;;
    *) echo "   ⚠ 相机状态未知($STATE), 继续" ;;
esac

# ── 2. 等待相机初始化 ──
echo "② 等待相机初始化 (5s)..."
sleep 5

# ── 3. 激活 map_builder (建图节点) ──
echo "③ 激活 map_builder..."
STATE=$(timeout 5 ros2 lifecycle get ${NS}/map_builder 2>/dev/null || true)
case "$STATE" in
    *active*) echo "   ✅ 已激活" ;;
    *)
        timeout 5 ros2 lifecycle set ${NS}/map_builder configure >/dev/null 2>&1 || true
        timeout 5 ros2 lifecycle set ${NS}/map_builder activate >/dev/null 2>&1 \
            && echo "   ✅ 激活成功" || echo "   ⚠ 激活失败" ;;
esac

# ── 4. 开始建图 ──
echo "④ 调用 start_mapping..."
RESULT=$(timeout 6 ros2 service call ${NS}/start_mapping std_srvs/srv/SetBool "{data: true}" 2>&1 \
    | grep -oE "success=[A-Za-z]+" | head -1)
echo "   $RESULT"
case "$RESULT" in
    "success=True")  echo "   ✅ 建图已开始" ;;
    "success=False") echo "   ⚠ start_mapping 返回 False (可能已在建图, 看⑥ /map 确认)" ;;
    *)               echo "   ⚠ start_mapping 无响应" ;;
esac

# ── 5. 验证输入数据 (python, 避开 ros2 topic hz 的 QoS 坑) ──
echo "⑤ 验证输入数据 (odom/imu/scan)..."
python3 - <<'EOF'
import rclpy, time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
NS = "/mi_desktop_48_b0_2d_7b_02_c7"
rclpy.init(); n = rclpy.create_node('mapping_check')
res = {}
def mk(topic, typ, key):
    def cb(m): res[key] = True
    n.create_subscription(typ, NS + topic, cb, 10)
mk('/odom_out', Odometry, 'odom')
mk('/camera/imu', Imu, 'imu')
mk('/scan', LaserScan, 'scan')
t0 = time.time()
while time.time() - t0 < 8 and len(res) < 3:
    rclpy.spin_once(n, timeout_sec=1)
print(f"   odom: {'✅' if 'odom' in res else '❌'}   imu: {'✅' if 'imu' in res else '❌'}   scan: {'✅' if 'scan' in res else '❌'}")
n.destroy_node(); rclpy.shutdown()
EOF

# ── 6. 验证 /map 输出 (TransientLocal QoS) ──
echo "⑥ 验证 /map (等 5s 建图产出)..."
sleep 5
python3 - <<'EOF'
import rclpy, time
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
NS = "/mi_desktop_48_b0_2d_7b_02_c7"
rclpy.init(); n = rclpy.create_node('map_check')
Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
res = {}
def cb(m):
    occ = [x for x in m.data if x > 50]
    res['m'] = f"{m.info.width}x{m.info.height} (占用{len(occ)})"
n.create_subscription(OccupancyGrid, NS + '/map', cb, Q)
t0 = time.time()
while time.time() - t0 < 8 and 'm' not in res:
    rclpy.spin_once(n, timeout_sec=1)
print("   /map:", res.get('m', '❌ 无数据(0x0 或未发布)'))
n.destroy_node(); rclpy.shutdown()
EOF

echo ""
echo "============================================"
echo " ✅ 建图已启动!"
echo " 接下来: 遥控狗在房间走动/转圈, /map 实时增长"
echo " 停止并保存: ros2 service call ${NS}/stop_mapping visualization/srv/Stop \"{}\""
echo " 地图保存位置: /home/mi/mapping/"
echo "============================================"
