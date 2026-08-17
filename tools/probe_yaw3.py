#!/usr/bin/env python3
# 三源航向对比: odom_out(yaw) vs global_to_robot(rpy[2]) vs external_imu(quat)
# 用法: 静止5s → 遥控直行2~3m → 停5s → Ctrl+C
# 输出 /tmp/yaw3.csv, 每个源打印"相对起点变化(deg)"
import time, math, struct, csv, sys
import lcm
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Rec(Node):
    def __init__(self):
        super().__init__('probe_yaw3')
        self.odom_yaw = None
        self.sub = self.create_subscription(
            Odometry, '/mi_desktop_48_b0_2d_7b_02_c7/odom_out', self.cb, 10)
    def cb(self, msg):
        self.odom_yaw = quat_to_yaw(msg.pose.pose.orientation)


def main():
    rclpy.init()
    node = Rec()
    lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=255')
    state = {'gy': None, 'iy': None}
    def on_g(c, d):
        n = struct.unpack('>9f', d[0:36])
        state['gy'] = n[8]   # rpy[2]
    def on_i(c, d):
        q = struct.unpack('>4f', d[8:24])   # quat [x,y,z,w] 字节偏移8
        x, y, z, w = q
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        state['iy'] = math.atan2(siny, cosy)
    lc.subscribe('global_to_robot', on_g)
    lc.subscribe('external_imu', on_i)

    # 等三个源就绪再锁起点
    while node.odom_yaw is None or state['gy'] is None or state['iy'] is None:
        rclpy.spin_once(node, timeout_sec=0.05)
        lc.handle_timeout(50)
    o0, g0, i0 = node.odom_yaw, state['gy'], state['iy']
    print('起点: odom=%.4f  global=%.4f  imu=%.4f (rad)' % (o0, g0, i0))
    print('接下来: 5s静止 → 直行2~3m → 停5s (Ctrl+C结束)')
    print('t(s)  odom(deg)  global(deg)  imu(deg)')
    rows = []
    t0 = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        lc.handle_timeout(20)
        t = time.time() - t0
        do = math.degrees((node.odom_yaw - o0 + math.pi) % (2 * math.pi) - math.pi)
        dg = math.degrees((state['gy'] - g0 + math.pi) % (2 * math.pi) - math.pi)
        di = math.degrees((state['iy'] - i0 + math.pi) % (2 * math.pi) - math.pi)
        rows.append([round(t, 2), round(do, 2), round(dg, 2), round(di, 2)])
        if len(rows) % 10 == 0:
            print('%.1f  %+.2f  %+.2f  %+.2f' % (t, do, dg, di))
        time.sleep(0.09)
        if t > 60:
            print('60s 自动结束')
            break

    with open('/tmp/yaw3.csv', 'w') as f:
        w = csv.writer(f)
        w.writerow(['t', 'odom_deg', 'global_deg', 'imu_deg'])
        w.writerows(rows)
    print('saved /tmp/yaw3.csv (%d rows)' % len(rows))


if __name__ == '__main__':
    main()
