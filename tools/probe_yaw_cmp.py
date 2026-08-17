#!/usr/bin/env python3
# 对比 odom_out yaw vs global_to_robot abs_yaw, 判断是否同源
import time
import math
import struct

import lcm
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def unwrap(ys):
    yu = [ys[0]]
    for i in range(1, len(ys)):
        d = ys[i] - ys[i - 1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        yu.append(yu[-1] + d)
    return yu


class Probe(Node):
    def __init__(self):
        super().__init__('probe_yaw_cmp')
        self.oy = []
        self.sub = self.create_subscription(
            Odometry, '/mi_desktop_48_b0_2d_7b_02_c7/odom_out', self.cb, 10)

    def cb(self, msg):
        self.oy.append(quat_to_yaw(msg.pose.pose.orientation))


def main():
    rclpy.init()
    node = Probe()
    # global_to_robot: localization_lcmt = xyz[3] vxyz[3] rpy[3] omegaBody[3] vBody[3]
    lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=255')
    gy = []
    def on_g(c, d):
        n = struct.unpack('>9f', d[0:36])   # xyz[3] vxyz[3] rpy[3]
        gy.append(n[8])  # rpy[2]
    lc.subscribe('global_to_robot', on_g)

    print('[probe] 抓 10 秒对比 ...')
    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        lc.handle_timeout(100)
    print('odom_out 收包=%d, global 收包=%d' % (len(node.oy), len(gy)))
    if len(node.oy) < 10 or len(gy) < 10:
        print('数据不足')
        return
    import statistics
    oy = unwrap(node.oy)
    gy2 = unwrap(gy)
    # 按时间对齐不现实, 直接用中位数差值
    do = oy[-1] - oy[0]
    dg = gy2[-1] - gy2[0]
    print('odom_out yaw 漂移: %+.4f deg' % math.degrees(do))
    print('global    yaw 漂移: %+.4f deg' % math.degrees(dg))
    print('odom yaw std: %.4f deg' % math.degrees(statistics.stdev(oy)))
    print('glob yaw std: %.4f deg' % math.degrees(statistics.stdev(gy2)))
    print('odom yaw 当前值: %.3f rad (%.1f deg)' % (node.oy[-1], math.degrees(node.oy[-1])))
    print('glob yaw 当前值: %.3f rad (%.1f deg)' % (gy[-1], math.degrees(gy[-1])))


if __name__ == '__main__':
    main()
