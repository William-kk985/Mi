#!/usr/bin/env python3
# 定量探测 odom_out 的 yaw 质量(静止10秒)
# 用法(NX): python3 /tmp/probe_odom.py
import time
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    # 四元数转 yaw
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Probe(Node):
    def __init__(self):
        super().__init__('probe_odom')
        self.ts = []
        self.yaws = []
        self.wzs = []
        self.n = 0
        self.sub = self.create_subscription(
            Odometry, '/mi_desktop_48_b0_2d_7b_02_c7/odom_out', self.cb, 10)

    def cb(self, msg):
        self.n += 1
        self.ts.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.yaws.append(quat_to_yaw(msg.pose.pose.orientation))
        self.wzs.append(msg.twist.twist.angular.z)


def main():
    rclpy.init()
    node = Probe()
    print('[probe] 抓 10 秒 odom_out ...')
    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    n = node.n
    print('收包 = %d, 频率 = %.1f Hz' % (n, n / 10.0))
    if n < 5:
        return
    import statistics
    yaws = node.yaws
    wzs = node.wzs
    # yaw 展开(处理 ±π 跳变)
    yu = [yaws[0]]
    for i in range(1, len(yaws)):
        d = yaws[i] - yaws[i - 1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        yu.append(yu[-1] + d)
    print('yaw 范围: %.4f deg  (max-min 漂移)' % (math.degrees(max(yu) - min(yu))))
    print('yaw std:  %.4f deg' % (math.degrees(statistics.stdev(yu))))
    print('angular.z 静止: mean=%.5f std=%.5f rad/s' %
          (statistics.mean(wzs), statistics.stdev(wzs)))
    print('首尾 yaw 差: %.4f deg' % (math.degrees(yu[-1] - yu[0])))


if __name__ == '__main__':
    main()
