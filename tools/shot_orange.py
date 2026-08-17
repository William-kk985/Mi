#!/usr/bin/env python3
# 非交互抓图: 自动存5张 RGB, 供橙色球阈值标定
# 用法(NX): python3 /tmp/shot_orange.py
# 存 /tmp/shots/orange_NN.png, scp 回 VM 分析
import os
import time
import rclpy
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

NS = '/mi_desktop_48_b0_2d_7b_02_c7'
TOPIC = NS + '/image_center'
OUT = '/tmp/shots'

def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    n = rclpy.create_node('shot_orange')
    b = CvBridge()
    latest = {'img': None, 't': 0}

    def cb(m):
        latest['img'] = b.imgmsg_to_cv2(m, 'bgr8')
        latest['t'] = time.time()

    n.create_subscription(Image, TOPIC, cb,
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    print('[shot] 订阅 %s, 等首帧...' % TOPIC)
    t0 = time.time()
    while latest['img'] is None and time.time() - t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.1)
    if latest['img'] is None:
        print('[shot] 8s 没收到帧, 退出')
        return
    print('[shot] 收到帧 %dx%d, 连续拍5张(间隔0.5s)' %
          (latest['img'].shape[1], latest['img'].shape[0]))
    for i in range(5):
        while time.time() - latest['t'] > 1.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        img = latest['img']
        path = os.path.join(OUT, 'orange_%02d.png' % i)
        cv2.imwrite(path, img)
        print('  已保存 %s' % path)
        for _ in range(5):
            rclpy.spin_once(n, timeout_sec=0.1)
        time.sleep(0.4)
    print('[shot] 完成')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
