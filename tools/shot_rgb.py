#!/usr/bin/env python3
# 手动拍照工具: 订阅 /image_center, 按 Enter 存一张, q 退出
# 用法(NX): python3 /tmp/shot_rgb.py
# 照片存 /tmp/shots/shot_NN.png, 之后 scp 回 VM:
#   scp -r cyberdog-wifi:/tmp/shots ~/Mi/
import os
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
    n = rclpy.create_node('shot')
    b = CvBridge()
    latest = {'img': None}

    def cb(m):
        latest['img'] = b.imgmsg_to_cv2(m, 'bgr8')

    n.create_subscription(Image, TOPIC, cb,
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    print('[shot] 订阅 %s' % TOPIC)
    print('[shot] 按 Enter 拍一张, 输入 q 退出')
    i = 0
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.1)
            line = input()
            if line.strip().lower() == 'q':
                break
            if latest['img'] is None:
                print('  还没收到帧!')
                continue
            path = os.path.join(OUT, 'shot_%02d.png' % i)
            cv2.imwrite(path, latest['img'])
            print('  已保存 %s (%dx%d)' % (path, latest['img'].shape[1], latest['img'].shape[0]))
            i += 1
    except (EOFError, KeyboardInterrupt):
        pass
    print('[shot] 共保存 %d 张' % i)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
