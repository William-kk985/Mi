#!/usr/bin/env python3
# 录制 center cam 视频 (订阅 image_center), 随时停止
#
# 用法(NX, 需要 ROS2 环境):
#   source /etc/mi/ros2_env.conf
#   python3 /tmp/rec_video.py                     # 默认 /tmp/rec.mp4, 半分辨率
#   python3 /tmp/rec_video.py -o /tmp/rec2.mp4 -s 1.0 -f 30   # 全分辨率30fps
#
# 停止方式 (三选一):
#   1. 前台按 Enter
#   2. Ctrl+C
#   3. 另一终端执行: touch /tmp/stop_record
#
# 拉回本地:
#   scp cyberdog-wifi:/tmp/rec.mp4 ~/Mi/nx_logs_2026-08-18/
import argparse
import os
import sys
import time

import rclpy
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

NS = '/mi_desktop_48_b0_2d_7b_02_c7'
TOPIC = NS + '/image_center'
STOP_FILE = '/tmp/stop_record'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--out', default='/tmp/rec.mp4',
                    help='输出文件 (默认 /tmp/rec.mp4)')
    ap.add_argument('-s', '--scale', type=float, default=0.5,
                    help='缩放系数 (默认0.5=半分辨率, 省CPU)')
    ap.add_argument('-f', '--fps', type=float, default=20.0,
                    help='录制帧率 (默认20)')
    args = ap.parse_args()

    if os.path.exists(STOP_FILE):
        os.remove(STOP_FILE)

    rclpy.init()
    n = rclpy.create_node('rec')
    b = CvBridge()
    st = {'writer': None, 'size': None, 'count': 0, 'start': time.time()}

    def cb(m):
        try:
            img = b.imgmsg_to_cv2(m, 'bgr8')
        except Exception:
            return
        if args.scale != 1.0:
            img = cv2.resize(img, None, fx=args.scale, fy=args.scale,
                             interpolation=cv2.INTER_AREA)
        h, w = img.shape[:2]
        if st['writer'] is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            st['writer'] = cv2.VideoWriter(args.out, fourcc, args.fps, (w, h))
            st['size'] = (w, h)
            st['start'] = time.time()
            print('[rec] 开始录制 %s (%dx%d @ %.0ffps)'
                  % (args.out, w, h, args.fps), flush=True)
        if st['size'] == (w, h):
            st['writer'].write(img)
            st['count'] += 1
            if st['count'] % 100 == 0:
                print('[rec] 已录 %d 帧 (%.1fs)'
                      % (st['count'], time.time() - st['start']), flush=True)

    n.create_subscription(Image, TOPIC, cb,
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    print('[rec] 订阅 %s' % TOPIC, flush=True)
    print('[rec] 停止: 按 Enter / Ctrl+C / touch %s' % STOP_FILE, flush=True)

    try:
        import select
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
            if os.path.exists(STOP_FILE):
                print('[rec] 检测到停止文件, 结束', flush=True)
                break
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line == '':   # EOF (非交互后台运行) 忽略, 不误停
                    continue
                print('[rec] 收到 Enter, 结束', flush=True)
                break
    except KeyboardInterrupt:
        print('[rec] Ctrl+C, 结束', flush=True)

    if st['writer'] is not None:
        st['writer'].release()
    dur = time.time() - st['start'] if st['count'] else 0.0
    print('[rec] 共 %d 帧 (%.1fs) → %s' % (st['count'], dur, args.out), flush=True)
    if os.path.exists(STOP_FILE):
        os.remove(STOP_FILE)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
