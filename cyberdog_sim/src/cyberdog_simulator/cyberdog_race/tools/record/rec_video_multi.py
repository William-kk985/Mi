#!/usr/bin/env python3
# 多段录制 center cam 视频 (订阅 image_center)
# 一次启动想录几段录几段, 每段自动编号, 不覆盖旧文件
#
# 用法(NX, 需要 ROS2 环境):
#   source /etc/mi/ros2_env.conf
#   python3 /tmp/rec_video_multi.py                       # 输出 /tmp/recs/rec_001.mp4 ...
#   python3 /tmp/rec_video_multi.py -d /SDCARD/recs -s 1.0 -f 30   # 全分辨率30fps
#
# 控制方式 (前台交互):
#   Enter   → 开始录制新的一段 / 结束当前段(保存)
#   q+Enter → 退出程序
#   Ctrl+C  → 保存当前段并退出
#   另一终端: touch /tmp/stop_record → 保存当前段并退出
#
# 拉回本地:
#   scp cyberdog-wifi:/SDCARD/recs/rec_001.mp4 ~/Mi/
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


def next_path(out_dir):
    """rec_001.mp4, rec_002.mp4 ... 自动递增"""
    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)
    i = 1
    while True:
        p = os.path.join(out_dir, 'rec_%03d.mp4' % i)
        if not os.path.exists(p):
            return p
        i += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-d', '--dir', default='/tmp/recs',
                    help='输出目录 (默认 /tmp/recs, 自动编号)')
    ap.add_argument('-s', '--scale', type=float, default=0.5,
                    help='缩放系数 (默认0.5=半分辨率, 省CPU)')
    ap.add_argument('-f', '--fps', type=float, default=20.0,
                    help='录制帧率 (默认20)')
    args = ap.parse_args()

    if os.path.exists(STOP_FILE):
        os.remove(STOP_FILE)

    rclpy.init()
    n = rclpy.create_node('rec_multi')
    b = CvBridge()
    st = {'writer': None, 'size': None, 'count': 0, 'start': 0.0,
          'path': None, 'seg': 0, 'last_frame': time.time()}

    def close_writer():
        if st['writer'] is not None:
            st['writer'].release()
            dur = time.time() - st['start']
            print('[rec] 段%d 保存: %s (%d帧 %.1fs)'
                  % (st['seg'], st['path'], st['count'], dur), flush=True)
        st['writer'] = None
        st['count'] = 0

    def start_writer():
        st['path'] = next_path(args.dir)
        st['seg'] += 1
        st['count'] = 0
        st['start'] = time.time()
        print('[rec] 段%d 开始 → %s' % (st['seg'], st['path']), flush=True)

    def cb(m):
        st['last_frame'] = time.time()
        if st['path'] is None:
            return   # 未按 Enter 开始, 丢弃帧 (否则空文件名创建VideoWriter会崩)
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
            st['writer'] = cv2.VideoWriter(st['path'], fourcc, args.fps, (w, h))
            st['size'] = (w, h)
        if st['size'] == (w, h):
            st['writer'].write(img)
            st['count'] += 1
            if st['count'] % 200 == 0:
                print('[rec] 段%d 已录 %d 帧 (%.0fs)'
                      % (st['seg'], st['count'], time.time() - st['start']),
                      flush=True)

    n.create_subscription(Image, TOPIC, cb,
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    print('[rec] 订阅 %s' % TOPIC, flush=True)
    print('[rec] Enter=开始/停止一段  q=退出  Ctrl+C=退出  touch %s=退出'
          % STOP_FILE, flush=True)

    try:
        import select
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
            if os.path.exists(STOP_FILE):
                print('[rec] 检测到停止文件, 结束', flush=True)
                break
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line == '':          # EOF(后台运行) 忽略
                    continue
                line = line.strip().lower()
                if line in ('q', 'quit', 'exit'):
                    print('[rec] 退出', flush=True)
                    break
                # Enter / 其他输入 → 切换录制状态
                if st['writer'] is not None:
                    close_writer()
                else:
                    if time.time() - st['last_frame'] > 3.0:
                        print('[rec] ⚠ 3s 无相机帧, 检查 image_center', flush=True)
                    start_writer()
    except KeyboardInterrupt:
        print('[rec] Ctrl+C, 结束', flush=True)

    close_writer()
    if os.path.exists(STOP_FILE):
        os.remove(STOP_FILE)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
