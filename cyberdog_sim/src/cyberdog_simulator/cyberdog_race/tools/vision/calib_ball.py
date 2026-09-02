#!/usr/bin/env python3
# 球距定标: 球放正好1m, 跑本脚本输出 raw 距离 → BALL_DIST_SCALE = 1.0/raw
# 检测逻辑完全复刻 ball_detector.cpp (半分辨率640域 + 当前阈值)
# 用法(NX): python3 /tmp/calib_ball.py
import math
import time
import rclpy
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

NS = '/mi_desktop_48_b0_2d_7b_02_c7'
TOPIC = NS + '/image_center'
FOCAL = 402.0
BALL_R = 0.10
ORANGE_LOW = (3, 80, 80)
ORANGE_HIGH = (12, 255, 255)

def detect(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, ORANGE_LOW, ORANGE_HIGH)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    m = cv2.erode(m, np.ones((3, 3), np.uint8), iterations=1)
    m = cv2.dilate(m, np.ones((3, 3), np.uint8), iterations=2)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 100:
            continue
        ratio = area / (img.shape[0] * img.shape[1])
        min_circ = 0.30 if ratio > 0.10 else 0.55
        per = cv2.arcLength(c, True)
        circ = 4 * math.pi * area / (per * per) if per > 0 else 0
        if circ < min_circ:
            continue
        if best is None or area > cv2.contourArea(best):
            best = c
    if best is None:
        return None
    area = cv2.contourArea(best)
    (cx, cy), r_enc = cv2.minEnclosingCircle(best)
    r_area = math.sqrt(area / math.pi)
    r = max(r_area, r_enc * 0.85)
    raw = BALL_R * FOCAL / r if r > 1 else 0
    return cx, cy, r, area, raw

def main():
    rclpy.init()
    n = rclpy.create_node('calib_ball')
    b = CvBridge()
    latest = {'img': None}

    def cb(m):
        latest['img'] = b.imgmsg_to_cv2(m, 'bgr8')

    n.create_subscription(Image, TOPIC, cb,
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    print('[calib] 等帧... 请把球放在相机正前方 1m 处')
    t0 = time.time()
    while latest['img'] is None and time.time() - t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.1)
    if latest['img'] is None:
        print('[calib] 8s 无帧')
        return
    print('[calib] 持续采样 5s, 每0.5s检测一次:')
    raws = []
    for i in range(10):
        img = latest['img']
        small = cv2.resize(img, None, fx=0.5, fy=0.5)
        res = detect(small)
        if res:
            cx, cy, r, area, raw = res
            raws.append(raw)
            print('  第%d次: 中心=(%.0f,%.0f) r=%.0fpx raw距离=%.3fm → scale=%.3f'
                  % (i, cx, cy, r, raw, 1.0 / raw if raw > 0 else 0))
        else:
            print('  第%d次: 未检出' % i)
        for _ in range(5):
            rclpy.spin_once(n, timeout_sec=0.1)
        time.sleep(0.4)
    if raws:
        med = sorted(raws)[len(raws) // 2]
        print('[calib] 中位 raw=%.3fm → 建议 BALL_DIST_SCALE = %.3f' % (med, 1.0 / med))
        print('        (球放 1m 时) 若球放的是 %dm 请换算' % 1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
