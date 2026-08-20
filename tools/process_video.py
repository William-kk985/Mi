#!/usr/bin/env python3
# 四目标检测效果预览 (2026-08-20 用户: 定阈值后看视频效果)
# 限高杆/足球/可乐用新阈值, 蓝障碍物保持现状
# 用法: python3 process_video.py -i rec.mp4 -o marked.mp4 [--show] [-e 每N帧处理1帧]
import argparse
import math

import cv2
import numpy as np


def detect_limbar(hsv):
    m = cv2.inRange(hsv, (0, 25, 55), (8, 120, 160)) | cv2.inRange(hsv, (160, 25, 55), (180, 120, 160))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        if cv2.contourArea(c) < 500:
            continue
        if w > 1.5 * h or h > 1.5 * w:   # 横梁或竖腿
            yield x, y, w, h, 'limbar'


def detect_football(hsv):
    white = cv2.inRange(hsv, (0, 0, 140), (180, 30, 255))
    black = cv2.inRange(hsv, (0, 0, 0), (180, 255, 90))
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 300:
            continue
        (cx, cy), r = cv2.minEnclosingCircle(c)
        if r < 8 or r > 90:
            continue
        if area / (math.pi * r * r) < 0.65:
            continue
        mask = np.zeros(white.shape, np.uint8)
        cv2.circle(mask, (int(cx), int(cy)), int(r), 255, -1)
        mask &= black
        ratio = cv2.countNonZero(mask) / max(1.0, math.pi * r * r)
        if ratio > 0.04:
            yield int(cx - r), int(cy - r), int(2 * r), int(2 * r), 'football'


def detect_coke(hsv):
    m = cv2.inRange(hsv, (0, 0, 0), (180, 45, 85))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        if cv2.contourArea(c) < 300:
            continue
        if h > 1.2 * w:
            yield x, y, w, h, 'coke'


def detect_obstacle(hsv):
    m = cv2.inRange(hsv, (90, 25, 70), (130, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        if cv2.contourArea(c) > 300 and w > 20 and h > 20:
            yield x, y, w, h, 'obstacle'


def detect_orange(hsv):
    m = cv2.inRange(hsv, (3, 80, 80), (15, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 300:
            continue
        (cx, cy), r = cv2.minEnclosingCircle(c)
        if r < 8 or r > 90:
            continue
        if area / (math.pi * r * r) < 0.65:
            continue
        yield int(cx - r), int(cy - r), int(2 * r), int(2 * r), 'orange'


COLORS = {'limbar': (0, 0, 255), 'football': (255, 0, 255),
          'coke': (0, 255, 255), 'obstacle': (255, 0, 0), 'orange': (0, 165, 255)}
NAMES = {'limbar': '限高杆', 'football': '足球', 'coke': '可乐',
         'obstacle': '蓝障碍', 'orange': '橙球'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--input', required=True)
    ap.add_argument('-o', '--out', default='marked.mp4')
    ap.add_argument('-e', '--every', type=int, default=1)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.input)
    vfps = cap.get(cv2.CAP_PROP_FPS) or 20
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    vw = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'mp4v'), vfps, (w, h))
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.every == 0:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            found = []
            for det in (detect_limbar, detect_football, detect_coke, detect_obstacle, detect_orange):
                found.extend(det(hsv))
            for x, y, bw, bh, tag in found:
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), COLORS[tag], 2)
                cv2.putText(frame, NAMES[tag], (x, y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLORS[tag], 2)
            t = idx / vfps
            if found:
                print('%.1fs: %s' % (t, ' '.join('%s(%d,%d %dx%d)' % (NAMES[g], x, y, bw, bh)
                                                 for x, y, bw, bh, g in found)), flush=True)
        vw.write(frame)
        idx += 1
    cap.release()
    vw.release()
    print('完成 → %s (共%d帧)' % (args.out, idx))


if __name__ == '__main__':
    main()
