#!/usr/bin/env python3
# 把照片转成结构化描述: 网格平均色 + 圆形候选
# 用法(NX): python3 /tmp/describe_img.py /tmp/shots/orange_00.png
import sys
import math
import cv2
import numpy as np

def describe(p):
    img = cv2.imread(p)
    if img is None:
        print('%s 读失败' % p); return
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    print('==== %s  %dx%d ====' % (p, w, h))

    # 6x8 网格平均色 (BGR + HSV)
    print('-- 网格主色 (行x列, HSV H/S/V + 主色名) --')
    names = {'红': (0, 10), '橙': (10, 25), '黄': (25, 35), '绿': (35, 85),
             '青': (85, 100), '蓝': (100, 130), '紫': (130, 155), '红2': (155, 180),
             '灰': (-1, -1)}
    for r in range(6):
        row = []
        for c in range(8):
            cell = hsv[r*h//6:(r+1)*h//6, c*w//8:(c+1)*w//8]
            hm = int(np.median(cell[:, :, 0]))
            sm = int(np.median(cell[:, :, 1]))
            vm = int(np.median(cell[:, :, 2]))
            nm = '灰'
            for k, (a, b) in names.items():
                if k == '灰': continue
                if a <= hm < b and sm > 60 and vm > 60:
                    nm = k; break
            row.append('%s(%d,%d,%d)' % (nm, hm, sm, vm))
        print('  ' + ' | '.join(row))

    # 圆形候选: 橙色调 mask 找圆度高的轮廓
    m = cv2.inRange(hsv, (4, 80, 60), (28, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    m = cv2.erode(m, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print('-- 圆形候选 (面积>2000) --')
    found = 0
    for cc in cnts:
        area = cv2.contourArea(cc)
        if area < 2000: continue
        per = cv2.arcLength(cc, True)
        circ = 4 * math.pi * area / (per * per) if per > 0 else 0
        (cx, cy), r = cv2.minEnclosingCircle(cc)
        if circ < 0.25: continue
        found += 1
        print('  圆度=%.2f 中心=(%d,%d) 半径=%.0f 面积=%d 占图%.1f%%' %
              (circ, cx, cy, r, area, 100.0*area/(w*h)))
    if not found:
        print('  (无圆形候选)')

for p in sys.argv[1:]:
    describe(p)
