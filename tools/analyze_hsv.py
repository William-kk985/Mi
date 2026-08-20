#!/usr/bin/env python3
# HSV 阈值分析工具 (2026-08-20 用户: 定限高杆/可乐/足球/蓝障碍物阈值)
# 用法:
#   python3 analyze_hsv.py -i 图.jpg -o 输出前缀
#   可选: --roi x,y,w,h 只分析区域; --h1/--h2/--s/--v 覆盖阈值
import argparse
import os

import cv2
import numpy as np

# 当前代码里的阈值 (stage4_detector.cpp 2026-08-18 版)
DEFAULTS = {
    'limbar':   {'h1': 0, 'h2': 15, 'h3': 160, 'h4': 180, 's': 35, 'v': 30},
    'coke':     {'v_black': 70},
    'football': {'s': 60, 'v': 160},
    'obstacle': {'s': 25, 'v': 70},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--input', required=True)
    ap.add_argument('-o', '--out', default='hsv_out')
    ap.add_argument('--roi', default='', help='x,y,w,h')
    ap.add_argument('--mode', default='limbar', choices=list(DEFAULTS))
    ap.add_argument('--h1', type=int, default=None)
    ap.add_argument('--h2', type=int, default=None)
    ap.add_argument('--h3', type=int, default=None)
    ap.add_argument('--h4', type=int, default=None)
    ap.add_argument('--s', type=int, default=None)
    ap.add_argument('--v', type=int, default=None)
    args = ap.parse_args()

    img = cv2.imread(args.input)
    if img is None:
        print('打不开:', args.input)
        return
    h, w = img.shape[:2]
    if args.roi:
        x, y, rw, rh = [int(v) for v in args.roi.split(',')]
        img = img[y:y + rh, x:x + rw]
        print('ROI: (%d,%d) %dx%d' % (x, y, rw, rh))

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    d = dict(DEFAULTS[args.mode])
    for k, v in [('h1', args.h1), ('h2', args.h2), ('h3', args.h3),
                 ('h4', args.h4), ('s', args.s), ('v', args.v)]:
        if v is not None:
            d[k] = v

    if args.mode == 'limbar':
        # 红 H: [h1,h2] ∪ [h3,h4]
        m1 = cv2.inRange(hsv, (d['h1'], d['s'], d['v']), (d['h2'], 255, 255))
        m2 = cv2.inRange(hsv, (d['h3'], d['s'], d['v']), (d['h4'], 255, 255))
        mask = cv2.bitwise_or(m1, m2)
    elif args.mode == 'coke':
        mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, d['v_black']))
    elif args.mode == 'football':
        mask = cv2.inRange(hsv, (0, 0, d['v']), (180, d['s'], 255))
    else:  # obstacle 淡蓝
        mask = cv2.inRange(hsv, (90, d['s'], d['v']), (130, 255, 255))

    ratio = 100.0 * cv2.countNonZero(mask) / (mask.size or 1)
    px = hsv[mask > 0]
    print('[%s] 阈值: %s' % (args.mode, d))
    print('[%s] 命中像素 %.1f%% (%d px)' % (args.mode, ratio, len(px)))
    if len(px):
        for name, ch in [('H', 0), ('S', 1), ('V', 2)]:
            a = px[:, ch].astype(float)
            print('  %s: min=%.0f p5=%.0f 中位=%.0f p95=%.0f max=%.0f'
                  % (name, a.min(), np.percentile(a, 5), np.median(a),
                     np.percentile(a, 95), a.max()))

    vis = img.copy()
    vis[mask > 0] = (vis[mask > 0] * 0.4 + np.array([0, 255, 255]) * 0.6).astype(np.uint8)
    outmask = os.path.join(os.path.dirname(args.out) or '.',
                           os.path.basename(args.out) + '_%s_mask.jpg' % args.mode)
    outvis = os.path.join(os.path.dirname(args.out) or '.',
                          os.path.basename(args.out) + '_%s_vis.jpg' % args.mode)
    cv2.imwrite(outmask, mask)
    cv2.imwrite(outvis, vis)
    print('保存: %s / %s' % (outmask, outvis))


if __name__ == '__main__':
    main()
