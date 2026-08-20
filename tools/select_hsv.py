#!/usr/bin/env python3
# 框选 HSV 取色工具 (2026-08-20 用户: 自己圈目标大概方位, 输出 HSV 范围用于定阈值)
#
# 用法:
#   python3 /home/kaka/Mi/tools/select_hsv.py -i 图片.jpg
#
# 操作:
#   鼠标左键拖拽框选目标 → 按 Enter 输出框内 HSV 分布并保存 crop 图
#   按 r 清除重选; 直接关窗退出
import argparse
import os

import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')   # 固定后端, 弹窗稳定
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--input', required=True)
    ap.add_argument('-o', '--out', default='select_out')
    args = ap.parse_args()

    img = cv2.imread(args.input)
    if img is None:
        print('打不开:', args.input)
        return
    print('加载: %s  尺寸 %dx%d' % (args.input, img.shape[1], img.shape[0]), flush=True)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    H, W = img.shape[:2]
    state = {'p0': None, 'rect': None, 'patch': None}

    fig, ax = plt.subplots(figsize=(W / 100.0, H / 100.0), dpi=100)
    fig.canvas.manager.set_window_title('框选: %s' % os.path.basename(args.input))
    ax.imshow(img)
    ax.set_title('%s (%dx%d) — 按住左键拖拽框选 → Enter 输出 HSV | r=重选 | 关窗退出'
                 % (os.path.basename(args.input), W, H))

    def on_press(ev):
        if ev.inaxes is not ax or ev.button != 1:
            return
        state['p0'] = (ev.xdata, ev.ydata)

    def on_motion(ev):
        if state['p0'] is None or ev.inaxes is not ax:
            return
        x0, y0 = state['p0']
        x1, y1 = ev.xdata, ev.ydata
        if state['patch'] is None:
            state['patch'] = Rectangle((0, 0), 0, 0, fill=False, ec='lime', lw=2)
            ax.add_patch(state['patch'])
        state['patch'].set_xy((min(x0, x1), min(y0, y1)))
        state['patch'].set_width(abs(x1 - x0))
        state['patch'].set_height(abs(y1 - y0))
        fig.canvas.draw_idle()

    def on_release(ev):
        if state['p0'] is None:
            return
        x0, y0 = state['p0']
        x1, y1 = ev.xdata, ev.ydata
        state['p0'] = None
        x1c, y1c = min(x0, x1), min(y0, y1)
        x2c, y2c = max(x0, x1), max(y0, y1)
        x1i, y1i = max(0, int(x1c)), max(0, int(y1c))
        x2i, y2i = min(W, int(x2c)), min(H, int(y2c))
        if x2i - x1i < 3 or y2i - y1i < 3:
            return
        state['rect'] = (x1i, y1i, x2i, y2i)
        print('框选: (%d,%d)-(%d,%d) 尺寸 %dx%d'
              % (x1i, y1i, x2i, y2i, x2i - x1i, y2i - y1i), flush=True)

    def stat(x1, y1, x2, y2):
        px = hsv[y1:y2, x1:x2].reshape(-1, 3).astype(float)
        n = len(px)
        print('── 框内 %d 像素 HSV 分布 ──' % n, flush=True)
        for name, ch in [('H', 0), ('S', 1), ('V', 2)]:
            a = px[:, ch]
            print('  %s: min=%.0f  p5=%.0f  p25=%.0f  中位=%.0f  p75=%.0f  p95=%.0f  max=%.0f'
                  % (name, a.min(), np.percentile(a, 5), np.percentile(a, 25),
                     np.median(a), np.percentile(a, 75), np.percentile(a, 95), a.max()),
                  flush=True)
        crop = img[y1:y2, x1:x2]
        path = os.path.basename(args.out) + '_crop.jpg'
        cv2.imwrite(path, cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
        print('已存 %s (当前目录)' % path, flush=True)

    def onkey(ev):
        if ev.key == 'enter' and state['rect']:
            stat(*state['rect'])
        elif ev.key == 'r':
            state['rect'] = None
            if state['patch']:
                state['patch'].remove()
                state['patch'] = None
            fig.canvas.draw_idle()
            print('已清除, 重新框选', flush=True)

    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('button_release_event', on_release)
    fig.canvas.mpl_connect('key_press_event', onkey)
    plt.show()


if __name__ == '__main__':
    main()
