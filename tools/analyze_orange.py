#!/usr/bin/env python3
# 分析橙色球照片的HSV分布, 校准ball_detector阈值
# 用法(NX): python3 /tmp/analyze_orange.py /tmp/shots/orange_00.png
import sys
import cv2
import numpy as np

def main():
    paths = sys.argv[1:] or ['/tmp/shots/orange_00.png']
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print('%s 读失败' % p)
            continue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        print('==== %s  %dx%d ====' % (p, img.shape[1], img.shape[0]))

        # 粗分割: 找橙色连通域(宽松)
        m = cv2.inRange(hsv, (0, 60, 50), (30, 255, 255))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        n, labels, stats, cents = cv2.connectedComponentsWithStats(m)
        if n <= 1:
            print('  无橙色区域')
            continue
        # 按面积排序, 输出前3个区域
        idxs = sorted(range(1, n), key=lambda i: -stats[i, cv2.CC_STAT_AREA])[:3]
        for rank, i in enumerate(idxs):
            x, y, w, h, area = stats[i, cv2.CC_STAT_AREA], stats[i, cv2.CC_STAT_TOP], \
                               stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT], stats[i, cv2.CC_STAT_AREA]
            mask_i = (labels == i).astype(np.uint8) * 255
            hs = hsv[:, :, 0][mask_i > 0]
            ss = hsv[:, :, 1][mask_i > 0]
            vs = hsv[:, :, 2][mask_i > 0]
            print('  [区域%d] bbox=(%d,%d,%d,%d) 面积=%d px 占图%.1f%%' %
                  (rank, x, y, w, h, area, 100.0 * area / (img.shape[0] * img.shape[1])))
            print('    H: min=%d max=%d mean=%.0f   p5=%.0f p95=%.0f' %
                  (hs.min(), hs.max(), hs.mean(), np.percentile(hs, 5), np.percentile(hs, 95)))
            print('    S: min=%d max=%d mean=%.0f   p5=%.0f p95=%.0f' %
                  (ss.min(), ss.max(), ss.mean(), np.percentile(ss, 5), np.percentile(ss, 95)))
            print('    V: min=%d max=%d mean=%.0f   p5=%.0f p95=%.0f' %
                  (vs.min(), vs.max(), vs.mean(), np.percentile(vs, 5), np.percentile(vs, 95)))
        # 当前阈值测试
        cur = cv2.inRange(hsv, (6, 90, 80), (22, 255, 255))
        print('  当前阈值(6-22,90-255,80-255) 命中像素=%d' % cur.sum())
        # 各阈值组合尝试: 显示几个候选的命中
        for lo in [(6, 90, 80), (0, 90, 70), (4, 100, 70), (8, 80, 60), (0, 70, 50)]:
            mm = cv2.inRange(hsv, lo, (25, 255, 255))
            print('    inRange%s~H25 = %d px' % (lo, mm.sum()))

if __name__ == '__main__':
    main()
