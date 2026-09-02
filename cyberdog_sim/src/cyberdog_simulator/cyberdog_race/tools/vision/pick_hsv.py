#!/usr/bin/env python3
# 鼠标框选区域 → 输出 HSV 分布 + 建议 inRange 阈值
# 用法: python3 pick_hsv.py <图片路径>
# 操作: 拖拽框选(可多次), n=清空重选, q=退出并输出汇总
import sys
import cv2
import numpy as np

class Picker:
    def __init__(self, img):
        self.img = img.copy()
        self.disp = img.copy()
        self.hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.rects = []
        self.dragging = False
        self.p0 = (0, 0)
        self.p1 = (0, 0)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.p0 = (x, y)
            self.p1 = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.p1 = (x, y)
            d = self.img.copy()
            for r in self.rects:
                cv2.rectangle(d, r[0], r[1], (0, 255, 0), 2)
            cv2.rectangle(d, self.p0, self.p1, (255, 0, 0), 1)
            cv2.imshow('pick_hsv (拖拽框选, n清空, q退出)', d)
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
            self.p1 = (x, y)
            x0, x1 = sorted([self.p0[0], self.p1[0]])
            y0, y1 = sorted([self.p0[1], self.p1[1]])
            if x1 - x0 > 5 and y1 - y0 > 5:
                self.rects.append(((x0, y0), (x1, y1)))
                print(f'  ✅ 已框选第{len(self.rects)}块: x[{x0},{x1}] y[{y0},{y1}]')
            d = self.img.copy()
            for r in self.rects:
                cv2.rectangle(d, r[0], r[1], (0, 255, 0), 2)
            cv2.imshow('pick_hsv (拖拽框选, n清空, q退出)', d)

    def run(self):
        cv2.imshow('pick_hsv (拖拽框选, n清空, q退出)', self.disp)
        cv2.setMouseCallback('pick_hsv (拖拽框选, n清空, q退出)', self.on_mouse)
        while True:
            k = cv2.waitKey(20) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('n'):
                self.rects = []
                cv2.imshow('pick_hsv (拖拽框选, n清空, q退出)', self.img.copy())
                print('  (已清空)')
        cv2.destroyAllWindows()

    def report(self):
        if not self.rects:
            print('未框选任何区域')
            return
        print('\n========== 汇总 ==========')
        all_px = []
        for i, ((x0, y0), (x1, y1)) in enumerate(self.rects):
            roi = self.hsv[y0:y1, x0:x1]
            px = roi.reshape(-1, 3)
            all_px.append(px)
            H = px[:, 0].astype(int); S = px[:, 1].astype(int); V = px[:, 2].astype(int)
            print(f'块{i+1} x[{x0},{x1}] y[{y0},{y1}]  像素{len(px)}:')
            print(f'  H  min={H.min():3d} 中位={np.median(H):3.0f}  max={H.max():3d}')
            print(f'  S  min={S.min():3d} 中位={np.median(S):3.0f}  max={S.max():3d}')
            print(f'  V  min={V.min():3d} 中位={np.median(V):3.0f}  max={V.max():3d}')
        all_px = np.vstack(all_px)
        H = all_px[:, 0].astype(int); S = all_px[:, 1].astype(int); V = all_px[:, 2].astype(int)
        lo = (max(int(H.min()) - 3, 0), max(int(S.min()) - 15, 0), max(int(V.min()) - 15, 0))
        hi = (min(int(H.max()) + 3, 180), 255, 255)
        print('\n建议 inRange 阈值 (含裕量):')
        print(f'  cv::inRange(hsv, cv::Scalar({lo[0]}, {lo[1]}, {lo[2]}), cv::Scalar({hi[0]}, 255, 255), mask);')
        print(f'  即 H[{lo[0]},{hi[0]}] S[{lo[1]},255] V[{lo[2]},255]')

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    if not path:
        print('用法: python3 pick_hsv.py <图片路径>')
        sys.exit(1)
    img = cv2.imread(path)
    if img is None:
        print(f'读失败: {path}')
        sys.exit(1)
    p = Picker(img)
    p.run()
    p.report()

if __name__ == '__main__':
    main()
