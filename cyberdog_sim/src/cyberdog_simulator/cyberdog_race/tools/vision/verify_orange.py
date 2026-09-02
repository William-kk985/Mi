#!/usr/bin/env python3
# 橙球阈值验证: ①原图新旧阈值完整检测对比 ②合成黄线球防误检测试 ③输出对比图
# 用法(NX): python3 /tmp/verify_orange.py /tmp/shots/orange_00.png
import sys, math
import cv2
import numpy as np

OLD_LOW, OLD_HIGH = (6, 90, 80), (22, 255, 255)
NEW_LOW, NEW_HIGH = (3, 80, 80), (12, 255, 255)

def full_pipeline(img, low, high):
    """完全复刻 ball_detector.cpp find_ball 流程"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, low, high)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    m = cv2.erode(m, np.ones((3,3), np.uint8), iterations=1)
    m = cv2.dilate(m, np.ones((3,3), np.uint8), iterations=2)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    if cnts:
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 100: continue
            ratio = area / (img.shape[0] * img.shape[1])
            min_circ = 0.30 if ratio > 0.10 else 0.55
            per = cv2.arcLength(c, True)
            circ = 4*math.pi*area/(per*per) if per > 0 else 0
            if circ < min_circ: continue
            if best is None or area > cv2.contourArea(best):
                best = c
    if best is None:
        return None, None, m
    area = cv2.contourArea(best)
    per = cv2.arcLength(best, True)
    circ = 4*math.pi*area/(per*per)
    (cx, cy), r = cv2.minEnclosingCircle(best)
    return (cx, cy, r, area, circ), best, m

def main():
    p = sys.argv[1]
    img = cv2.imread(p)
    h, w = img.shape[:2]

    print('==== ① 原图检测对比 ====')
    for name, lo, hi in [('旧(6-22,90-255,80-255)', OLD_LOW, OLD_HIGH),
                          ('新(3-12,80-255,80-255)', NEW_LOW, NEW_HIGH)]:
        res, best, mask = full_pipeline(img, lo, hi)
        if res:
            cx, cy, r, area, circ = res
            dist = 0.10 * 402.0 / (math.sqrt(area/math.pi)) if area > 0 else 0
            print('  %s: ✅检出 中心=(%.0f,%.0f) 半径=%.0f 圆度=%.2f 估距=%.2fm' %
                  (name, cx, cy, r, circ, dist))
        else:
            print('  %s: ❌未检出(圆度过滤后无轮廓)' % name)

    print('==== ② 合成黄线球测试(球改成黄色H=25, 看谁误检) ====')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 把 NEW 阈值内的像素色相改成 25(黄线色), S/V 不动
    ball_mask = cv2.inRange(hsv, NEW_LOW, NEW_HIGH)
    print('  被改黄的像素数=%d' % (ball_mask.sum() // 255))
    hsv2 = hsv.copy()
    hsv2[:, :, 0][ball_mask > 0] = 25
    fake = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)
    for name, lo, hi in [('旧阈值', OLD_LOW, OLD_HIGH), ('新阈值', NEW_LOW, NEW_HIGH)]:
        res, best, mask = full_pipeline(fake, lo, hi)
        if res:
            print('  %s: ⚠️检出 中心=(%.0f,%.0f) r=%.0f 圆度=%.2f (mask像素=%d)' %
                  (name, res[0], res[1], res[2], res[4], mask.sum() // 255))
        else:
            print('  %s: ✅未检出 (mask像素=%d)' % (name, mask.sum() // 255))

    print('==== ②b 远处小橙点区域HSV(旧阈值唯一检出的目标) ====')
    roi = hsv[540:640, 670:770]
    if roi.size:
        print('  区域(670-770, 540-640) H:min=%d max=%d mean=%.0f S:min=%d max=%d mean=%.0f V:mean=%.0f' %
              (roi[:,:,0].min(), roi[:,:,0].max(), roi[:,:,0].mean(),
               roi[:,:,1].min(), roi[:,:,1].max(), roi[:,:,1].mean(), roi[:,:,2].mean()))

    print('==== ③ 生成对比图 /tmp/shots/verify.png ====')
    res_old, _, m_old = full_pipeline(img, OLD_LOW, OLD_HIGH)
    res_new, _, m_new = full_pipeline(img, NEW_LOW, NEW_HIGH)
    panel = np.zeros((h, w*3, 3), np.uint8)
    panel[:, :w] = img
    panel[:, w:2*w] = cv2.cvtColor(m_old, cv2.COLOR_GRAY2BGR)
    panel[:, 2*w:] = cv2.cvtColor(m_new, cv2.COLOR_GRAY2BGR)
    if res_old:
        cv2.circle(panel, (int(res_old[0]), int(res_old[1])), int(res_old[2]), (0, 0, 255), 4)
    if res_new:
        cv2.circle(panel, (w*2 + int(res_new[0]), int(res_new[1])), int(res_new[2]), (0, 255, 0), 4)
    cv2.putText(panel, 'src', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(panel, 'OLD mask', (w+10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(panel, 'NEW mask', (2*w+10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    small = cv2.resize(panel, (panel.shape[1]//3, panel.shape[0]//3))
    cv2.imwrite('/tmp/shots/verify.png', small)
    print('  已保存 /tmp/shots/verify.png')

if __name__ == '__main__':
    main()
