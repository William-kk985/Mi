#!/usr/bin/env python3
# 手机照片 → 模拟狗相机画质 (2026-08-20 用户: 手机拍的图先转成狗相机近似画质再定阈值)
# 处理: 缩放到宽1280 (狗相机1280x960比例) + 降饱和70% + 轻微提亮 (gc02m1低饱和特性)
# 用法:
#   python3 phone_to_dog.py -i 输入目录 -o 输出目录
#   python3 phone_to_dog.py -i 图.jpg -o 图_conv.jpg        # 单张
import argparse
import glob
import os

import cv2
import numpy as np


def convert(img):
    # 缩到宽1280
    if img.shape[1] > 1280:
        h = int(img.shape[0] * 1280 / img.shape[1])
        img = cv2.resize(img, (1280, h), interpolation=cv2.INTER_AREA)
    # 降饱和70% + 明显提亮, 模拟gc02m1低饱和/偏亮特性 (2026-08-20 用户: 之前太暗, V*1.15+20)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] *= 0.70
    hsv[:, :, 2] = np.clip(hsv[:, :, 2] * 1.15 + 20, 0, 255)
    img = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--input', required=True)
    ap.add_argument('-o', '--out', required=True)
    args = ap.parse_args()

    if os.path.isdir(args.input):
        files = sorted(glob.glob(os.path.join(args.input, '*.jpg')) +
                       glob.glob(os.path.join(args.input, '*.png')))
        os.makedirs(args.out, exist_ok=True)
        for f in files:
            img = cv2.imread(f)
            if img is None:
                print('跳过(读不了):', f)
                continue
            img = convert(img)
            out = os.path.join(args.out, os.path.basename(f))
            cv2.imwrite(out, img)
            print('已转换: %s (%dx%d)' % (out, img.shape[1], img.shape[0]))
    else:
        img = cv2.imread(args.input)
        if img is None:
            print('打不开:', args.input)
            return
        img = convert(img)
        cv2.imwrite(args.out, img)
        print('已转换: %s (%dx%d)' % (args.out, img.shape[1], img.shape[0]))


if __name__ == '__main__':
    main()
