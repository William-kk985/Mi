#!/usr/bin/env python3
# 视频拆帧工具 (2026-08-20 用户: 定HSV阈值用 — 限高杆/可乐/足球/蓝障碍物)
# 支持: 视频拆帧 / 未来摄像头画面抽帧
#
# 用法:
#   python3 video_to_frames.py -i rec.mp4 -o frames                 # 默认0.5s一帧
#   python3 video_to_frames.py -i rec.mp4 -o frames -e 6            # 每6帧取1
#   python3 video_to_frames.py -i rec.mp4 -o frames --fps 2         # 按2fps抽
#   python3 video_to_frames.py -i rec.mp4 -o frames -s 10 -t 30     # 只要10~30秒
import argparse
import os
import sys

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--input', required=True, help='输入视频')
    ap.add_argument('-o', '--outdir', default='frames', help='输出目录')
    ap.add_argument('-e', '--every', type=int, default=0, help='每N帧取1')
    ap.add_argument('--fps', type=float, default=0, help='按fps抽帧(优先)')
    ap.add_argument('--scale', type=float, default=1.0, help='缩放系数')
    ap.add_argument('-s', '--start', type=float, default=0, help='起始秒')
    ap.add_argument('-t', '--end', type=float, default=0, help='结束秒, 0=到结尾')
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.input)
    if not cap.isOpened():
        print('打不开:', args.input)
        sys.exit(1)
    vfps = cap.get(cv2.CAP_PROP_FPS) or 20.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    os.makedirs(args.outdir, exist_ok=True)

    step = args.every
    if args.fps > 0:
        step = max(1, int(round(vfps / args.fps)))
    if step <= 0:
        step = max(1, int(round(vfps * 0.5)))   # 默认0.5s一帧

    idx = saved = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = idx / vfps
        if t < args.start:
            idx += 1
            continue
        if args.end > 0 and t > args.end:
            break
        if idx % step == 0:
            if args.scale != 1.0:
                frame = cv2.resize(frame, None, fx=args.scale, fy=args.scale,
                                   interpolation=cv2.INTER_AREA)
            name = os.path.join(args.outdir, 'f_%06d_%06.2fs.jpg' % (idx, t))
            cv2.imwrite(name, frame)
            saved += 1
        idx += 1
    cap.release()
    dur = (total / vfps) if vfps else 0.0
    print('完成: %d 张 → %s (视频约%.1fs, 每%.2fs一帧)' % (saved, args.outdir, dur, step / vfps))


if __name__ == '__main__':
    main()
