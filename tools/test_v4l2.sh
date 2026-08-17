#!/bin/bash
# 用 opencv 直接读 /dev/video4/5 (RealSense) 各存一帧
for V in 4 5; do
  timeout 15 python3 - "$V" <<'PY'
import cv2, sys
v = int(sys.argv[1])
cap = cv2.VideoCapture(v)
if not cap.isOpened():
    print('video%d 打不开' % v)
else:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ok, frame = cap.read()
    if ok:
        cv2.imwrite('/tmp/rs%d.png' % v, frame)
        print('video%d 存图 shape=%s' % (v, frame.shape))
    else:
        print('video%d read失败' % v)
    cap.release()
PY
done
