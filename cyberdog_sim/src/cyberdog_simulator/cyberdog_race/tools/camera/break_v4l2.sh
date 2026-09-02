#!/bin/bash
# 突破 RealSense V4L2 锁: 杀占用者 → opencv 直读 video4/5
echo "── 占用者命令(备份):"
cat /proc/10916/cmdline 2>/dev/null | tr '\0' ' '; echo
cat /proc/10866/cmdline 2>/dev/null | tr '\0' ' '; echo

kill 10916 2>/dev/null
sleep 2
kill 10866 2>/dev/null
sleep 3

echo "── 占用检查:"
fuser -v /dev/video4 /dev/video5 2>&1

for V in 4 5; do
  timeout 15 python3 - "$V" <<'PY'
import cv2, sys
v = int(sys.argv[1])
cap = cv2.VideoCapture(v)
if not cap.isOpened():
    print('video%d 打不开' % v)
else:
    ok, frame = cap.read()
    if ok:
        cv2.imwrite('/tmp/rs%d.png' % v, frame)
        print('video%d 存图 shape=%s' % (v, frame.shape))
    else:
        print('video%d read失败' % v)
    cap.release()
PY
done
