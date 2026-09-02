#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════
# annotate_video.py — 回放视频 + 模型推理 + 画框标注输出视频 (2026-08-22)
# 走实机同款链路: 半分辨率JPEG → socket s4_detect_server → 双模型
# 每 --every 帧推理一次, 框保持到下一推理帧; 所有帧写入输出视频
# 用法 (NX):
#   python3 /SDCARD/race_bins/annotate_video.py /SDCARD/recs/rec_003.mp4 /SDCARD/recs/s4_ann_003.mp4
# 输出 mp4v; 拉回本地后可用 imageio-ffmpeg 转 H264
# ═══════════════════════════════════════════════════════════
import argparse
import os
import socket
import struct
import sys

import cv2

SOCK_PATH = "/tmp/s4_detect.sock"
SCALE = 0.5   # 发送分辨率 = 原图 × 0.5 (与 C++ remote_detect_async 一致)


def ask_server(jpg_bytes, timeout=25.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        s.connect(SOCK_PATH)
        s.sendall(struct.pack("<I", len(jpg_bytes)) + jpg_bytes)
        s.settimeout(timeout)
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        return buf.split(b"\n")[0].decode(errors="replace")
    finally:
        s.close()


def parse_line(line):
    """返回 ((found,cx,dist,x,y,w,h,conf), (found,...)) 可乐/足球"""
    res = {"coke": None, "fb": None}
    for part in line.split(";"):
        part = part.strip()
        if part.startswith("coke "):
            f = part.split()
            if len(f) >= 9:
                res["coke"] = (int(f[1]), float(f[2]), float(f[3]),
                               int(f[4]), int(f[5]), int(f[6]), int(f[7]), float(f[8]))
        elif part.startswith("fb "):
            f = part.split()
            if len(f) >= 9:
                res["fb"] = (int(f[1]), float(f[2]), float(f[3]),
                             int(f[4]), int(f[5]), int(f[6]), int(f[7]), float(f[8]))
    return res["coke"], res["fb"]


def draw(frame, coke, fb, inv_scale):
    """框坐标是半分辨率图坐标 → ×inv_scale 还原到原图"""
    H, W = frame.shape[:2]
    for det, name, color in ((coke, "COKE", (0, 255, 0)), (fb, "FB", (255, 0, 0))):
        if det is None or det[0] == 0:
            continue
        _, cx, dist, x, y, w, h, conf = det
        x1 = int(x * inv_scale); y1 = int(y * inv_scale)
        x2 = int((x + w) * inv_scale); y2 = int((y + h) * inv_scale)
        x1 = max(0, min(W - 1, x1)); y1 = max(0, min(H - 1, y1))
        x2 = max(1, min(W, x2)); y2 = max(1, min(H, y2))
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{name} {conf:.2f} d={dist:.2f}m"
        cv2.putText(frame, label, (x1, max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("out")
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--fps", type=float, default=0, help="输出fps, 0=用原视频")
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args()

    if not os.path.exists(SOCK_PATH):
        print(f"✗ 服务未运行: {SOCK_PATH}")
        return 1
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"✗ 打不开 {args.video}")
        return 1
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = args.fps if args.fps > 0 else (cap.get(cv2.CAP_PROP_FPS) or 20)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
    out = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
    if not out.isOpened():
        print(f"✗ 写不了 {args.out}")
        return 1
    print(f"[ann] {args.video} → {args.out} 共{total}帧 每{args.every}帧推理一次")

    inv = 1.0 / SCALE
    coke_last = fb_last = None
    idx = 0
    infer_n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if args.max_frames and idx >= args.max_frames:
            break
        if idx % args.every == 0:
            small = cv2.resize(frame, None, fx=SCALE, fy=SCALE)
            ok2, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok2:
                try:
                    line = ask_server(jpg.tobytes())
                    coke_last, fb_last = parse_line(line)
                    infer_n += 1
                except Exception as e:
                    print(f"  帧{idx} 推理失败: {e}")
        draw(frame, coke_last, fb_last, inv)
        out.write(frame)
        idx += 1
        if idx % 100 == 0:
            print(f"  已处理 {idx} 帧")
            sys.stdout.flush()
    cap.release()
    out.release()
    print(f"[ann] 完成: 输出 {args.out}, 共{idx}帧, 推理{infer_n}次")
    return 0


if __name__ == "__main__":
    sys.exit(main())
