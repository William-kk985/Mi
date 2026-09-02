#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════
# replay_video.py — 用录制的视频回放验证实机推理链路 (2026-08-21)
# 与 C++ 实机路径完全一致:
#   视频帧 → resize 0.5 → JPEG编码 → unix socket 发 s4_detect_server
#   → 收到 "coke ...;fb ..." 结果行
# 用法 (NX 上跑, 服务需已在运行: pgrep -f s4_detect_server):
#   python3 /SDCARD/race_bins/replay_video.py /SDCARD/recs/rec_004.mp4
#   python3 /SDCARD/race_bins/replay_video.py /SDCARD/recs/rec_004.mp4 --every 3 --save-hits /SDCARD/recs/s4_hits
# 说明: 视频加速不影响逐帧检测结果; 服务串行~1s/帧, 建议 --every 2~5 抽帧
# ═══════════════════════════════════════════════════════════
import argparse
import os
import socket
import struct
import sys

import cv2

SOCK_PATH = "/tmp/s4_detect.sock"


def ask_server(jpg_bytes, timeout=25.0):
    """一次请求: 4字节LE长度 + JPEG; 返回响应文本行 (不含换行)"""
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video", help="视频路径")
    ap.add_argument("--every", type=int, default=2, help="每N帧送一次 (默认2)")
    ap.add_argument("--save-hits", default="", help="命中帧存图目录 (可选)")
    ap.add_argument("--max-frames", type=int, default=0, help="最多处理帧数, 0=全部")
    args = ap.parse_args()

    if not os.path.exists(SOCK_PATH):
        print(f"✗ 服务未运行: {SOCK_PATH} 不存在, 先启动 s4_detect_server")
        return 1
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"✗ 打不开视频: {args.video}")
        return 1
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    print(f"[replay] {args.video} 共{total_frames}帧 fps={fps:.1f} 每{args.every}帧送一次")
    if args.save_hits:
        os.makedirs(args.save_hits, exist_ok=True)
        print(f"[replay] 命中帧存到 {args.save_hits}/")

    sent = 0
    hits_coke = 0
    hits_fb = 0
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.every != 0:
            idx += 1
            continue
        if args.max_frames and sent >= args.max_frames:
            break
        idx += 1

        # ── 与 C++ remote_detect_async 一致: 半分辨率 JPEG 质量80 ──
        small = cv2.resize(frame, None, fx=0.5, fy=0.5)
        ok2, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok2:
            continue
        sent += 1
        try:
            line = ask_server(jpg.tobytes())
        except Exception as e:
            print(f"  帧{idx-1} 请求失败: {e}")
            continue

        coke_hit = "coke 1" in line
        fb_hit = "fb 1" in line
        if coke_hit:
            hits_coke += 1
        if fb_hit:
            hits_fb += 1
        tag = ""
        if coke_hit or fb_hit:
            tag = "  ★命中"
            if args.save_hits:
                cv2.imwrite(os.path.join(args.save_hits, f"hit_{idx-1:05d}.jpg"), frame)
        print(f"  帧{idx-1}: {line}{tag}")
        sys.stdout.flush()

    cap.release()
    print(f"[replay] 完成: 送{sent}帧, 可乐命中{hits_coke}, 足球命中{hits_fb}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
