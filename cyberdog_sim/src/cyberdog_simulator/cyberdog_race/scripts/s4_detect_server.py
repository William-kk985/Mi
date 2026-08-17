#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════
# s4_detect_server.py — NX 本地 YOLO 推理服务 (2026-08-17)
# 背景: NX 系统 OpenCV 4.1.1 加载不了新 opset 的 ONNX 模型,
#       但 pip 的 opencv-python 4.12 可以 → C++ 通过 unix socket
#       把 JPEG 帧发给本服务, 本服务跑两个模型(可乐/足球)后回结果。
# 协议:
#   请求 = 4字节小端长度 + JPEG 数据
#   响应 = "coke <f> <cx> <dist> <x> <y> <w> <h> <conf>;fb <f> <cx> <dist> <x> <y> <w> <h> <conf>\n"
#          f=0/1, cx=画面x归一化[-1,1], dist=米, x/y/w/h=原始分辨率像素框, conf=置信度
# 推理后端: onnxruntime 1.10 (OpenCV DNN 加载OK但推理时 /model.24/Reshape 崩, 2026-08-17)
# 参数与 C++ detect_yolo 完全一致: conf=0.45 nms=0.45
#   real_size: coke=0.30m(瓶高) fb=0.22m(球径) FOCAL=402
# 输入尺寸 320 (NX arm64 CPU 下约 150~300ms/模型, 两模型并行线程)
# 用法: python3 s4_detect_server.py   (日志重定向到 /tmp/s4_detect.log)
# ═══════════════════════════════════════════════════════════
import os
import socket
import struct
import sys
import threading
import time

import numpy as np
import cv2
import onnxruntime as ort

BASE = "/SDCARD/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race/models"
SOCK_PATH = "/tmp/s4_detect.sock"
INPUT = 640
FOCAL = 402.0

# name: (model_path, conf, nms, real_size_m, target_class_id)  cls=-1=取最大类(多类模型类别索引未知时)
MODELS = {
    "coke": (os.path.join(BASE, "cola.onnx"),   0.45, 0.45, 0.30, 0),
    "fb":   (os.path.join(BASE, "soccer.onnx"), 0.45, 0.45, 0.22, -1),
}


def yolo(sess, img_bgr, conf, nms, cls, real):
    """onnxruntime 推理; 返回 (cx, dist, conf, x, y, w, h) 或 None"""
    H, W = img_bgr.shape[:2]
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    blob = cv2.resize(rgb, (INPUT, INPUT))
    blob = blob.transpose(2, 0, 1).astype(np.float32) / 255.0
    blob = blob[None, ...]
    out = sess.run(None, {sess.get_inputs()[0].name: blob})[0]

    # C++ 端重塑逻辑: 3维时 d1=shape[1], d2=shape[2]; d1<d2→[C,N]转置成[N,C]
    if out.ndim == 3:
        d1, d2 = out.shape[1], out.shape[2]
        if d1 < d2:
            det = out.reshape(d1, -1).T      # [C,N] → [N,C]
        else:
            det = out.reshape(d1, -1)        # [N,C]
    else:
        det = out

    has_obj = det.shape[1] >= 6
    off = 5 if has_obj else 4
    sx, sy = W / INPUT, H / INPUT

    boxes, confs = [], []
    for row in det:
        obj = float(row[4]) if has_obj else 1.0
        scores = row[off:] * obj
        max_id = int(np.argmax(scores))
        max_score = float(scores[max_id])
        if (cls >= 0 and max_id != cls) or max_score < conf:
            continue
        cx, cy, w, h = row[0] * sx, row[1] * sy, row[2] * sx, row[3] * sy
        boxes.append([int(cx - w / 2), int(cy - h / 2), int(w), int(h)])
        confs.append(max_score)

    if not boxes:
        return None
    idx = cv2.dnn.NMSBoxes(boxes, confs, conf, nms)
    if isinstance(idx, tuple):
        idx = idx[0]          # OpenCV≥4.5.4 返回 (indices, scores)
    if idx is None or len(idx) == 0:
        return None
    best = max(idx, key=lambda i: confs[int(i)])
    x, y, w, h = boxes[int(best)]
    cxc = ((x + w / 2.0) - W / 2.0) / (W / 2.0)
    dist = (real * FOCAL) / h if h > 1 else 0.0
    return (cxc, dist, confs[int(best)], x, y, w, h)


def main():
    print("[s4_detect] 加载模型(onnxruntime)...", flush=True)
    nets = {}
    for name, (path, conf, nms, real, cls) in MODELS.items():
        if not os.path.exists(path):
            print(f"[s4_detect] ❌ 模型不存在: {path}", flush=True)
            sys.exit(1)
        so = ort.SessionOptions()
        so.intra_op_num_threads = 2
        sess = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
        nets[name] = (sess, conf, nms, real, cls)
        print(f"[s4_detect] ✅ {name} 加载OK ({path})", flush=True)

    try:
        os.unlink(SOCK_PATH)
    except OSError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK_PATH)
    srv.listen(2)
    os.chmod(SOCK_PATH, 0o666)
    print(f"[s4_detect] 服务就绪 {SOCK_PATH} (输入{INPUT}x{INPUT})", flush=True)

    total, slow = 0, 0
    while True:
        conn, _ = srv.accept()
        try:
            while True:
                hdr = _recv_all(conn, 4)
                if hdr is None:
                    break
                n = struct.unpack("<I", hdr)[0]
                if n <= 0 or n > 8 * 1024 * 1024:
                    break
                jpg = _recv_all(conn, n)
                if jpg is None:
                    break

                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    conn.sendall(b"coke 0 0 0 0 0 0 0 0;fb 0 0 0 0 0 0 0 0\n")
                    continue

                # 串行推理 (2026-08-18: 实测两模型并行线程2.2-3.8s(CPU争抢),
                #   串行~1.6s更快; forward 释放 GIL 并行无益)
                t0 = time.time()
                res = {}
                for name in ("coke", "fb"):
                    sess, conf, nms, real, cls = nets[name]
                    res[name] = yolo(sess, img, conf, nms, cls, real)

                dt = (time.time() - t0) * 1000.0
                total += 1
                if dt > 600.0:
                    slow += 1
                if total % 10 == 0:
                    print(f"[s4_detect] last {dt:.0f}ms, slow(>600ms) {slow}/{total}", flush=True)

                c = res.get("coke")
                f = res.get("fb")
                def fmt(r):
                    return ("1 {0:.4f} {1:.3f} {2} {3} {4} {5} {6:.2f}".format(r[0], r[1], r[3], r[4], r[5], r[6], r[2])
                            if r else "0 0 0 0 0 0 0 0")
                line = "coke {0};fb {1}\n".format(fmt(c), fmt(f))
                conn.sendall(line.encode())
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass


def _recv_all(conn, n):
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = conn.recv(n - len(buf))
        except socket.timeout:
            return None
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


if __name__ == "__main__":
    main()
