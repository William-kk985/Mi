#!/usr/bin/env python3
# 在 NX 上测试伙伴新模型 cola2/soccer2 对两个官方赛场视频的检测效果
# 用法:
#   python3 /SDCARD/s4_test/test_models.py                    # 默认每10帧推一次
#   python3 /SDCARD/s4_test/test_models.py --every 5
#   python3 /SDCARD/s4_test/test_models.py --conf 0.25 --only soccer2
import argparse
import os
import sys
import time

import cv2
import numpy as np
import onnxruntime as ort

DIR = '/SDCARD/s4_test'
MODELS = {'cola': 'cola.onnx', 'soccer': 'soccer.onnx'}
VIDEOS = ['9d1d6b4d696a5e3992e2a886d435bacc.mp4',
          '6c1267ee0574b37807bc475707ec14a0.mp4']
# (2026-08-20 纠正: 伙伴文件名与训练内容相反)
#   cola.onnx  = 原 soccer2.onnx, 单类 'c' = 可乐
#   soccer.onnx = 原 cola2.onnx, 双类, 类1 ball = 足球 (类0未知→丢弃)
CLASS_NAMES = {
    'cola': {0: '可乐'},
    'soccer': {1: '足球'},
}
# 独立阈值 (2026-08-20 按分数分布切分: 可乐真命中0.9+/误检≤0.78; 足球真命中0.95+/误检≤0.67)
CONF_THR = {'cola': 0.80, 'soccer': 0.70}


def letterbox(img, size=640):
    h, w = img.shape[:2]
    r = float(size) / max(h, w)
    nh, nw = int(round(h * r)), int(round(w * r))
    img2 = cv2.resize(img, (nw, nh))
    canvas = np.full((size, size, 3), 114, np.uint8)
    pad_x, pad_y = (size - nw) // 2, (size - nh) // 2
    canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = img2
    return canvas, r, pad_x, pad_y


def nms(boxes, iou_thr=0.45):
    keep = []
    boxes = sorted(boxes, key=lambda b: -b[4])
    while boxes:
        keep.append(boxes[0])
        rest = []
        bx1, by1, bx2, by2 = boxes[0][:4]
        ba = (bx2 - bx1) * (by2 - by1)
        for b in boxes[1:]:
            ix1, iy1 = max(bx1, b[0]), max(by1, b[1])
            ix2, iy2 = min(bx2, b[2]), min(by2, b[3])
            inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
            union = ba + (b[2] - b[0]) * (b[3] - b[1]) - inter
            if union <= 0 or inter / union <= iou_thr:
                rest.append(b)
        boxes = rest
    return keep


def decode(out, conf_thr, names, r, pad_x, pad_y, fw, fh):
    """out: (1, 25200, N) YOLOv5 风格 [cx,cy,w,h,obj,cls...]; 只认 names 里的类别"""
    d = out[0]                     # (25200, N)
    obj = d[:, 4]
    allowed = sorted(names.keys())
    cls_scores = d[:, 5:][:, allowed]   # 只取允许的类, 其他类丢弃
    conf = obj[:, None] * cls_scores
    best = conf.max(axis=1)
    rows = np.where(best >= conf_thr)[0]
    dets = []
    for i in rows:
        cx, cy, w, h = d[i, 0] - pad_x, d[i, 1] - pad_y, d[i, 2], d[i, 3]
        cx, cy, w, h = cx / r, cy / r, w / r, h / r
        j = int(cls_scores[i].argmax())
        cls_id = allowed[j]
        score = float(conf[i, j])
        # d 已是 640 画布像素坐标; -pad 后 /r 即原图像素, 直接使用(2026-08-20 修复: 再乘fw/640会放大2倍全跑出画框)
        x1 = max(0.0, cx - w / 2)
        y1 = max(0.0, cy - h / 2)
        x2 = min(float(fw), cx + w / 2)
        y2 = min(float(fh), cy + h / 2)
        dets.append([x1, y1, x2, y2, score, cls_id])
    return nms(dets)


def run_video(vname, every, conf_thr, verbose):
    """一个视频同时跑两个模型, 结果合并"""
    vpath = os.path.join(DIR, vname)
    out_path = os.path.join(DIR, 'out_%s.mp4' % vname[:8])

    sessions = {}
    for mname, mfile in MODELS.items():
        mpath = os.path.join(DIR, mfile)
        sessions[mname] = (
            ort.InferenceSession(mpath, providers=['CPUExecutionProvider']),
            CLASS_NAMES[mname])

    cap = cv2.VideoCapture(vpath)
    vfps = cap.get(cv2.CAP_PROP_FPS) or 25
    fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = None

    hits = {}                    # (模型, 类别) -> 命中帧数
    frame_idx = 0
    n_infer = 0
    t_infer = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_idx % every == 0:
            img, r, px, py = letterbox(frame)
            blob = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
            if writer is None:
                writer = cv2.VideoWriter(out_path,
                                         cv2.VideoWriter_fourcc(*'mp4v'),
                                         vfps, (fw, fh))
            ms_list = []
            for mname, (sess, names) in sessions.items():
                iname = sess.get_inputs()[0].name
                t1 = time.time()
                out = sess.run(None, {iname: blob})[0]
                ms_list.append((time.time() - t1) * 1000.0)
                n_infer += 1
                dets = decode(out, CONF_THR.get(mname, conf_thr), names,
                              r, px, py, fw, fh)
                for x1, y1, x2, y2, sc, cid in dets:
                    name = names.get(cid, str(cid))
                    key = (mname, name)
                    hits[key] = hits.get(key, 0) + 1
                    # 可乐=绿框, 足球=橙框
                    color = (0, 255, 0) if mname == 'cola' else (0, 165, 255)
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                                  color, 2)
                    cv2.putText(frame, '%s:%s %.2f' % (mname, name, sc),
                                (int(x1), int(y1) - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                if verbose and dets:
                    print('[%s] %.1fs %s: %s' %
                          (vname[:8], frame_idx / vfps, mname,
                           ' '.join('%s=%.2f' % (names[d[5]], d[4]) for d in dets)),
                          flush=True)
            if verbose:
                print('[%s] %.1fs 两模型耗时 %.0f/%.0f ms' %
                      (vname[:8], frame_idx / vfps, ms_list[0], ms_list[1]),
                      flush=True)
        writer.write(frame)
        frame_idx += 1

    cap.release()
    if writer is not None:
        writer.release()
    print('== %s: %d帧 两模型各推理%d次 (共%d)' %
          (vname[:8], frame_idx, n_infer // 2, n_infer))
    for (mname, name), n in sorted(hits.items()):
        print('   [%s] %s 命中 %d 帧' % (mname, name, n))
    print('   标注视频 → %s' % out_path, flush=True)
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--every', type=int, default=10, help='每N帧推理一次(默认10)')
    ap.add_argument('--conf', type=float, default=0.25, help='置信度阈值')
    ap.add_argument('--only', default='', help='只跑指定视频(前8位)')
    ap.add_argument('--quiet', action='store_true', help='不打印每帧详情')
    args = ap.parse_args()

    print('NX python=%s ort=%s' % (sys.version.split()[0], ort.__version__))
    for vname in VIDEOS:
        if args.only and args.only not in vname:
            continue
        run_video(vname, args.every, args.conf, not args.quiet)
    print('全部完成')


if __name__ == '__main__':
    main()
