#!/usr/bin/env python3
# ============================================================
# record_dataset.py — 原始帧录制（无损，用于训练数据集）
# 与 start_web.sh 可同时运行（各自订阅同一 topic）
#
# 用法:
#   source /etc/mi/ros2_env.conf
#   python3 record_dataset.py                # 默认录 RGB → PNG
#   python3 record_dataset.py --format mp4   # 录 MP4 视频
#   python3 record_dataset.py --infra --depth # 同时录红外+深度
#
# 输出: /SDCARD/dataset/  (按时间戳子目录)
#   PNG模式:  frame_00000.png ... (无损, 训练推荐)
#   MP4模式:  rgb.mp4 / infra.mp4 / depth.mp4 (高码率)
# ============================================================
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

NS = "/mi_desktop_48_b0_2d_7b_02_c7"
# 存到脚本所在目录的 dataset 子目录下
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset")

TOPICS = {
    "rgb":   f"{NS}/image",
    "infra": f"{NS}/camera/infra1/image_rect_raw",
    "depth": f"{NS}/camera/depth/image_rect_raw",
}


class DatasetRecorder(Node):
    def __init__(self, fmt, enable_rgb, enable_infra, enable_depth, fps):
        super().__init__("dataset_recorder")
        self.bridge = CvBridge()
        self.fmt = fmt

        # 输出目录
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.out_dir = os.path.join(OUTPUT_DIR, stamp)
        os.makedirs(self.out_dir, exist_ok=True)

        self.writers = {}
        self.png_counts = {}
        self.start_time = time.time()
        self.frame_interval = 1.0 / max(fps, 1)
        self.last_save = {}

        # 订阅 + 初始化录制器
        subs = []
        if enable_rgb:    subs.append(("rgb", TOPICS["rgb"]))
        if enable_infra:  subs.append(("infra", TOPICS["infra"]))
        if enable_depth:  subs.append(("depth", TOPICS["depth"]))
        if not subs:
            self.get_logger().error("至少启用一个传感器: --rgb/--infra/--depth")
            raise SystemExit(1)

        for name, topic in subs:
            self.create_subscription(Image, topic,
                                     lambda m, n=name: self.on_frame(m, n),
                                     10)
            self.last_save[name] = 0.0
            if self.fmt == "mp4":
                self.writers[name] = None  # 首个回调时按尺寸创建

        self.get_logger().info(
            f"📹 录制开始 -> {self.out_dir} (格式={fmt})")
        self.get_logger().info(f"   按 Ctrl+C 停止")

    def on_frame(self, msg, name):
        now = time.time()
        if now - self.last_save.get(name, 0) < self.frame_interval:
            return  # 限帧率
        self.last_save[name] = now

        try:
            if name == "depth":
                # 深度 16UC1 → 伪彩色（存彩色图便于查看；原始值也可存16位）
                if msg.encoding.find("16UC") != -1 or msg.encoding == "mono16":
                    mat = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
                    mat = mat.astype(np.float32) / 5000.0 * 255.0
                    mat = np.clip(mat, 0, 255).astype(np.uint8)
                    frame = cv2.applyColorMap(mat, cv2.COLORMAP_JET)
                else:
                    frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            else:
                # RGB/红外统一 BGR
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"[{name}] 转换失败: {e}")
            return

        if frame is None or frame.size == 0:
            return

        if self.fmt == "png":
            idx = self.png_counts.get(name, 0)
            path = os.path.join(self.out_dir, f"{name}_{idx:05d}.png")
            cv2.imwrite(path, frame)  # PNG 无损
            self.png_counts[name] = idx + 1
        else:  # mp4
            if self.writers[name] is None:
                h, w = frame.shape[:2]
                path = os.path.join(self.out_dir, f"{name}.mp4")
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                self.writers[name] = cv2.VideoWriter(path, fourcc, 30.0, (w, h))
            self.writers[name].write(frame)

    def shutdown(self):
        for w in self.writers.values():
            if w is not None:
                w.release()
        if self.fmt == "png":
            for n, c in self.png_counts.items():
                self.get_logger().info(f"  {n}: {c} 帧 PNG")
        else:
            self.get_logger().info(f"  MP4 已保存到 {self.out_dir}")


def main():
    parser = argparse.ArgumentParser(description="CyberDog 原始帧录制")
    parser.add_argument("--format", choices=["png", "mp4"], default="png",
                        help="png=无损帧序列(训练推荐), mp4=视频")
    parser.add_argument("--rgb", action="store_true", default=True,
                        help="录 RGB (默认开)")
    parser.add_argument("--infra", action="store_true",
                        help="录 D430i 红外")
    parser.add_argument("--depth", action="store_true",
                        help="录 D430i 深度(伪彩色)")
    parser.add_argument("--fps", type=int, default=15,
                        help="录制帧率上限 (默认15)")
    args = parser.parse_args()

    rclpy.init()
    rec = DatasetRecorder(args.format, args.rgb, args.infra,
                          args.depth, args.fps)
    try:
        rclpy.spin(rec)
    except KeyboardInterrupt:
        pass
    finally:
        rec.shutdown()
        rec.destroy_node()
        rclpy.shutdown()
    print("✅ 录制结束")


if __name__ == "__main__":
    main()
