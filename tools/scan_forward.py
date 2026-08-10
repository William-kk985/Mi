#!/usr/bin/env python3
"""
NX 端：雷达转发器
通过 localhost DDS 正常订阅 ${NS}/scan，经 TCP 转发到 VM (192.168.44.50:8001)
不修改任何 DDS 配置，安全。
用法(NX 上): source /etc/mi/ros2_env.conf && python3 /home/mi/scan_forward.py
"""
import rclpy
import socket
import pickle
import struct
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

NS = "/mi_desktop_48_b0_2d_7b_02_c7"
VM_IP = "192.168.44.50"
PORT = 8001


class ScanForwarder(Node):
    def __init__(self):
        super().__init__('scan_forwarder')
        self.sock = None
        self.count = 0
        self.sub = self.create_subscription(
            LaserScan, NS + "/scan", self.cb, 10)
        self.get_logger().info(f"订阅 {NS}/scan -> 转发 {VM_IP}:{PORT}")

    def connect(self):
        while self.sock is None:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5)
                s.connect((VM_IP, PORT))
                self.sock = s
                self.get_logger().info(f"✅ 已连接 VM {VM_IP}:{PORT}")
            except Exception:
                time.sleep(2)

    def cb(self, msg):
        try:
            if self.sock is None:
                self.connect()
                return
            data = pickle.dumps(msg)
            self.sock.sendall(struct.pack(">I", len(data)) + data)
            self.count += 1
        except (BrokenPipeError, ConnectionResetError, OSError):
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


def main():
    rclpy.init()
    node = ScanForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
