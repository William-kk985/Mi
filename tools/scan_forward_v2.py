#!/usr/bin/env python3
"""
NX 端：多 topic 转发器 v2
通过 localhost DDS 正常订阅，经 TCP 转发到 VM。
  /scan -> 192.168.44.50:8001
  /map  -> 192.168.44.50:8002
不修改任何 DDS 配置，安全。
用法(NX 上, systemd 服务 scan-forward 已托管):
  sudo systemctl restart scan-forward
"""
import rclpy
import socket
import pickle
import struct
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

NS = "/mi_desktop_48_b0_2d_7b_02_c7"
VM_IP = "192.168.44.50"

# topic -> (端口, 消息类型)
TOPICS = {
    "/scan": (8001, LaserScan),
    "/map": (8002, OccupancyGrid),
    "/point_cloud": (8003, PointCloud2),
}


class MultiForwarder(Node):
    def __init__(self):
        super().__init__('scan_forwarder')
        self.socks = {}
        for t, (port, _) in TOPICS.items():
            self.socks[port] = None
        for t, (port, msgtype) in TOPICS.items():
            self.create_subscription(
                msgtype, NS + t, self.make_cb(port), 10)
        self.get_logger().info(
            f"转发 {NS}{list(TOPICS.keys())} -> {VM_IP}")

    def make_cb(self, port):
        def cb(msg):
            try:
                if self.socks.get(port) is None:
                    self.connect(port)
                    return
                data = pickle.dumps(msg)
                self.socks[port].sendall(
                    struct.pack(">I", len(data)) + data)
            except (BrokenPipeError, ConnectionResetError, OSError):
                try:
                    self.socks[port].close()
                except Exception:
                    pass
                self.socks[port] = None
        return cb

    def connect(self, port):
        while self.socks.get(port) is None:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5)
                s.connect((VM_IP, port))
                self.socks[port] = s
                self.get_logger().info(f"✅ 已连接 {VM_IP}:{port}")
            except Exception:
                time.sleep(2)


def main():
    rclpy.init()
    node = MultiForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
