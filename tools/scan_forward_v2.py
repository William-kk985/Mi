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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

NS = "/mi_desktop_48_b0_2d_7b_02_c7"
VM_IP = "192.168.44.50"

# /tf 可能是 BEST_EFFORT 发布, 用宽松 QoS 订阅保证匹配
TF_SUB_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)

# /map 是 TransientLocal 地图发布, 必须用相同 durability 才收到
MAP_SUB_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

# /point_cloud 可能是 BEST_EFFORT 发布
PC_SUB_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)

# topic -> (端口, 消息类型, 订阅QoS)
TOPICS = {
    "/scan": (8001, LaserScan, 10),
    "/map": (8002, OccupancyGrid, MAP_SUB_QOS),
    "/point_cloud": (8003, PointCloud2, PC_SUB_QOS),
    "/tf": (8004, TFMessage, TF_SUB_QOS),
}


class MultiForwarder(Node):
    def __init__(self):
        super().__init__('scan_forwarder')
        self.socks = {}
        for t, (port, _, _) in TOPICS.items():
            self.socks[port] = None
        for t, (port, msgtype, qos) in TOPICS.items():
            self.create_subscription(
                msgtype, NS + t, self.make_cb(port), qos)
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
