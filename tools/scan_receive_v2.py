#!/usr/bin/env python3
"""
VM 端：多 topic 接收器 v2
TCP 接收 NX 转发的数据，重新 publish 成本地 topic，供 rviz2 显示。
  8001 -> /scan   (LaserScan)
  8002 -> /map    (OccupancyGrid)
用法(VM 上): source /opt/ros/galactic/setup.bash && python3 scan_receive_v2.py
rviz2: Add -> By topic -> /scan(LaserScan) / /map(Map)
"""
import rclpy
import socket
import pickle
import struct
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

# /map 是瞬态地图(OccupancyGrid), 必须用 Transient Local 才能被 rviz2 Map 显示
MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

PORTS = {
    8001: ("/scan", LaserScan),
    8002: ("/map", OccupancyGrid, MAP_QOS),
    8003: ("/point_cloud", PointCloud2),
}


class MultiReceiver(Node):
    def __init__(self):
        super().__init__('scan_receiver')
        self.pubs = {}
        self.socks = {}
        self.conns = {}
        self.bufs = {}
        for port, item in PORTS.items():
            topic, msgtype = item[0], item[1]
            qos = item[2] if len(item) > 2 else 10
            self.pubs[port] = self.create_publisher(msgtype, topic, qos)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(('0.0.0.0', port))
            s.listen(1)
            s.setblocking(False)
            self.socks[port] = s
            self.conns[port] = None
            self.bufs[port] = b""
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f"监听 {list(PORTS.keys())}，等待 NX 连接...")

    def tick(self):
        for port in PORTS:
            if self.conns[port] is None:
                try:
                    self.conns[port], addr = self.socks[port].accept()
                    self.get_logger().info(f"✅ 端口{port} 已连接 {addr}")
                except BlockingIOError:
                    continue
            try:
                data = self.conns[port].recv(65536)
                if not data:
                    self.conns[port].close()
                    self.conns[port] = None
                    self.bufs[port] = b""
                    continue
                self.bufs[port] += data
                while len(self.bufs[port]) >= 4:
                    size = struct.unpack(">I", self.bufs[port][:4])[0]
                    if size > 50 * 1024 * 1024:
                        self.bufs[port] = b""
                        break
                    if len(self.bufs[port]) < 4 + size:
                        break
                    payload = self.bufs[port][4:4 + size]
                    self.bufs[port] = self.bufs[port][4 + size:]
                    try:
                        msg = pickle.loads(payload)
                        msg.header.stamp = self.get_clock().now().to_msg()
                        self.pubs[port].publish(msg)
                    except Exception:
                        pass
            except BlockingIOError:
                pass
            except (BrokenPipeError, ConnectionResetError, OSError):
                self.conns[port].close()
                self.conns[port] = None
                self.bufs[port] = b""


def main():
    rclpy.init()
    node = MultiReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
