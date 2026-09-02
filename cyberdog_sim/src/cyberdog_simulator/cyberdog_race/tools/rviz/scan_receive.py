#!/usr/bin/env python3
"""
VM 端：雷达接收器
TCP 接收 NX 转发的 /scan 数据，重新 publish 成本地 /scan，供 rviz2 显示。
用法(VM 上): source /opt/ros/galactic/setup.bash && python3 scan_receive.py
然后: rviz2 -> Add -> By topic -> /scan -> LaserScan
"""
import rclpy
import socket
import pickle
import struct
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

PORT = 8001


class ScanReceiver(Node):
    def __init__(self):
        super().__init__('scan_receiver')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', PORT))
        self.sock.listen(1)
        self.sock.setblocking(False)
        self.conn = None
        self.buf = b""
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f"监听 :{PORT}，等待 NX 连接...")

    def tick(self):
        if self.conn is None:
            try:
                self.conn, addr = self.sock.accept()
                self.get_logger().info(f"✅ NX 已连接 {addr}")
            except BlockingIOError:
                return
        try:
            data = self.conn.recv(65536)
            if not data:
                self.conn.close()
                self.conn = None
                self.buf = b""
                self.get_logger().info("NX 断开，等待重连...")
                return
            self.buf += data
            while len(self.buf) >= 4:
                size = struct.unpack(">I", self.buf[:4])[0]
                if size > 10 * 1024 * 1024:  # 防异常
                    self.buf = b""
                    break
                if len(self.buf) < 4 + size:
                    break
                payload = self.buf[4:4 + size]
                self.buf = self.buf[4 + size:]
                try:
                    msg = pickle.loads(payload)
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self.pub.publish(msg)
                except Exception:
                    pass
        except BlockingIOError:
            pass
        except (BrokenPipeError, ConnectionResetError, OSError):
            self.conn.close()
            self.conn = None
            self.buf = b""


def main():
    rclpy.init()
    node = ScanReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
