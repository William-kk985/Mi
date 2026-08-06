#!/usr/bin/env python3
"""
传感器探测 — 验证 NX 上所有可用传感器 topic 是否真的有数据
==========================================================
逐个订阅 topic 收几帧，报告数据类型/尺寸/样本值。
覆盖：LiDAR / 超声 / TOF×4 / UWB / GPS / Touch / BMS / D430i右目 / 对齐深度 / 点云

用法（NX 上）:
    python3 sensor_probe.py
"""
import importlib
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

NS = '/mi_desktop_48_b0_2d_7b_02_c7'

# (topic后缀, 消息模块路径, 收帧数, 超时秒)
SENSORS = [
    ('scan',                              'sensor_msgs.msg.LaserScan',      3, 5),
    ('ultrasonic_payload',                'sensor_msgs.msg.Range',          3, 5),
    ('head_tof_payload',                  'protocol.msg.HeadTofPayload',    3, 5),
    ('rear_tof_payload',                  'protocol.msg.RearTofPayload',    3, 5),
    ('uwb_raw',                           'protocol.msg.UwbRaw',            3, 5),
    ('gps_payload',                       'protocol.msg.GpsPayload',        3, 5),
    ('touch_status',                      'protocol.msg.TouchStatus',       2, 5),
    ('bms_status',                        'protocol.msg.BmsStatus',         2, 5),
    ('camera/infra2/image_rect_raw',      'sensor_msgs.msg.Image',          2, 5),
    ('camera/aligned_depth_to_rgb/image_raw', 'sensor_msgs.msg.Image',      2, 5),
    ('camera/depth/color/points',         'sensor_msgs.msg.PointCloud2',    2, 5),
]


def summarize(node, topic, msgs):
    """对收到的消息做摘要"""
    if not msgs:
        return "❌ 无数据"
    m = msgs[0]
    info = []
    # 通用字段
    for f in ('header',):
        if hasattr(m, f) and hasattr(m.header, 'frame_id'):
            info.append(f"frame={m.header.frame_id or '-'}")
    # 按类型取样本值
    cls = type(m).__name__
    if cls == 'LaserScan':
        n = len(m.ranges)
        mn = min(m.ranges) if n else 0
        info.append(f"点={n} min={mn:.2f}m")
    elif cls == 'Range':
        info.append(f"range={m.range:.3f}m fov={m.field_of_view:.2f}")
    elif cls == 'HeadTofPayload' or cls == 'RearTofPayload':
        for name in ('left_head', 'right_head', 'left_rear', 'right_rear'):
            if hasattr(m, name):
                d = getattr(m, name)
                if hasattr(d, 'data') and hasattr(d, 'data_available'):
                    info.append(f"{name}=avail:{d.data_available} n:{len(d.data)}")
    elif cls == 'UwbRaw':
        for f in ('dist', 'angle', 'rssi_1', 'rssi_2'):
            if hasattr(m, f):
                info.append(f"{f}={getattr(m, f):.1f}")
    elif cls == 'GpsPayload':
        info.append(f"fix={m.fix_type} sv={m.num_sv} lat={m.lat:.5f} lon={m.lon:.5f}")
    elif cls == 'TouchStatus':
        info.append(f"state=0x{m.touch_state:02x}")
    elif cls == 'BmsStatus':
        info.append(f"soc={m.batt_soc}% V={m.batt_volt} charging={m.power_wired_charging}")
    elif cls == 'Image':
        info.append(f"{m.width}x{m.height} enc={m.encoding}")
    elif cls == 'PointCloud2':
        info.append(f"{m.width}x{m.height} pts={m.width * m.height} f={m.fields[0].name if m.fields else '-'}")
    return " ".join(info)


def probe(node, executor, suffix, msg_path, need, timeout):
    mod, _, clsname = msg_path.rpartition('.')
    try:
        cls = getattr(importlib.import_module(mod), clsname)
    except Exception as e:
        return f"❌ 消息类型导入失败: {e}"
    topic = f"{NS}/{suffix}"
    got = []
    qos = QoSProfile(depth=5)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    try:
        sub = node.create_subscription(cls, topic, lambda m: got.append(m), qos)
    except Exception as e:
        return f"❌ 订阅失败: {e}"
    t0 = time.time()
    while len(got) < need and time.time() - t0 < timeout:
        executor.spin_once(timeout_sec=0.5)
    node.destroy_subscription(sub)
    return summarize(node, topic, got)


def main():
    print("=" * 60)
    print(" 传感器探测 — 逐个 topic 收帧验证")
    print(f" 命名空间: {NS}")
    print("=" * 60)
    rclpy.init()
    node = rclpy.create_node('sensor_probe')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    ok = 0
    fail = 0
    for suffix, msg_path, need, timeout in SENSORS:
        result = probe(node, executor, suffix, msg_path, need, timeout)
        status = "✅" if "❌" not in result else "⚠️"
        if "✅" == status:
            ok += 1
        else:
            fail += 1
        print(f"  {status} {suffix:42s} → {result}")
        time.sleep(0.2)

    print("=" * 60)
    print(f" 结果: {ok} 个可用, {fail} 个异常")
    print("=" * 60)
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
