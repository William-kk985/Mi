#!/usr/bin/env python3
"""
真机低头测试 — CyberDog 2 官方接口
==================================
使用 ROS2 motion_servo_cmd topic + FORCECONTROL_DEFINITIVELY(201) 控制机身姿态。
rpy_des = [roll, pitch, yaw]，pitch 负值 = 低头。

用法:
    python3 pitch_test_servo.py --pitch -0.26 --hold 3.0 [--stand]
    先 --stand 让狗站起来（RECOVERYSTAND=111），再低头。

参考官方: /opt/ros2/cyberdog/lib/motion_action/pose_teleop.py
"""
import argparse
import sys
import time

import rclpy
from rclpy.qos import QoSProfile

from protocol.msg import MotionServoCmd, MotionID, MotionStatus
from protocol.srv import MotionResultCmd

NS = '/mi_desktop_48_b0_2d_7b_02_c7'


def wait_for_services(node, timeout=8.0):
    cli = node.create_client(MotionResultCmd, NS + '/motion_result_cmd')
    if not cli.wait_for_service(timeout_sec=timeout):
        print('[FAIL] motion_result_cmd 服务不可用，motion_manager 未激活?')
        return None
    return cli


def stand_up(node, cli, timeout=15.0):
    """让狗从趴下恢复站立 (RECOVERYSTAND=111)"""
    req = MotionResultCmd.Request()
    req.motion_id = MotionID.RECOVERYSTAND
    req.cmd_source = -1  # DEBUG 最高优先级
    req.duration = 2000
    print('[Stand] 发送 RECOVERYSTAND(111) ...')
    fut = cli.call_async(req)
    t_end = time.time() + timeout
    while not fut.done() and time.time() < t_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    if fut.done() and fut.result() is not None:
        r = fut.result()
        print(f'[Stand] result={r.result} code={r.code}')
        return r.result
    print('[Stand] 服务调用超时')
    return False


def send_pitch(node, pub, pitch, hold=3.0, body_height=0.235):
    """持续发布 motion_servo_cmd 控制机身 pitch (低头为负值)
    注意: 不能用 rate.sleep()（ROS 时钟不 tick 会阻塞），用 time.sleep。
    参考官方 pose_teleop.py: rclpy.spin_once(node, timeout_sec=0) + sleep
    """
    cmd = MotionServoCmd()
    cmd.motion_id = MotionID.FORCECONTROL_DEFINITIVELY  # 201 力控姿态-绝对
    cmd.cmd_type = MotionServoCmd.SERVO_DATA
    cmd.cmd_source = -1  # DEBUG 最高优先级
    cmd.value = 0
    cmd.vel_des = [0.0, 0.0, 0.0]
    cmd.rpy_des = [0.0, pitch, 0.0]   # roll, pitch, yaw
    cmd.pos_des = [0.0, 0.0, body_height]
    cmd.step_height = [0.05, 0.05]

    end = time.time() + hold
    n = 0
    while time.time() < end and rclpy.ok():
        pub.publish(cmd)
        n += 1
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(0.05)
    print(f'[Pitch] 已发布 {n} 帧 rpy=[0, {pitch}, 0]')
    return cmd


def reset_pitch(node, pub, hold=1.5, body_height=0.235):
    """回正"""
    send_pitch(node, pub, 0.0, hold, body_height)


def check_status(node):
    """订阅一帧 motion_status 看当前 motion"""
    got = {}

    def cb(msg):
        got['id'] = msg.motion_id
        got['switch_status'] = msg.switch_status  # 0=normal 2=estop 3=edamp
        got['ori_error'] = msg.ori_error
        got['motor_error'] = list(msg.motor_error)

    sub = node.create_subscription(
        MotionStatus, NS + '/motion_status', cb, 10)
    t_end = time.time() + 5
    while time.time() < t_end and 'id' not in got and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    if 'id' in got:
        print(f'[Status] motion_id={got["id"]} switch_status={got["switch_status"]} '
              f'ori_error={got["ori_error"]} motor_error={got["motor_error"]}')
    else:
        print('[Status] 5s 内未收到 motion_status（可能 LCM 与 MR813 断连）')
    node.destroy_subscription(sub)


def main():
    parser = argparse.ArgumentParser(description='CyberDog2 真机低头测试')
    parser.add_argument('--pitch', type=float, default=-0.20, help='pitch 弧度(负=低头, 官方限 -0.25)')
    parser.add_argument('--hold', type=float, default=3.0, help='保持秒数')
    parser.add_argument('--stand', action='store_true', help='先站立再低头')
    parser.add_argument('--height', type=float, default=0.235, help='机身高度(官方默认 0.235)')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('pitch_test_servo')
    qos = QoSProfile(depth=10)
    pub = node.create_publisher(MotionServoCmd, NS + '/motion_servo_cmd', qos)

    check_status(node)

    cli = wait_for_services(node)
    if cli is None:
        rclpy.shutdown()
        sys.exit(1)

    if args.stand:
        if not stand_up(node, cli):
            print('[WARN] 站立失败，继续尝试低头...')
        time.sleep(1.0)

    print(f'[Pitch] 低头 {args.pitch} rad，保持 {args.hold}s ...')
    send_pitch(node, pub, args.pitch, args.hold, args.height)
    print('[Pitch] 回正 ...')
    reset_pitch(node, pub, 1.5, args.height)
    print('[Done]')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
