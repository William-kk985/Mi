#!/usr/bin/env python3
# 探测 RT板工业级IMU: LCM通道 "myIMU" (microstrain_lcmt)
# 用法(NX): python3 /tmp/probe_myimu.py
# 输出: 前5包数据 + 5s收包数(频率) + 静止omega噪声
import lcm
import struct
import time
import statistics

N = 0
OMEGA = []
FIRST = []


def cb(ch, data):
    global N
    N += 1
    # LCM 报文: hash(8B大端) + payload; microstrain: 17 float + 3 int64, 全部大端
    payload = data[8:]
    if len(payload) < 17 * 4 + 3 * 8:
        return
    f = struct.unpack('>17f3q', payload[:17 * 4 + 3 * 8])
    quat = f[0:4]
    rpy = f[4:7]
    rpy_imu = f[7:10]
    omega = f[10:13]
    acc = f[13:16]
    OMEGA.append(omega[2])
    if len(FIRST) < 5:
        FIRST.append((quat, rpy, rpy_imu, omega, acc, f[19], f[20]))


def main():
    # 对照组: global_to_robot 应正常收(验证LCM通道本身通)
    lc0 = lcm.LCM('udpm://239.255.76.67:7667?ttl=255')
    ctrl = {'n': 0}
    lc0.subscribe('global_to_robot', lambda c, d: ctrl.__setitem__('n', ctrl['n'] + 1))
    t0 = time.time()
    while time.time() - t0 < 3.0:
        lc0.handle_timeout(100)
    print('[对照] global_to_robot 3s = %d 包' % ctrl['n'])
    del lc0

    urls = [
        ('7667', 'udpm://239.255.76.67:7667?ttl=255'),
        ('7668', 'udpm://239.255.76.67:7668?ttl=255'),
        ('7670', 'udpm://239.255.76.67:7670?ttl=1'),
    ]
    for name, url in urls:
        global N
        N = 0
        del FIRST[:]
        del OMEGA[:]
        try:
            lc = lcm.LCM(url)
            lc.subscribe('myIMU', cb)
        except Exception as e:
            print('[%s] LCM 初始化失败: %s' % (name, e))
            continue
        print('[%s] 收3秒...' % name)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            lc.handle_timeout(100)
        if N > 0:
            print('  ✅ 3s 收包 = %d (≈%.1f Hz)' % (N, N / 3.0))
            for i, (q, r, ri, w, a, good, bad) in enumerate(FIRST):
                print('  [%d] quat=%.3f,%.3f,%.3f,%.3f  rpy=(%.2f,%.2f,%.2f)deg=(%.1f,%.1f,%.1f)'
                      % (i, q[0], q[1], q[2], q[3], r[0], r[1], r[2],
                         r[0] * 57.3, r[1] * 57.3, r[2] * 57.3))
                print('       omega=(%.4f,%.4f,%.4f)  acc=(%.2f,%.2f,%.2f) good=%d bad=%d'
                      % (w[0], w[1], w[2], a[0], a[1], a[2], good, bad))
            if len(OMEGA) > 10:
                print('  omega_z 统计: mean=%.5f std=%.5f' %
                      (statistics.mean(OMEGA), statistics.stdev(OMEGA)))
        else:
            print('  ❌ 0 包')


if __name__ == '__main__':
    main()
