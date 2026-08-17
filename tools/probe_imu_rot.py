#!/usr/bin/env python3
# external_imu 字段标定: 找出哪个字段是角速度(gyro)
# 用法: python3 /tmp/probe_imu_rot.py
#   前5s狗静止 → 听到提示后把狗抱起来旋转约90°(慢慢转)再放下 → 自动分析
import time, struct, math
import lcm

lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=255')
frames = []   # (t, data)

def on(ch, d):
    frames.append((time.time(), d))
lc.subscribe('external_imu', on)

def be32(d, off):
    return struct.unpack('>f', d[off:off+4])[0]
def le32(d, off):
    return struct.unpack('<f', d[off:off+4])[0]

print('[1/3] 狗保持静止 5 秒 ...')
t0 = time.time()
while time.time() - t0 < 5:
    lc.handle_timeout(50)

print('[2/3] 现在把狗抱起来, 缓慢旋转约 90°, 再放下 (给你 10 秒) ...')
while time.time() - t0 < 15:
    lc.handle_timeout(50)

print('[3/3] 狗放下保持静止 3 秒 ...')
while time.time() - t0 < 18:
    lc.handle_timeout(50)

# 分段: 静止A(0~4s) / 旋转(4~14s) / 静止B(14~18s)
fa = [d for t, d in frames if t - t0 < 4]
fb = [d for t, d in frames if 4 <= t - t0 < 14]
fc = [d for t, d in frames if t - t0 >= 14]
print('帧数: 静止A=%d 旋转=%d 静止B=%d' % (len(fa), len(fb), len(fc)))
if not (fa and fb and fc):
    print('数据不足, 重试'); raise SystemExit

# 每个偏移(0~96, step4)算 BE/LE 的静止均值 vs 旋转均值差
print('\n按"旋转期均值 - 静止期均值"变化幅度排序 (可能是gyro的字段):')
rows = []
for off in range(0, 100, 4):
    ma = sum(be32(d, off) for d in fa) / len(fa)
    mb = sum(be32(d, off) for d in fb) / len(fb)
    mc = sum(be32(d, off) for d in fc) / len(fc)
    la = sum(le32(d, off) for d in fa) / len(fa)
    lb = sum(le32(d, off) for d in fb) / len(fb)
    lc = sum(le32(d, off) for d in fc) / len(fc)
    rows.append((off, 'BE', ma, mb, mc, abs(mb - (ma + mc) / 2)))
    rows.append((off, 'LE', la, lb, lc, abs(lb - (la + lc) / 2)))
rows.sort(key=lambda r: -r[5])
print('off endian 静止A  旋转中  静止B  |变化|')
for r in rows[:14]:
    print('%3d  %-3s  %+8.4f %+8.4f %+8.4f  %.4f' % (r[0], r[1], r[2], r[3], r[4], r[5]))
