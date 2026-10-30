#!/usr/bin/env python3
# -- coding: UTF-8
"""模拟头显：发现适配器、按 VR 端的节奏发 CTRL，打印 STAT（RTT、丢包、看门狗）。

取自 arx-vr-analysis/teleop/tools/fake_vr.py，改动：--arm 选画圆的手（默认右手），--scale 缩放动作幅度，去掉底盘场景。

  python3 fake_vr.py                         广播发现，跑默认脚本 circle
  python3 fake_vr.py --host 192.168.1.10     指定适配器 IP
  python3 fake_vr.py --scenario dropout      跟随 3 s 后断包 2 s（看门狗：150 ms 停车、1 s 停发）
  python3 fake_vr.py --scenario static       解锁开闸后静止（检查不漂移）
  python3 fake_vr.py --scale 0.3             动作幅度缩到 0.3 倍（真机第一次跑用小幅度）

脚本时间线（circle）：0 s 未解锁 → 0.5 s 解锁 → 1 s 开闸并两臂自动归零 → 2 s 起按住扳机画圆（x 0~0.2 m、y ±0.1 m、z ±0.05 m、pitch ±0.3 rad，乘 --scale）、
同一只手握持开合 → 结束前松开扳机、关闸。
"""
import argparse
import math
import os
import random
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from franka_vr_bridge import protocol as P  # noqa: E402

WD_NAMES = {P.WD_IDLE: '空闲', P.WD_ACTIVE: '跟随', P.WD_HOLD: '保持', P.WD_STOPPED: '已停发'}
OWNER_NAMES = {P.OWNER_FREE: '空闲', P.OWNER_YOU: '本机', P.OWNER_OTHER: '被占用'}


def discover(sock, port, session, timeout=2.0):
    sock.sendto(P.pack_discover(session, 'fake_vr'), ('255.255.255.255', port))
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        sock.settimeout(max(0.01, end - time.monotonic()))
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            break
        if P.parse_header(data) == P.T_HELLO:
            h = P.unpack_hello(data)
            print(f'发现 {h["robot_name"]} @ {addr[0]}:{h["port"]}  ROS{h["ros_version"] or "(dry-run)"}  '
                  f'v{h["bridge_version"]}  控制权={OWNER_NAMES[h["owner"]]}  限幅={h["limits"]}')
            return addr[0], h['port']
    return None


def scenario_frame(name, t, duration, arm='right', scale=1.0):
    """返回 (flags, left7, right7, chassis3, height, mode1, send?)。"""
    zero = [0.0] * 6
    flags = 0
    left, right = zero + [5.0], zero + [5.0]
    ch = (0.0, 0.0, 0.0)
    mode1 = 2
    send = True
    if t >= 0.5:
        flags |= P.F_UNLOCKED
    if t >= 1.0:
        flags |= P.F_GATE | P.F_SMOOTH
    if 1.0 <= t < 1.05:
        flags |= P.F_LEFT_RESET | P.F_RIGHT_RESET
    end = duration - 0.5
    follow = P.F_LEFT_FOLLOW if arm == 'left' else P.F_RIGHT_FOLLOW
    hand = zero + [5.0]
    if name == 'circle' and 2.0 <= t < end:
        flags |= follow
        a = (t - 2.0) * 2 * math.pi / 4.0
        k = scale
        hand = [k * (0.1 - 0.1 * math.cos(a)), k * 0.1 * math.sin(a), k * 0.05 * math.sin(2 * a), 0.0,
                k * 0.3 * math.sin(a), 0.0, 2.5 + 2.5 * math.cos(t * 2)]
    if name == 'dropout' and 2.0 <= t < end:
        flags |= follow
        hand = [scale * 0.1, scale * 0.05, scale * 0.02, 0.0, 0.0, scale * 0.2, 5.0]
        send = not (3.0 <= t < 5.0)
    if arm == 'left':
        left = hand
    else:
        right = hand
    if t >= end:
        flags &= ~P.F_GATE
    return flags, left, right, ch, 0.0, mode1, send


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host')
    ap.add_argument('--port', type=int, default=P.DEFAULT_PORT)
    ap.add_argument('--rate', type=float, default=90.0)
    ap.add_argument('--duration', type=float, default=10.0)
    ap.add_argument('--scenario', default='circle', choices=('circle', 'static', 'dropout'))
    ap.add_argument('--arm', default='right', choices=('left', 'right'), help='哪只手画圆，要和 config.yaml 的 franka.arm 一致')
    ap.add_argument('--scale', type=float, default=0.5, help='动作幅度倍数，默认 0.5（圆的直径 0.1 m）')
    ap.add_argument('--loss', type=float, default=0.0, help='模拟丢包率 0..1')
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(('0.0.0.0', 0))
    session = random.getrandbits(32)

    if args.host:
        target = (args.host, args.port)
        sock.sendto(P.pack_discover(session, 'fake_vr'), target)
        found = discover(sock, args.port, session, 0.5)
        if found:
            target = found
    else:
        found = discover(sock, args.port, session)
        if not found:
            sys.exit('没有发现适配器，请用 --host 指定 IP')
        target = found

    sock.setblocking(False)
    t0 = time.monotonic()
    seq = 0
    period = 1.0 / args.rate
    next_t = t0
    last_print = 0.0
    rtts = []
    last_stat = None
    while True:
        now = time.monotonic()
        t = now - t0
        if t > args.duration:
            break
        flags, left, right, ch, height, mode1, send = scenario_frame(args.scenario, t, args.duration, args.arm, args.scale)
        seq += 1
        if send and random.random() >= args.loss:
            sock.sendto(P.pack_ctrl({'session': session, 'seq': seq, 't_vr_us': int(now * 1e6), 'flags': flags,
                                     'mode1': mode1, 'left': left, 'right': right,
                                     'chx': ch[0], 'chy': ch[1], 'chz': ch[2], 'height': height}), target)
        while True:
            try:
                data, _ = sock.recvfrom(2048)
            except BlockingIOError:
                break
            if P.parse_header(data) != P.T_STAT:
                continue
            s = P.unpack_stat(data)
            rtt = (time.monotonic() * 1e6 - s['echo_t_vr_us'] - s['hold_us']) / 1000
            rtts.append(rtt)
            last_stat = s
        if last_stat and now - last_print > 0.5:
            last_print = now
            s = last_stat
            fb = ''.join(n for b, n in ((P.S_LEFT_FB, 'L'), (P.S_RIGHT_FB, 'R'), (P.S_BODY_FB, 'B')) if s['status'] & b)
            print(f't={t:5.2f}s seq={seq:<5d} 看门狗={WD_NAMES.get(s["watchdog"], s["watchdog"])} '
                  f'控制权={OWNER_NAMES[s["owner"]]} 发布={"是" if s["status"] & P.S_PUBLISHING else "否"} '
                  f'RTT={rtts[-1]:.2f}ms 丢包={s["loss"] * 100:.1f}% 收包={s["rx_rate"]:.0f}Hz '
                  f'反馈={fb or "-"}')
        next_t += period
        time.sleep(max(0.0, next_t - time.monotonic()))

    if rtts:
        rtts.sort()
        print(f'RTT 样本 {len(rtts)}：中位 {rtts[len(rtts) // 2]:.2f} ms，P95 {rtts[int(len(rtts) * 0.95)]:.2f} ms')


if __name__ == '__main__':
    main()
