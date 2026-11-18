# -- coding: UTF-8
"""只动 Franka Hand（手臂不动）：测夹爪每条指令的固定开销，判断能不能做连续跟随。

    uv run python tools/gripper_latency.py [--host 192.168.3.1] [--yes]

夹爪会开合十几次，手和东西远离指尖。不需要 FCI 激活，也不要和 start.sh 同时跑。
所有读开口都在主线程里做：夹爪状态约 10 Hz 才来一包，两个线程同时读会互相抢包、一等就是一两秒。
  1. 原地 move（距离 0）：纯指令开销
  2. 合拢 / 张开 5 / 10 / 20 / 40 mm：发指令到完成
  3. 发 move 后多久开口开始变（运动启动延迟）
  4. 合拢途中 stop：stop 本身多久完成
  5. “开始走 + 到位就 stop”：往全合走，读到开口 ≤ 目标就 stop，看停在哪（过冲多少）
"""
import argparse
import os
import sys
import time


def ms(v):
    return '   -   ' if v is None else f'{v * 1000:6.0f} ms'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default=os.environ.get('FRANKA_IP', '192.168.3.1'))
    ap.add_argument('--speed', type=float, default=0.1)
    ap.add_argument('--yes', action='store_true')
    args = ap.parse_args()

    import franky
    speed = args.speed
    print(f'连接夹爪 {args.host} ...', flush=True)
    g = franky.Gripper(args.host)
    mw = float(g.max_width)
    print(f'max_width = {mw:.4f} m，当前开口 {float(g.width):.4f} m，speed = {speed} m/s')
    if mw <= 0:
        sys.exit('夹爪没做过 homing')
    if not args.yes and input('夹爪会开合十几次，手和东西远离指尖，输入 yes 继续：').strip().lower() != 'yes':
        sys.exit('已取消')

    def move(width):
        t0 = time.monotonic()
        ok = g.move_async(width, speed).wait(10.0)
        return time.monotonic() - t0, ok

    print('\n== 0. 先全开', flush=True)
    move(mw)

    print('\n== 1. 原地 move（距离 0）')
    for _ in range(3):
        w = float(g.width)
        t, ok = move(w)
        print(f'  完成 {ms(t)}{"" if ok else "（超时）"}', flush=True)

    print('\n== 2. 合拢再张开（发指令到完成；理论运动时间 = 距离 / 速度）')
    for d in (0.005, 0.010, 0.020, 0.040):
        for target in (mw - d, mw):
            t, ok = move(target)
            print(f'  {d * 1000:4.0f} mm {"合" if target < mw else "开"}  完成 {ms(t)}  理论 {ms(d / speed)}'
                  f'  多出 {ms(t - d / speed)}{"" if ok else "（超时）"}', flush=True)

    print('\n== 3. 运动启动延迟（主线程轮询开口）')
    for _ in range(3):
        move(mw)
        w0 = float(g.width)
        t0 = time.monotonic()
        f = g.move_async(0.01, speed)
        t_start = None
        while time.monotonic() - t0 < 3.0:
            w = float(g.width)
            if abs(w - w0) > 0.0005:
                t_start = time.monotonic() - t0
                break
        f.wait(10.0)
        print(f'  开口开始变：{ms(t_start)}（状态包间隔约 100 ms，误差在这个量级）', flush=True)

    print('\n== 4. 合拢途中 stop')
    for delay in (0.2, 0.4):
        move(mw)
        f_move = g.move_async(0.01, speed)
        time.sleep(delay)
        t1 = time.monotonic()
        f_stop = g.stop_async()      # franky 会等 f_move 结束才返回
        t_call = time.monotonic() - t1
        f_stop.wait(10.0)
        t_done = time.monotonic() - t1
        w1 = float(g.width)
        time.sleep(0.5)
        w2 = float(g.width)
        print(f'  {delay:.1f} s 后 stop：stop_async 返回 {ms(t_call)}，stop 完成 {ms(t_done)}，'
              f'停后开口 {w1 * 1000:.1f} → 0.5 s 后 {w2 * 1000:.1f} mm', flush=True)

    print('\n== 5. 开始走 + 到位就 stop（往全合走，开口 ≤ 目标时 stop）')
    for target in (0.06, 0.04, 0.02):
        move(mw)
        t0 = time.monotonic()
        f = g.move_async(0.0, speed)
        t_hit = None
        while time.monotonic() - t0 < 3.0:
            if float(g.width) <= target:
                t_hit = time.monotonic() - t0
                break
        g.stop_async().wait(10.0)
        f.wait(10.0)
        time.sleep(0.3)
        w = float(g.width)
        print(f'  目标 {target * 1000:.0f} mm：{ms(t_hit)} 时读到到位，停在 {w * 1000:.1f} mm，'
              f'过冲 {(target - w) * 1000:+.1f} mm，总用时 {ms(time.monotonic() - t0 - 0.3)}', flush=True)

    move(mw)
    print('\n测完，夹爪已全开')


if __name__ == '__main__':
    main()
