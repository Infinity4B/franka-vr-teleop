# -- coding: UTF-8
"""只动 Franka Hand（手臂不动）：测夹爪指令的延迟，以及 linear 模式能不能跟上握持键。

    uv run --project ../franky_ctl python tools/gripper_latency.py [--host 192.168.3.1] [--yes] [--csv out.csv]

夹爪会开合十几次，手和东西远离指尖。不需要 FCI 激活，也不要和 start.sh 同时跑。
测四项：
  1. 原地 move（距离 0）的耗时：纯指令开销
  2. 合拢 5 / 10 / 20 / 40 mm：从发指令到开口开始变、到指令完成各要多久
  3. stop：合拢途中 0.15 s 时 stop，看 stop_async 本身卡调用方多久、开口多久停住
  4. 用适配器里的 GripperCtl（linear）跟一段模拟的握持键曲线，统计目标开口和实际开口的差
"""
import argparse
import csv
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from franka_vr_bridge.__main__ import load_config  # noqa: E402
from franka_vr_bridge.franka import GripperCtl, GRIPPER_OPEN_VALUE  # noqa: E402


class WidthPoller:
    """后台不停读开口，记 (t, width)。"""

    def __init__(self, gripper):
        self.gripper = gripper
        self.samples = []
        self.error = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                self.samples.append((time.monotonic(), float(self.gripper.width)))
            except Exception as e:  # noqa: BLE001
                self.error = e
                time.sleep(0.01)

    def last(self):
        return self.samples[-1][1] if self.samples else float('nan')

    def since(self, t0):
        return [(t, w) for t, w in self.samples if t >= t0]

    def rate(self):
        s = self.samples[-200:]
        return (len(s) - 1) / (s[-1][0] - s[0][0]) if len(s) > 1 and s[-1][0] > s[0][0] else 0.0


class CountingGripper:
    """包一层，记下 GripperCtl 发了哪些指令。"""

    def __init__(self, gripper):
        self.g = gripper
        self.calls = []

    def __getattr__(self, name):
        attr = getattr(self.g, name)
        if name.endswith('_async'):
            def wrapped(*a, **k):
                t = time.monotonic()
                f = attr(*a, **k)
                self.calls.append((name, t, time.monotonic() - t))
                return f
            return wrapped
        return attr


def first_change(samples, w0, thresh=0.0005):
    for t, w in samples:
        if abs(w - w0) > thresh:
            return t
    return None


def timed_move(gripper, poller, width, speed):
    w0 = poller.last()
    t0 = time.monotonic()
    f = gripper.move_async(width, speed)
    t_call = time.monotonic() - t0
    f.wait()
    t_done = time.monotonic() - t0
    time.sleep(0.1)
    tc = first_change(poller.since(t0), w0)
    return t_call, (tc - t0) if tc else None, t_done, w0, poller.last()


def ms(v):
    return '   -   ' if v is None else f'{v * 1000:6.0f} ms'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default=os.environ.get('FRANKA_IP', '192.168.3.1'))
    ap.add_argument('--yes', action='store_true')
    ap.add_argument('--csv', help='第 4 项的目标 / 实际开口写入 CSV')
    args = ap.parse_args()

    import franky
    cfg = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml'))
    gcfg = dict(cfg['franka']['gripper'], mode='linear')
    speed = float(gcfg['speed'])

    print(f'连接夹爪 {args.host} ...', flush=True)
    gripper = franky.Gripper(args.host)
    mw = float(gripper.max_width)
    print(f'max_width = {mw:.4f} m，当前开口 {float(gripper.width):.4f} m，speed = {speed} m/s')
    if mw <= 0:
        sys.exit('夹爪没做过 homing')
    if not args.yes and input('夹爪会开合十几次，手和东西远离指尖，输入 yes 继续：').strip().lower() != 'yes':
        sys.exit('已取消')

    poller = WidthPoller(gripper)
    time.sleep(0.5)
    print(f'读开口的频率约 {poller.rate():.0f} Hz' + (f'（读取出错：{poller.error}）' if poller.error else ''))

    print('\n== 0. 先全开')
    gripper.move_async(mw, speed).wait()

    print('\n== 1. 原地 move（距离 0）')
    for _ in range(3):
        t_call, _, t_done, _, _ = timed_move(gripper, poller, poller.last(), speed)
        print(f'  调用 {ms(t_call)}  完成 {ms(t_done)}')

    print('\n== 2. 合拢再张开（距离 / 调用返回 / 开口开始变 / 指令完成 / 实际走了）')
    for d in (0.005, 0.010, 0.020, 0.040):
        for target in (mw - d, mw):
            t_call, t_move, t_done, w0, w1 = timed_move(gripper, poller, target, speed)
            print(f'  {d * 1000:4.0f} mm {"合" if target < mw else "开"}  调用 {ms(t_call)}  开始变 {ms(t_move)}'
                  f'  完成 {ms(t_done)}  走了 {abs(w1 - w0) * 1000:5.1f} mm')

    print('\n== 3. 合拢途中 stop')
    for delay in (0.15, 0.3):
        gripper.move_async(mw, speed).wait()
        t0 = time.monotonic()
        f_move = gripper.move_async(0.01, speed)
        time.sleep(delay)
        t1 = time.monotonic()
        f_stop = gripper.stop_async()
        t_stop_call = time.monotonic() - t1
        f_move_done = f_move.wait(5.0)
        t_move_end = time.monotonic() - t1
        f_stop.wait(5.0)
        t_stop_end = time.monotonic() - t1
        try:
            move_ok = f_move.get()
        except Exception as e:  # noqa: BLE001
            move_ok = f'异常 {e}'
        time.sleep(0.3)
        # 开口最后一次变化的时刻
        s = poller.since(t1)
        t_still = None
        for i in range(len(s) - 1, 0, -1):
            if abs(s[i][1] - s[i - 1][1]) > 0.0002:
                t_still = s[i][0] - t1
                break
        print(f'  {delay:.2f} s 后 stop：stop_async 卡调用方 {ms(t_stop_call)}  move 结束 {ms(t_move_end)}'
              f'（{"完成" if f_move_done else "超时"}，返回 {move_ok}）  stop 完成 {ms(t_stop_end)}'
              f'  开口最后在 {ms(t_still)} 时还在变，停在 {poller.last() * 1000:.1f} mm')

    print('\n== 4. linear 跟随模拟握持键（按 200 Hz 调 set()，和适配器一样）')
    gripper.move_async(mw, speed).wait()
    counting = CountingGripper(gripper)
    ctl = GripperCtl(counting, gcfg, log=lambda s: print('  ' + s, flush=True))

    def value_at(t):
        """握持键曲线（夹爪值 5 开 ~ 1.2 合，不进抓取区）：1 s 慢合、停 0.5 s、1 s 慢开、停 0.5 s、0.4 s 快合、0.4 s 快开、来回抖 2 次。"""
        lo, hi = 1.2, GRIPPER_OPEN_VALUE
        segs = [(1.0, hi, lo), (0.5, lo, lo), (1.0, lo, hi), (0.5, hi, hi), (0.4, hi, lo), (0.4, lo, hi),
                (0.5, hi, hi), (0.3, hi, 3.0), (0.3, 3.0, hi), (0.3, hi, 3.0), (0.3, 3.0, hi), (1.5, hi, hi)]
        for dur, a, b in segs:
            if t < dur:
                return a + (b - a) * t / dur
            t -= dur
        return None

    rows = []
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        v = value_at(t)
        if v is None:
            break
        ctl.set(v)
        rows.append((t, v, mw * min(max(v / GRIPPER_OPEN_VALUE, 0.0), 1.0), poller.last()))
        time.sleep(0.005)
    ctl.mode = 'binary'   # 让工作线程不再发 linear 指令
    ctl.value = GRIPPER_OPEN_VALUE + 1.0

    errs = [abs(r[2] - r[3]) for r in rows]
    print(f'  目标与实际开口差：平均 {sum(errs) / len(errs) * 1000:.1f} mm，最大 {max(errs) * 1000:.1f} mm')
    names = [c[0] for c in counting.calls]
    print(f'  发出指令：move {names.count("move_async")} 次，stop {names.count("stop_async")} 次，'
          f'grasp {names.count("grasp_async")} 次；单次调用最长卡 {max((c[2] for c in counting.calls), default=0) * 1000:.0f} ms')
    # 估计整体滞后：把实际曲线往前挪多少和目标最接近
    best = None
    for lag_ms in range(0, 1500, 20):
        e = 0.0
        n = 0
        j = 0
        for t, _, tw, _ in rows:
            while j < len(rows) - 1 and rows[j][0] < t + lag_ms / 1000:
                j += 1
            if rows[j][0] >= t + lag_ms / 1000:
                e += abs(tw - rows[j][3])
                n += 1
        if n and (best is None or e / n < best[1]):
            best = (lag_ms, e / n)
    if best:
        print(f'  整体滞后约 {best[0]} ms（挪过去后平均差 {best[1] * 1000:.1f} mm）')
    print('  每 0.1 s：时间 / 目标开口 / 实际开口 (mm)')
    k = 0.0
    for t, _, tw, w in rows:
        if t >= k:
            print(f'    {t:4.1f} s  {tw * 1000:5.1f}  {w * 1000:5.1f}')
            k += 0.1
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['t', 'value', 'target_width', 'width'])
            wr.writerows(rows)
        print(f'  已写入 {args.csv}')

    time.sleep(0.5)
    gripper.move_async(mw, speed).wait()
    poller.running = False
    print('\n测完，夹爪已全开')


if __name__ == '__main__':
    main()
