# -- coding: UTF-8
"""UDP 收发 + 固定频率主循环。publish(left, right) 由 ROS 节点（或 dry-run）提供。"""
import csv
import select
import socket
import threading
import time

from .core import Bridge

CSV_FIELDS = ['t', 'watchdog', 'seq', 't_vr_us', 'rx_age_ms', 'flags',
              'lx', 'ly', 'lz', 'lroll', 'lpitch', 'lyaw', 'lgripper',
              'rx', 'ry', 'rz', 'rroll', 'rpitch', 'ryaw', 'rgripper',
              'chx', 'chy', 'chz', 'height', 'mode1', 'time_count']


class Runner:
    def __init__(self, config, publish, ros_version, log=print, csv_path=None):
        self.publish = publish
        self.log = log
        net = config.get('network', {}) if config else {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind((net.get('bind', '0.0.0.0'), int(net.get('port', 47800))))
        self.sock.setblocking(False)
        self.bridge = Bridge(config, sendto=self._sendto, ros_version=ros_version, log=log)
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.csv_file = None
        self.csv = None
        if csv_path:
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(CSV_FIELDS)
        port = self.sock.getsockname()[1]
        log(f'UDP 监听 {net.get("bind", "0.0.0.0")}:{port}，发布频率 {1 / self.bridge.dt:.0f} Hz')

    def _sendto(self, data, addr):
        try:
            self.sock.sendto(data, addr)
        except OSError as e:
            self.log(f'发送到 {addr[0]} 失败：{e}')

    # 反馈回调可能来自 ROS 线程
    def on_arm_feedback(self, arm, xyzrpy):
        with self.lock:
            self.bridge.on_arm_feedback(arm, xyzrpy)

    def on_body_feedback(self, height):
        with self.lock:
            self.bridge.on_body_feedback(height)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, name='arx_vr_bridge', daemon=True)
        self.thread.start()

    def stop(self):
        """停止主循环；正在发布时补发停车帧（手臂保持当前指令，底盘清零、mode1=2）。"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        with self.lock:
            out = self.bridge.shutdown_frames()
        if out:
            n = int(self.bridge.cfg['timing']['stop_frames'])
            self.log(f'退出：补发 {n} 帧停车指令')
            for _ in range(n):
                self.publish(*out)
                time.sleep(self.bridge.dt)
        self.sock.close()
        if self.csv_file:
            self.csv_file.close()

    def _loop(self):
        dt = self.bridge.dt
        next_t = time.monotonic()
        while self.running:
            wait = next_t - time.monotonic()
            if wait > 0:
                select.select([self.sock], [], [], wait)
            self._drain()
            now = time.monotonic()
            if now < next_t:
                continue
            next_t += dt
            if now - next_t > 5 * dt:  # 落后太多（调度卡顿）时不追帧
                next_t = now + dt
            with self.lock:
                out = self.bridge.tick()
            if out:
                try:
                    self.publish(*out)
                except Exception as e:  # ROS 正在关闭等
                    self.log(f'发布失败：{e}')
                if self.csv:
                    self._log_csv(now, out)

    def _drain(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError):
                return
            except OSError as e:
                self.log(f'接收失败：{e}')
                return
            with self.lock:
                self.bridge.handle_packet(data, addr)

    def _log_csv(self, now, out):
        b = self.bridge
        link = b.link
        left, right = out
        keys = ('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'gripper')
        self.csv.writerow([f'{now:.6f}', b.watchdog, link.last_seq if link else '', link.last_t_vr if link else '',
                           f'{(now - link.last_rx) * 1000:.2f}' if link else '',
                           link.ctrl['flags'] if link and link.ctrl else '',
                           *(f'{left[k]:.5f}' for k in keys), *(f'{right[k]:.5f}' for k in keys),
                           f'{left["chx"]:.3f}', f'{left["chy"]:.3f}', f'{left["chz"]:.3f}',
                           f'{left["height"]:.3f}', left['mode1'], left['time_count']])


class DryRunPrinter:
    """--dry-run：不连 ROS，按 rate 限频打印将要发布的内容。"""

    def __init__(self, rate=5.0, log=print):
        self.period = 1.0 / rate
        self.last = 0.0
        self.log = log

    def __call__(self, left, right):
        now = time.monotonic()
        if now - self.last < self.period:
            return
        self.last = now
        fmt = lambda f: ' '.join(f'{f[k]:+.3f}' for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')) + \
            f' g={f["gripper"]:.2f}'
        self.log(f'#{left["time_count"]:<7d} L[{fmt(left)}]  R[{fmt(right)}]  '
                 f'ch=({left["chx"]:+.2f},{left["chy"]:+.2f},{left["chz"]:+.2f}) '
                 f'h={left["height"]:.2f} mode1={left["mode1"]}')
