# -- coding: UTF-8
"""工作空间录制（--record）：遥操时记录头显目标、Bridge 指令、FR3 实际位姿、关节角和外力，
事后用 tools/analyze_workspace.py 分析：哪一面方框经常顶到、机器人是不是自己到不了（关节限位 / 跟不上）。

为什么要单独录：方框是在头显上钳位的（碰到边界后头显的锚点跟着移），适配器收到的目标永远不会出框，
--csv 里看不出“想往哪去”，只能看到“停在边界上多久、多少次”。所以这里记的是：
  - 目标贴在方框某一面上的时刻（= 那一刻手还想往外走，被拦住了）；
  - 指令与实际的偏差（= 机器人跟不上，多半是关节到限位、奇异或顶住了东西）；
  - 关节角（= 离关节限位还有多少）。

输出两份文件：<名字>.csv（每行一个采样）和 <名字>.json（零点、当时的方框和限速、事件）。
"""
import csv
import json
import math
import os
import threading
import time

import numpy as np

from . import protocol as P

AXES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
POS_AXES = ('x', 'y', 'z')
FIELDS = (['t', 'publishing', 'follow', 'reset', 'paused', 'watchdog']
          + ['t' + k for k in AXES]           # 头显发来的目标（头显已按方框钳位位置）
          + ['c' + k for k in AXES]           # Bridge 发给 FR3 的指令（方框 + 限速之后）
          + ['a' + k for k in AXES]           # FR3 实际末端，相对零点
          + ['bx', 'by', 'bz']                # FR3 实际末端，基座系
          + [f'q{i}' for i in range(1, 8)]    # 关节角 rad
          + ['fx', 'fy', 'fz', 'gripper'])    # 末端外力估计（基座系，N）、头显夹爪值
FACE_TOL = 1e-4   # 头显用 float32 钳位，目标和边界之差小于它就算贴在边界上


def default_path(root):
    return os.path.join(root, 'recordings', time.strftime('ws_%Y%m%d_%H%M%S.csv'))


def _f(v, nd=5):
    return '' if v is None or not math.isfinite(v) else f'{v:.{nd}f}'


class Recorder:
    def __init__(self, path, cfg, arm, zero=None, rate=100.0, log=print, extra_meta=None):
        self.path = path
        self.meta_path = os.path.splitext(path)[0] + '.json'
        self.arm = arm
        self.box = {k: [float(v) for v in cfg['arm']['workspace'][arm][k]] for k in AXES}
        self.period = 1.0 / float(rate)
        self.log = log
        self.next_t = 0.0
        self.t0 = time.monotonic()
        self.lock = threading.Lock()
        self.at_face = {}
        self.events = []
        self.n = 0
        d = os.path.dirname(os.path.abspath(path))
        os.makedirs(d, exist_ok=True)
        self.file = open(path, 'w', newline='')
        self.csv = csv.writer(self.file)
        self.csv.writerow(FIELDS)
        self.last_flush = 0.0
        a = cfg['arm']
        f = cfg.get('franka', {})
        self.meta = {
            'started': time.strftime('%Y-%m-%d %H:%M:%S'),
            'arm': arm,
            'workspace': self.box,
            'max_lin_vel': a['max_lin_vel'], 'max_ang_vel': a['max_ang_vel'], 'lag_warn': a['lag_warn'],
            'error_clip': f.get('error_clip'), 'stiffness': f.get('stiffness'),
            'zero_p0': None if zero is None else [float(v) for v in zero.p0],
            'zero_r0': None if zero is None else np.asarray(zero.r0).tolist(),
            'joint_lower': None, 'joint_upper': None,
            'rate': float(rate),
            'events': self.events,
        }
        self.meta.update(extra_meta or {})
        self._write_meta()
        log(f'录制到 {path}（{rate:.0f} Hz）')
        if zero is not None:
            z0 = zero.p0[2]
            log(f'方框（相对零点）：' + '  '.join(f'{k} {lo:+.2f}~{hi:+.2f}' for k, (lo, hi) in
                                         ((k, self.box[k]) for k in POS_AXES))
                + f'；z 对应基座上方 {z0 + self.box["z"][0]:.2f}~{z0 + self.box["z"][1]:.2f} m')

    def set_joint_limits(self, lower, upper):
        self.meta['joint_lower'] = [float(v) for v in lower]
        self.meta['joint_upper'] = [float(v) for v in upper]
        self._write_meta()

    def _write_meta(self):
        with open(self.meta_path, 'w', encoding='utf-8') as fp:
            json.dump(self.meta, fp, ensure_ascii=False, indent=1)

    def event(self, text):
        t = time.monotonic() - self.t0
        with self.lock:
            self.events.append([round(t, 3), text])
            self._write_meta()

    # ------------------------------------------------------------------ 采样
    def due(self, now):
        if now < self.next_t:
            return False
        self.next_t = max(self.next_t + self.period, now)
        return True

    def sample(self, now, runner, ee=None, q=None, force=None, zero=None):
        """runner：取 Bridge 状态（加锁）；ee：基座系末端 4×4；q：关节角；force：基座系外力 3 维。"""
        with runner.lock:
            b = runner.bridge
            c = b.link.ctrl if b.link is not None else None
            a = b.arms[self.arm]
            cmd = list(a.cmd) if a.cmd is not None else None
            publishing = b.publishing
            wd = b.watchdog
        nan6 = [math.nan] * 6
        if c is not None:
            flags = c['flags']
            tgt = [float(v) for v in c[self.arm]]
            follow = bool(flags & (P.F_RIGHT_FOLLOW if self.arm == 'right' else P.F_LEFT_FOLLOW))
            reset = bool(flags & (P.F_RIGHT_RESET if self.arm == 'right' else P.F_LEFT_RESET))
            paused = bool(flags & P.F_PAUSED)
        else:
            tgt, follow, reset, paused = nan6 + [math.nan], False, False, False
        act = nan6
        base = [math.nan] * 3
        if ee is not None and zero is not None:
            act = zero.from_base(ee)
            base = [float(v) for v in np.asarray(ee)[:3, 3]]
        qv = [math.nan] * 7 if q is None else [float(v) for v in q]
        fv = [math.nan] * 3 if force is None else [float(v) for v in force[:3]]
        if publishing and follow and not reset:
            self._check_faces(tgt)
        row = ([f'{now - self.t0:.4f}', int(publishing), int(follow), int(reset), int(paused), wd]
               + [_f(v) for v in tgt[:6]] + [_f(v) for v in (cmd[:6] if cmd else nan6)]
               + [_f(v) for v in act] + [_f(v) for v in base] + [_f(v) for v in qv]
               + [_f(v, 2) for v in fv] + [_f(tgt[6], 2)])
        self.csv.writerow(row)
        self.n += 1
        if now - self.last_flush > 1.0:
            self.last_flush = now
            self.file.flush()

    def _check_faces(self, tgt):
        """目标刚贴上某一面方框时打一行日志，方便现场对照。"""
        for i, k in enumerate(POS_AXES):
            lo, hi = self.box[k]
            face = '-' if tgt[i] <= lo + FACE_TOL else '+' if tgt[i] >= hi - FACE_TOL else ''
            if face and self.at_face.get(k) != face:
                self.log(f'[录制] 到边界 {k}{face}（{lo if face == "-" else hi:+.2f}）')
            self.at_face[k] = face

    def run_thread(self, runner, stop):
        """dry-run 用：没有机器人，只按频率记头显目标和 Bridge 指令。"""
        def loop():
            while not stop.is_set():
                now = time.monotonic()
                if self.due(now):
                    self.sample(now, runner)
                time.sleep(self.period / 4)
        th = threading.Thread(target=loop, name='recorder', daemon=True)
        th.start()
        return th

    def close(self):
        self.file.close()
        self.meta['ended'] = time.strftime('%Y-%m-%d %H:%M:%S')
        self.meta['samples'] = self.n
        self._write_meta()
        self.log(f'录制结束：{self.n} 个采样 → {self.path}')
        self.log(f'分析：uv run --frozen python tools/analyze_workspace.py {self.path}')
