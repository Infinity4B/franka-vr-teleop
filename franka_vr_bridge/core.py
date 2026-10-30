# -- coding: UTF-8
"""与 ROS 无关的适配器核心：收包、控制权、看门狗、限幅/插值，产出 /ARX_VR_L /ARX_VR_R 的字段。

ros2_node.py / ros1_node.py 只负责把 Bridge.tick() 的输出填进 PosCmd 发布，并把反馈喂回来。

安全约定（/ARX_VR_L 同时承载左臂位姿和底盘，lift_controller 走这个话题时没有超时停车）：
  - 超过 soft_timeout（默认 0.12 s）未收包：继续发布当前左右臂指令（不能发全 0，否则手臂被拉回零点），底盘清零、mode1=2；
  - 超过 hard_timeout，或 VR 端未解锁 / 闸门关闭：补发 stop_frames 帧停车指令后停止发布；
  - 每次开始发布时，手臂指令从 follow*_pos_back 反馈起步（没有反馈时从目标起步），按速度限幅逼近目标。
"""
import math
import time

from . import protocol as P

__version__ = '1.0.0'

ARMS = ('left', 'right')
CHASSIS_RUN = 1
CHASSIS_STOP = 2


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ---------------------------------------------------------------------- 姿态
# X7 的 Xyzrpy2Isometry 在闭源库里，这里假设 ZYX（R = Rz(yaw)·Ry(pitch)·Rx(roll)）。
# 限幅和插值只用它做中间路径，起点和终点的 rpy 原样往返，约定不对也不会改变最终位姿。

def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def quat_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(clamp(2 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def quat_angle(a, b):
    d = abs(sum(i * j for i, j in zip(a, b)))
    return 2 * math.acos(clamp(d, -1.0, 1.0))


def slerp(a, b, t):
    d = sum(i * j for i, j in zip(a, b))
    if d < 0:
        b, d = tuple(-v for v in b), -d
    if d > 0.9995:
        q = tuple(i + (j - i) * t for i, j in zip(a, b))
    else:
        th = math.acos(d)
        s = math.sin(th)
        wa, wb = math.sin((1 - t) * th) / s, math.sin(t * th) / s
        q = tuple(wa * i + wb * j for i, j in zip(a, b))
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)


def interp_pose(a, b, t):
    """a, b 为 7 元组 (x y z roll pitch yaw gripper)，t∈[0,1]。"""
    if t <= 0:
        return list(a)
    if t >= 1:
        return list(b)
    pos = [a[i] + (b[i] - a[i]) * t for i in range(3)]
    rpy = quat_to_rpy(slerp(rpy_to_quat(*a[3:6]), rpy_to_quat(*b[3:6]), t))
    return pos + list(rpy) + [a[6] + (b[6] - a[6]) * t]


def step_pose(cur, target, max_lin, max_ang, max_grip):
    """cur 朝 target 走一步。返回 (新位姿, 是否被限幅)。"""
    limited = False
    d = [target[i] - cur[i] for i in range(3)]
    dist = math.sqrt(sum(v * v for v in d))
    if dist > max_lin:
        k = max_lin / dist
        pos = [cur[i] + d[i] * k for i in range(3)]
        limited = True
    else:
        pos = list(target[:3])

    qa, qb = rpy_to_quat(*cur[3:6]), rpy_to_quat(*target[3:6])
    ang = quat_angle(qa, qb)
    if ang > max_ang:
        rpy = list(quat_to_rpy(slerp(qa, qb, max_ang / ang)))
        limited = True
    else:
        rpy = list(target[3:6])

    g = cur[6] + clamp(target[6] - cur[6], -max_grip, max_grip)
    return pos + rpy + [g], limited


# ---------------------------------------------------------------------- 配置

DEFAULT_CONFIG = {
    'network': {'bind': '0.0.0.0', 'port': P.DEFAULT_PORT, 'robot_name': 'ARX-X7s'},
    'timing': {
        'rate': 200.0,          # 发布频率 Hz
        'stat_rate': 30.0,      # STAT 回传频率 Hz
        'soft_timeout': 0.12,   # s，超过后保持手臂、底盘停车（验收要求断链 150 ms 内停车）
        'hard_timeout': 1.0,    # s，超过后补发停车帧并停止发布
        'stop_frames': 10,
        'owner_timeout': 2.0,   # s，控制权持有者多久没包后释放
        'feedback_timeout': 0.5,
    },
    'arm': {
        'max_lin_vel': 1.0,     # m/s
        'max_ang_vel': 4.0,     # rad/s
        'reset_lin_vel': 0.25,  # 复位 / 自动归零时的速度
        'reset_ang_vel': 1.0,
        'gripper_speed': 25.0,  # 每秒
        'gripper': [0.0, 5.0],
        'lag_warn': 0.02,       # m，指令落后目标超过该值视为被限幅
        'workspace': {          # 末端指令范围（与 ARX_VR_L/R 同一坐标系，零点即复位位姿）
            'left': {'x': [-0.15, 0.65], 'y': [-0.45, 0.55], 'z': [-0.35, 0.60],
                     'roll': [-3.1416, 3.1416], 'pitch': [-1.57, 1.57], 'yaw': [-3.1416, 3.1416]},
            'right': {'x': [-0.15, 0.65], 'y': [-0.55, 0.45], 'z': [-0.35, 0.60],
                      'roll': [-3.1416, 3.1416], 'pitch': [-1.57, 1.57], 'yaw': [-3.1416, 3.1416]},
        },
    },
    'chassis': {'max': [1.0, 1.0, 1.0]},   # |chx| |chy| |chz| 上限（摇杆满杆为 1）
    'lift': {'min': 0.0, 'max': 20.0, 'speed': 6.0},
}


def merge_config(base, override):
    out = dict(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            out[k] = merge_config(base[k], v)
        else:
            out[k] = v
    return out


# ---------------------------------------------------------------------- 状态

class _Arm:
    def __init__(self):
        self.target = None       # 最新目标（已钳位）
        self.prev_target = None  # 上一包目标，平滑插值用
        self.t_target = 0.0
        self.cmd = None          # 当前发布的指令
        self.resetting = False
        self.limited = False
        self.fb = None           # (x y z roll pitch yaw)
        self.fb_stamp = -1e9


class _Link:
    """当前控制者的收包统计。"""

    def __init__(self, session, addr, now):
        self.session = session
        self.addr = addr
        self.last_rx = now
        self.last_seq = None
        self.last_t_vr = 0
        self.last_rx_seq_time = now
        self.ctrl = None
        self.interval = 1 / 90
        self.win_start = now
        self.win_first = None
        self.win_last = None
        self.win_count = 0
        self.loss = 0.0
        self.rx_rate = 0.0

    def account(self, seq, now):
        if self.win_first is None:
            self.win_first = seq
        self.win_last = seq
        self.win_count += 1
        if now - self.win_start >= 1.0:
            expected = self.win_last - self.win_first + 1
            self.loss = clamp(1 - self.win_count / expected, 0.0, 1.0) if expected > 0 else 0.0
            self.rx_rate = self.win_count / (now - self.win_start)
            self.win_start, self.win_first, self.win_count = now, None, 0


class Bridge:
    """sendto(data, addr) 由运行器提供；tick() 返回 None（不发布）或 (left, right) 两个字段字典。"""

    def __init__(self, config=None, sendto=None, ros_version=0, clock=time.monotonic, log=print):
        self.cfg = merge_config(DEFAULT_CONFIG, config)
        self.sendto = sendto or (lambda data, addr: None)
        self.ros_version = ros_version
        self.clock = clock
        self.log = log
        t = self.cfg['timing']
        self.dt = 1.0 / float(t['rate'])
        self.stat_period = 1.0 / float(t['stat_rate'])
        self.arms = {a: _Arm() for a in ARMS}
        self.link = None
        self.publishing = False
        self.stop_left = 0
        self.watchdog = P.WD_IDLE
        self.time_count = 0
        self.height_cmd = None
        self.height_fb = None
        self.height_fb_stamp = -1e9
        self.last_stat = -1e9
        self.reject_reply = {}
        self.last_out = None

    # ------------------------------------------------------------------ 反馈
    def on_arm_feedback(self, arm, xyzrpy):
        a = self.arms[arm]
        a.fb = tuple(float(v) for v in xyzrpy[:6])
        a.fb_stamp = self.clock()

    def on_body_feedback(self, height):
        self.height_fb = float(height)
        self.height_fb_stamp = self.clock()

    def _fresh(self, stamp, now):
        return now - stamp < self.cfg['timing']['feedback_timeout']

    # ------------------------------------------------------------------ 收包
    def handle_packet(self, data, addr):
        now = self.clock()
        try:
            ptype = P.parse_header(data)
            if ptype == P.T_DISCOVER:
                self._on_discover(P.unpack_discover(data), addr, now)
            elif ptype == P.T_CTRL:
                self._on_ctrl(P.unpack_ctrl(data), addr, now)
        except P.ProtocolError as e:
            self.log(f'丢弃来自 {addr[0]}:{addr[1]} 的包：{e}')

    def _owner_of(self, session, now):
        if self.link is None or now - self.link.last_rx > self.cfg['timing']['owner_timeout']:
            return P.OWNER_FREE
        return P.OWNER_YOU if self.link.session == session else P.OWNER_OTHER

    def _on_discover(self, d, addr, now):
        a, l, n = self.cfg['arm'], self.cfg['lift'], self.cfg['network']
        limits = (a['max_lin_vel'], a['max_ang_vel'], l['min'], l['max'])
        # 工作空间发给头显，头显按同一范围钳位，碰到边界后往回动立即生效
        ws = [float(v) for arm in ARMS for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
              for v in a['workspace'][arm][k]]
        self.sendto(P.pack_hello(d['session'], int(n['port']), self.ros_version, self._owner_of(d['session'], now),
                                 n['robot_name'], __version__, limits, ws), addr)

    def _on_ctrl(self, c, addr, now):
        owner = self._owner_of(c['session'], now)
        if owner == P.OWNER_OTHER:
            if now - self.reject_reply.get(addr, -1e9) > 0.2:
                self.reject_reply[addr] = now
                self.sendto(P.pack_stat(self._stat_fields(c['session'], c['seq'], c['t_vr_us'], 0, P.OWNER_OTHER)),
                            addr)
            return
        if owner == P.OWNER_FREE and self.link is not None and self.link.session == c['session']:
            pass  # 同一头显断链后恢复，沿用原来的链路
        elif owner == P.OWNER_FREE:
            if self.link is not None:
                self.log(f'控制权从 {self.link.addr[0]} 释放')
            self.log(f'控制权交给 {addr[0]}:{addr[1]}（会话 {c["session"]:08x}）')
            self.link = _Link(c['session'], addr, now)
        link = self.link
        link.addr = addr
        if link.last_seq is not None and c['seq'] <= link.last_seq and link.last_seq - c['seq'] < 100000:
            return  # 过期或重复包
        if link.last_seq is not None:
            gap = now - link.last_rx
            if 0 < gap < 0.1:
                link.interval += (gap - link.interval) * 0.1
        link.last_rx = now
        link.last_seq = c['seq']
        link.last_t_vr = c['t_vr_us']
        link.account(c['seq'], now)
        link.ctrl = c
        self._apply_targets(c, now)

    def _apply_targets(self, c, now):
        ws = self.cfg['arm']['workspace']
        g_lo, g_hi = self.cfg['arm']['gripper']
        for arm in ARMS:
            a = self.arms[arm]
            raw = [float(v) for v in c[arm]]
            if not all(math.isfinite(v) for v in raw):
                self.log(f'{arm} 收到非有限数值，忽略本包')
                continue
            box = ws[arm]
            tgt = [clamp(raw[i], *box[k]) for i, k in enumerate(('x', 'y', 'z', 'roll', 'pitch', 'yaw'))]
            tgt.append(clamp(raw[6], g_lo, g_hi))
            a.limited = tgt[:6] != raw[:6]
            if a.target is None:
                a.prev_target = tgt
            else:  # 从当前插值点出发，包间隔抖动时不回跳
                a.prev_target = interp_pose(a.prev_target, a.target,
                                            (now - a.t_target) / max(self.link.interval, 1e-3))
            a.target = tgt
            a.t_target = now
            if c['flags'] & (P.F_LEFT_RESET if arm == 'left' else P.F_RIGHT_RESET):
                a.resetting = True

    # ------------------------------------------------------------------ 主循环
    def tick(self):
        """每 1/rate 秒调用一次。返回要发布的 (left, right) 或 None。"""
        now = self.clock()
        out = self._tick(now)
        if now - self.last_stat >= self.stat_period and self.link is not None \
                and now - self.link.last_rx < self.cfg['timing']['owner_timeout']:
            self.last_stat = now
            l = self.link
            hold_us = int(clamp((now - l.last_rx) * 1e6, 0, 0xFFFFFFFF))
            self.sendto(P.pack_stat(self._stat_fields(l.session, l.last_seq or 0, l.last_t_vr, hold_us,
                                                      P.OWNER_YOU)), l.addr)
        self.last_out = out
        return out

    def _tick(self, now):
        t = self.cfg['timing']
        link = self.link
        c = link.ctrl if link else None
        age = now - link.last_rx if link else math.inf
        engaged = c is not None and (c['flags'] & P.F_UNLOCKED) and (c['flags'] & P.F_GATE) \
            and all(self.arms[a].target is not None for a in ARMS)

        if age > t['hard_timeout'] or not engaged:
            if self.publishing:
                reason = '超时' if age > t['hard_timeout'] else 'VR 端未解锁或闸门关闭'
                self.log(f'{reason}，补发 {t["stop_frames"]} 帧停车指令后停止发布')
                self.publishing = False
                self.stop_left = int(t['stop_frames'])
                for a in self.arms.values():
                    a.target = list(a.cmd)
            if self.stop_left > 0:
                self.stop_left -= 1
                self.watchdog = P.WD_STOPPED if age > t['hard_timeout'] else P.WD_IDLE
                return self._frames(hold=True)
            self.watchdog = P.WD_STOPPED if (link and age > t['hard_timeout']) else P.WD_IDLE
            return None

        if not self.publishing:
            self._start(now)

        hold = False
        if age > t['soft_timeout']:
            if self.watchdog != P.WD_HOLD:
                self.log(f'{age * 1000:.0f} ms 未收包：保持手臂，底盘停车')
            for a in self.arms.values():
                a.target = list(a.cmd)
                a.prev_target = a.target
            hold = True
        elif c['flags'] & P.F_PAUSED:
            hold = True
        self.watchdog = P.WD_HOLD if hold else P.WD_ACTIVE

        smooth = bool(c['flags'] & P.F_SMOOTH) and not hold
        a_cfg = self.cfg['arm']
        for name, a in self.arms.items():
            tgt = a.target
            if smooth:
                tgt = interp_pose(a.prev_target, a.target, (now - a.t_target) / max(link.interval, 1e-3))
            lin = a_cfg['reset_lin_vel'] if a.resetting else a_cfg['max_lin_vel']
            ang = a_cfg['reset_ang_vel'] if a.resetting else a_cfg['max_ang_vel']
            a.cmd, rate_limited = step_pose(a.cmd, tgt, lin * self.dt, ang * self.dt,
                                            a_cfg['gripper_speed'] * self.dt)
            lag = math.dist(a.cmd[:3], a.target[:3])
            if a.resetting and not rate_limited and lag < 1e-4:
                a.resetting = False
            a.limited = a.limited or lag > a_cfg['lag_warn']

        lift = self.cfg['lift']
        h_tgt = clamp(float(c['height']), lift['min'], lift['max']) if math.isfinite(c['height']) else self.height_cmd
        self.height_cmd += clamp(h_tgt - self.height_cmd, -lift['speed'] * self.dt, lift['speed'] * self.dt)
        return self._frames(hold=hold)

    def _start(self, now):
        self.publishing = True
        self.stop_left = 0
        for name, a in self.arms.items():
            if self._fresh(a.fb_stamp, now):
                a.cmd = list(a.fb) + [a.target[6]]
                src = '反馈'
            elif a.cmd is not None:
                src = '上次发布的指令（没有 follow 反馈）'
            else:
                a.cmd = list(a.target)
                src = '目标（没有 follow 反馈）'
            a.prev_target = a.target
            self.log(f'{name} 开始发布，起点取自{src}：' + ' '.join(f'{v:+.3f}' for v in a.cmd))
        if self._fresh(self.height_fb_stamp, now):
            self.height_cmd = self.height_fb
        elif self.height_cmd is None:
            h = float(self.link.ctrl['height'])
            lift = self.cfg['lift']
            self.height_cmd = clamp(h, lift['min'], lift['max']) if math.isfinite(h) else lift['min']

    def _frames(self, hold):
        c = self.link.ctrl if self.link else None
        chx = chy = chz = 0.0
        mode1 = CHASSIS_STOP
        if not hold and c is not None:
            m = self.cfg['chassis']['max']
            chx, chy, chz = (clamp(float(c[k]), -m[i], m[i]) if math.isfinite(c[k]) else 0.0
                             for i, k in enumerate(('chx', 'chy', 'chz')))
            mode1 = c['mode1'] if c['mode1'] in (CHASSIS_RUN, CHASSIS_STOP) else CHASSIS_STOP
        self.time_count += 1
        out = []
        for name in ARMS:
            cmd = self.arms[name].cmd
            f = dict(zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'gripper'), cmd))
            f['time_count'] = self.time_count
            out.append(f)
        out[0].update(chx=chx, chy=chy, chz=chz, height=self.height_cmd if self.height_cmd is not None else 0.0,
                      mode1=mode1)
        return out[0], out[1]

    # ------------------------------------------------------------------ 状态
    def _stat_fields(self, session, seq, t_vr, hold_us, owner):
        now = self.clock()
        status = 0
        if self._fresh(self.arms['left'].fb_stamp, now):
            status |= P.S_LEFT_FB
        if self._fresh(self.arms['right'].fb_stamp, now):
            status |= P.S_RIGHT_FB
        if self._fresh(self.height_fb_stamp, now):
            status |= P.S_BODY_FB
        if self.arms['left'].limited:
            status |= P.S_LEFT_LIMITED
        if self.arms['right'].limited:
            status |= P.S_RIGHT_LIMITED
        if self.publishing:
            status |= P.S_PUBLISHING
        if self.ros_version == 0:
            status |= P.S_DRY_RUN
        link = self.link if owner == P.OWNER_YOU else None
        return {'session': session, 'echo_seq': seq, 'echo_t_vr_us': t_vr, 'hold_us': hold_us,
                'loss': link.loss if link else 0.0, 'status': status, 'watchdog': self.watchdog, 'owner': owner,
                'height_fb': self.height_fb if self.height_fb is not None else math.nan,
                'height_cmd': self.height_cmd if self.height_cmd is not None else math.nan,
                'rx_rate': link.rx_rate if link else 0.0}

    def shutdown_frames(self):
        """退出前调用：正在发布时返回停车帧（保持手臂），否则返回 None。"""
        if not self.publishing and self.stop_left == 0:
            return None
        self.publishing = False
        for a in self.arms.values():
            a.target = list(a.cmd)
        return self._frames(hold=True)
