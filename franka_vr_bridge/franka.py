# -- coding: UTF-8
"""Franka FR3 端：把 Bridge 输出的末端位姿交给 franky 的笛卡尔阻抗跟踪器，并控制 Franka Hand。

坐标约定沿用 ARX 头显（docs/协议.md）：Bridge 输出相对“零点”的位姿 (x y z roll pitch yaw)，
x 前 / y 左 / z 上，和 FR3 基座系同向；姿态按 ZYX，是基座系下的增量（左乘）：
    p = p0 + (x, y, z)
    R = Rzyx(roll, pitch, yaw) · R0
零点 (p0, R0) 是适配器启动时（或 --home 归位后）的末端位姿，头显上按 A 复位就回到这里。

线程：Runner 线程按 timing.rate 调 publish()，只存目标；主线程跑跟踪循环（set_target、夹爪、回报反馈）。
跟踪器掉出控制（reflex 等）时清错、重建，并让 Bridge 从机械臂实际位姿重新起步，不按旧指令冲回去。
"""
import math
import threading
import time

import numpy as np

ARMS = ('left', 'right')

FR3_LOWER = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
FR3_UPPER = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])


# ---------------------------------------------------------------------- 姿态（与 core.py 同为 ZYX）

def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy(r):
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    roll = math.atan2(r[2, 1], r[2, 2])
    yaw = math.atan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


class Zero:
    """零点：基座系下的末端位姿（4×4）。"""

    def __init__(self, matrix):
        m = np.asarray(matrix, dtype=float)
        self.p0 = m[:3, 3].copy()
        self.r0 = m[:3, :3].copy()

    def to_base(self, xyzrpy):
        t = np.eye(4)
        t[:3, :3] = rpy_to_matrix(*xyzrpy[3:6]) @ self.r0
        t[:3, 3] = self.p0 + np.asarray(xyzrpy[:3], dtype=float)
        return t

    def from_base(self, matrix):
        m = np.asarray(matrix, dtype=float)
        return [*(m[:3, 3] - self.p0), *matrix_to_rpy(m[:3, :3] @ self.r0.T)]


# ---------------------------------------------------------------------- 夹爪

class GripperCtl:
    """头显夹爪值（0 合 ~ 5 开）→ Franka Hand 二值抓取，带回差；上一条指令没完成时不发新的。"""

    def __init__(self, gripper, cfg, log):
        self.gripper = gripper
        self.close_below = float(cfg['close_below'])
        self.open_above = float(cfg['open_above'])
        self.speed = float(cfg['speed'])
        self.force = float(cfg['force'])
        self.log = log
        self.closed = None
        self.future = None

    def update(self, value):
        if value < self.close_below:
            want = True
        elif value > self.open_above:
            want = False
        else:
            return
        if want == self.closed:
            return
        if self.future is not None and not self.future.wait(0):
            return
        try:
            if want:
                self.future = self.gripper.grasp_async(0.0, self.speed, self.force, epsilon_outer=1.0)
            else:
                self.future = self.gripper.open_async(self.speed)
            self.closed = want
        except Exception as e:  # noqa: BLE001 夹爪断线不影响手臂
            self.log(f'夹爪指令失败：{e}')


# ---------------------------------------------------------------------- 手臂

class FrankaArm:
    def __init__(self, robot, cfg, log=print, gripper=None):
        import franky
        self.franky = franky
        self.robot = robot
        self.cfg = cfg['franka']
        self.log = log
        self.arm = self.cfg['arm']
        if self.arm not in ARMS:
            raise ValueError(f'franka.arm 只能是 left / right，收到 {self.arm!r}')
        self.other = 'left' if self.arm == 'right' else 'right'
        self.lock = threading.Lock()
        self.target = None   # (4×4 末端位姿, 头显夹爪值)
        self.zero = None
        self.gripper = GripperCtl(gripper, self.cfg['gripper'], log) if gripper is not None else None

    def ee_matrix(self):
        return np.asarray(self.robot.current_pose.end_effector_pose.matrix)

    def set_zero_here(self):
        self.zero = Zero(self.ee_matrix())
        p = self.zero.p0
        self.log(f'零点 = 当前末端位姿：xyz = {p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f} m，'
                 f'rpy = ' + ' '.join(f'{math.degrees(v):+.1f}' for v in matrix_to_rpy(self.zero.r0)) + '°')

    # Runner 线程调用
    def publish(self, left, right):
        f = left if self.arm == 'left' else right
        t = self.zero.to_base([f[k] for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')])
        with self.lock:
            self.target = (t, float(f['gripper']))

    def _feedback(self, bridge):
        """回报两臂反馈：用到的那只手是实际位姿，另一只给 0，头显据此显示“控制器在线”。"""
        bridge.on_arm_feedback(self.arm, self.zero.from_base(self.ee_matrix()))
        bridge.on_arm_feedback(self.other, [0.0] * 6)

    def run(self, runner, stop):
        """主线程：跟踪循环。掉出控制后清错重建，一分钟内超过 max_recover 次则返回。"""
        recover_times = []
        while not stop.is_set():
            try:
                self._track(runner, stop)
            except self.franky.ControlException as e:
                self.log(f'控制异常：{str(e).splitlines()[0][:160]}')
            if stop.is_set():
                return
            now = time.monotonic()
            recover_times = [t for t in recover_times if now - t < 60.0] + [now]
            if len(recover_times) > int(self.cfg['max_recover']):
                self.log(f'一分钟内掉出控制超过 {self.cfg["max_recover"]} 次，退出。检查是否顶住了东西，看 Desk 的报错')
                return
            self.log('跟踪器掉出控制（碰撞 / 超限等），清除错误后从当前位姿重新起步 ...')
            try:
                self.robot.recover_from_errors()
            except Exception as e:  # noqa: BLE001
                self.log(f'清除错误失败：{e}。到 Desk 查看后重启适配器')
                return
            with self.lock:
                self.target = None
            with runner.lock:
                self._feedback(runner.bridge)
                runner.bridge.publishing = False   # 下一拍从刚回报的实际位姿起步
            time.sleep(0.3)

    def _track(self, runner, stop):
        fk, c = self.franky, self.cfg
        posture = None
        if c['posture']['enabled']:
            posture = fk.PostureTask(np.asarray(c['posture']['target'], dtype=float), float(c['posture']['stiffness']))
        fb_period = 1.0 / float(c['feedback_rate'])
        next_fb = 0.0
        with fk.CartesianImpedanceTracker(
            self.robot,
            translational_stiffness=float(c['stiffness']['translational']),
            rotational_stiffness=float(c['stiffness']['rotational']),
            translational_error_clip=np.full(3, float(c['error_clip']['translational'])),
            rotational_error_clip=np.full(3, float(c['error_clip']['rotational'])),
            posture_task=posture,
            lower_joint_limits=FR3_LOWER,
            upper_joint_limits=FR3_UPPER,
            period=1.0 / float(c['loop_rate']),
        ) as tracker:
            self.log('跟踪器已启动，等待头显开启遥操（长按 B）')
            while not stop.is_set() and tracker.tick():
                with self.lock:
                    tgt = self.target
                if tgt is not None:
                    tracker.set_target(fk.Affine(tgt[0]))
                    if self.gripper is not None:
                        self.gripper.update(tgt[1])
                now = time.monotonic()
                if now >= next_fb:
                    next_fb = now + fb_period
                    runner.on_arm_feedback(self.arm, self.zero.from_base(self.ee_matrix()))
                    runner.on_arm_feedback(self.other, [0.0] * 6)
