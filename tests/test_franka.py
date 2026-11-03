# -- coding: UTF-8
"""Franka 端的离线测试：坐标换算、publish → 目标位姿、反馈、夹爪回差、掉出控制后的重新起步。
不需要 franky 和机器人：用假的 franky 模块和假机器人。"""
import math
import os
import sys
import threading
import types
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from franka_vr_bridge import franka as F  # noqa: E402
from franka_vr_bridge.__main__ import load_config  # noqa: E402
from franka_vr_bridge.core import rpy_to_quat  # noqa: E402

READY_EE = np.array([[1.0, 0.0, 0.0, 0.307],
                     [0.0, -1.0, 0.0, 0.0],
                     [0.0, 0.0, -1.0, 0.487],
                     [0.0, 0.0, 0.0, 1.0]])   # ready 姿态：夹爪朝下


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class FakeFranky(types.ModuleType):
    class ControlException(Exception):
        pass

    class Affine:
        def __init__(self, m):
            self.matrix = np.asarray(m)


class FakeRobot:
    def __init__(self, ee):
        self.ee = np.array(ee, dtype=float)
        self.recovered = 0

    @property
    def current_pose(self):
        return types.SimpleNamespace(end_effector_pose=types.SimpleNamespace(matrix=self.ee.copy()))

    def recover_from_errors(self):
        self.recovered += 1


class FakeFuture:
    def __init__(self, done=True):
        self.done = done

    def wait(self, timeout):
        return self.done


class FakeGripper:
    max_width = 0.08

    def __init__(self):
        self.calls = []
        self.done = True
        self.width = 0.08

    def grasp_async(self, width, speed, force, epsilon_outer):
        self.calls.append(('grasp', force))
        return FakeFuture(self.done)

    def open_async(self, speed):
        self.calls.append(('open',))
        return FakeFuture(self.done)

    def move_async(self, width, speed):
        self.calls.append(('move', round(width, 4)))
        return FakeFuture(self.done)

    def stop_async(self):
        self.calls.append(('stop',))
        return FakeFuture(self.done)


def make_arm(arm='right', gripper=None, gripper_mode='linear'):
    sys.modules['franky'] = FakeFranky('franky')
    cfg = load_config(None)
    cfg['franka']['arm'] = arm
    cfg['franka']['gripper']['mode'] = gripper_mode
    robot = FakeRobot(READY_EE)
    a = F.FrankaArm(robot, cfg, log=lambda s: None, gripper=gripper)
    a.set_zero_here()
    return a, robot


def fields(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, gripper=5.0):
    return dict(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, gripper=gripper, time_count=0)


class TestFrames(unittest.TestCase):
    def test_rpy_matches_core_convention(self):
        for rpy in [(0.3, -0.2, 1.1), (-1.0, 0.5, -2.0), (0.0, 1.2, 0.4)]:
            np.testing.assert_allclose(F.rpy_to_matrix(*rpy), quat_to_matrix(rpy_to_quat(*rpy)), atol=1e-12)

    def test_rpy_roundtrip(self):
        for rpy in [(0.3, -0.2, 1.1), (-1.0, 0.5, -2.0), (2.9, 1.0, -3.0)]:
            np.testing.assert_allclose(F.matrix_to_rpy(F.rpy_to_matrix(*rpy)), rpy, atol=1e-12)

    def test_zero_roundtrip(self):
        z = F.Zero(READY_EE)
        rel = [0.1, -0.05, 0.02, 0.2, -0.1, 0.3]
        np.testing.assert_allclose(z.from_base(z.to_base(rel)), rel, atol=1e-12)
        np.testing.assert_allclose(z.to_base([0.0] * 6), READY_EE, atol=1e-12)

    def test_rotation_is_world_frame_increment(self):
        """yaw 是绕基座 z 轴转：夹爪朝下时，末端 x 轴在水平面里转过 yaw。"""
        z = F.Zero(READY_EE)
        t = z.to_base([0, 0, 0, 0, 0, math.pi / 2])
        np.testing.assert_allclose(t[:3, 0], [0, 1, 0], atol=1e-12)   # 原来朝 +x，转到 +y
        np.testing.assert_allclose(t[:3, 2], [0, 0, -1], atol=1e-12)  # 夹爪仍朝下


class TestArm(unittest.TestCase):
    def test_publish_uses_configured_arm(self):
        a, _ = make_arm('right')
        a.publish(fields(x=0.5), fields(x=0.1, z=-0.05, gripper=1.0))
        t, g = a.target
        np.testing.assert_allclose(t[:3, 3], [0.407, 0.0, 0.437], atol=1e-12)
        self.assertEqual(g, 1.0)
        a2, _ = make_arm('left')
        a2.publish(fields(y=0.2), fields())
        np.testing.assert_allclose(a2.target[0][:3, 3], [0.307, 0.2, 0.487], atol=1e-12)

    def test_feedback_reports_both_arms(self):
        a, robot = make_arm('right')
        robot.ee[:3, 3] += [0.05, 0.0, -0.1]
        got = {}
        bridge = types.SimpleNamespace(on_arm_feedback=lambda arm, v: got.__setitem__(arm, v))
        a._feedback(bridge)
        np.testing.assert_allclose(got['right'], [0.05, 0.0, -0.1, 0, 0, 0], atol=1e-12)
        self.assertEqual(got['left'], [0.0] * 6)

    def test_bad_arm_name(self):
        sys.modules['franky'] = FakeFranky('franky')
        cfg = load_config(None)
        cfg['franka']['arm'] = 'both'
        with self.assertRaises(ValueError):
            F.FrankaArm(FakeRobot(READY_EE), cfg)

    def test_recover_restarts_from_feedback(self):
        """掉出控制后：清错、清掉旧目标、回报当前位姿并让 Bridge 下一拍从反馈起步。"""
        a, robot = make_arm('right')
        a.publish(fields(), fields(x=0.2))
        calls = {'n': 0}
        stop = threading.Event()

        def fake_track(runner, stop_):
            calls['n'] += 1
            if calls['n'] >= 2:
                stop.set()

        a._track = fake_track
        got = {}
        bridge = types.SimpleNamespace(on_arm_feedback=lambda arm, v: got.__setitem__(arm, v), publishing=True)
        runner = types.SimpleNamespace(lock=threading.Lock(), bridge=bridge)
        a.run(runner, stop)
        self.assertEqual(robot.recovered, 1)
        self.assertIsNone(a.target)
        self.assertFalse(bridge.publishing)
        self.assertIn('right', got)

    def test_recover_gives_up(self):
        a, robot = make_arm('right')
        a.cfg['max_recover'] = 2
        a._track = lambda runner, stop: None
        bridge = types.SimpleNamespace(on_arm_feedback=lambda arm, v: None, publishing=True)
        runner = types.SimpleNamespace(lock=threading.Lock(), bridge=bridge)
        F.time.sleep, orig = (lambda s: None), F.time.sleep
        try:
            a.run(runner, threading.Event())
        finally:
            F.time.sleep = orig
        self.assertEqual(robot.recovered, 2)


class TestGripper(unittest.TestCase):
    def test_linear_follow(self):
        g = FakeGripper()
        a, _ = make_arm('right', gripper=g)
        ctl = a.gripper
        ctl.update(5.0)                # 已经全开，不发
        ctl.update(4.9)                # 差 1.6 mm，在死区内
        self.assertEqual(g.calls, [])
        ctl.update(4.0)
        self.assertEqual(g.calls, [('move', 0.064)])
        g.width = 0.064
        g.done = False
        ctl.update(3.5)
        self.assertEqual(g.calls[-1], ('move', 0.056))
        ctl.update(2.5)                # 同方向更远：等这一步走完，不停
        ctl.update(2.0)
        self.assertEqual(len(g.calls), 2)
        ctl.future.done = True
        g.width = 0.056
        ctl.update(2.5)
        self.assertEqual(g.calls[-1], ('move', 0.04))
        ctl.update(4.5)                # 掉头：先停
        self.assertEqual(g.calls[-1], ('stop',))
        ctl.update(4.5)                # 还在停，不重复发
        self.assertEqual(len(g.calls), 4)
        ctl.future.done = True
        g.width = 0.05
        ctl.update(4.5)
        self.assertEqual(g.calls[-1], ('move', 0.072))

    def test_linear_after_stop_keeps_direction(self):
        """停下后重新出发的那一步，方向按实际开口算：继续同向时不再停。"""
        g = FakeGripper()
        a, _ = make_arm('right', gripper=g)
        ctl = a.gripper
        g.done = False
        ctl.update(2.5)                # 0.08 → 0.04，合拢
        ctl.update(4.5)                # 掉头 → stop
        ctl.future.done = True
        g.width = 0.06
        ctl.update(4.5)                # 0.06 → 0.072，张开
        ctl.update(5.0)                # 同向更远，等
        self.assertEqual([c[0] for c in g.calls], ['move', 'stop', 'move'])

    def test_linear_stopped_at_target(self):
        g = FakeGripper()
        a, _ = make_arm('right', gripper=g)
        ctl = a.gripper
        g.done = False
        ctl.update(2.5)
        ctl.update(4.0)                # stop
        ctl.future.done = True
        g.width = 0.0641               # 停下时正好在新目标附近
        ctl.update(4.0)
        self.assertEqual([c[0] for c in g.calls], ['move', 'stop'])

    def test_linear_grasp_and_release(self):
        g = FakeGripper()
        a, _ = make_arm('right', gripper=g)
        ctl = a.gripper
        g.done = False
        ctl.update(3.0)
        ctl.update(0.2)                # 按到底：先停正在走的 move，再带力抓
        self.assertEqual(g.calls[-1], ('stop',))
        ctl.future.done = True
        ctl.update(0.2)
        self.assertEqual(g.calls[-1], ('grasp', 30.0))
        ctl.update(0.8)                # 回差区内保持抓紧
        self.assertEqual(len(g.calls), 3)
        ctl.future.done = True
        g.width = 0.01
        ctl.update(1.5)
        self.assertEqual(g.calls[-1], ('move', 0.024))

    def test_set_does_not_block(self):
        """夹爪指令卡住时，跟踪循环调 set() 也要马上返回。"""
        g = FakeGripper()
        release = threading.Event()
        g.move_async = lambda width, speed: (release.wait(2.0), FakeFuture(True))[1]
        a, _ = make_arm('right', gripper=g)
        t0 = F.time.monotonic()
        a.gripper.set(2.0)
        F.time.sleep(0.05)
        a.gripper.set(1.5)
        self.assertLess(F.time.monotonic() - t0, 0.5)
        release.set()

    def test_binary_hysteresis_and_busy(self):
        g = FakeGripper()
        a, _ = make_arm('right', gripper=g, gripper_mode='binary')
        ctl = a.gripper
        ctl.update(5.0)
        self.assertEqual(g.calls, [('open',)])
        ctl.update(2.5)               # 回差区内不动
        ctl.update(4.0)               # 已经是张开
        self.assertEqual(len(g.calls), 1)
        g.done = False
        ctl.update(1.0)
        self.assertEqual(g.calls[-1], ('grasp', 30.0))
        ctl.update(5.0)               # 上一条没完成，不发
        self.assertEqual(len(g.calls), 2)
        ctl.future.done = True
        ctl.update(5.0)
        self.assertEqual(g.calls[-1], ('open',))


class TestConfig(unittest.TestCase):
    def test_repo_config_loads(self):
        cfg = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml'))
        self.assertEqual(cfg['network']['robot_name'], 'Franka-FR3')
        self.assertEqual(cfg['franka']['arm'], 'right')
        for arm in ('left', 'right'):
            for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
                lo, hi = cfg['arm']['workspace'][arm][k]
                self.assertLess(lo, hi)


if __name__ == '__main__':
    unittest.main()
