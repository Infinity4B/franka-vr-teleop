# -- coding: UTF-8
"""工作空间录制和分析的离线测试：假机器人 + 真 Bridge，模拟一段“z 往下顶到边界”和一段“关节 7 到限位、跟不上”。"""
import math
import os
import subprocess
import sys
import tempfile
import threading
import types
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from franka_vr_bridge import protocol as P  # noqa: E402
from franka_vr_bridge.__main__ import parse_ws  # noqa: E402
from franka_vr_bridge.core import Bridge  # noqa: E402
from franka_vr_bridge.franka import FR3_LOWER, FR3_UPPER  # noqa: E402
from franka_vr_bridge.recorder import Recorder  # noqa: E402
from test_franka import make_arm  # noqa: E402

READY_Q = np.array([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854])


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def ctrl(seq, right, follow=True):
    flags = P.F_UNLOCKED | P.F_GATE | (P.F_RIGHT_FOLLOW if follow else 0)
    return P.pack_ctrl({'session': 1, 'seq': seq, 't_vr_us': seq * 11111, 'flags': flags, 'mode1': 2,
                        'left': [0.0] * 6 + [5.0], 'right': list(right) + [5.0],
                        'chx': 0.0, 'chy': 0.0, 'chz': 0.0, 'height': 0.0})


class TestRecorder(unittest.TestCase):
    def test_parse_ws(self):
        self.assertEqual(parse_ws(['z=-0.3,0.45', 'y=-0.5,0.5']), {'z': [-0.3, 0.45], 'y': [-0.5, 0.5]})
        for bad in (['q=0,1'], ['z=0.4,0.1'], ['z=0.1']):
            with self.assertRaises(SystemExit):
                parse_ws(bad)

    def test_record_and_analyze(self):
        arm, robot = make_arm('right')
        clock = Clock()
        bridge = Bridge(None, clock=clock, log=lambda s: None)
        cfg = bridge.cfg
        cfg['arm']['workspace']['right']['z'] = [-0.30, 0.30]
        box_z_lo = -0.30
        runner = types.SimpleNamespace(lock=threading.Lock(), bridge=bridge)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'rec.csv')
            rec = Recorder(path, cfg, 'right', zero=arm.zero, rate=100, log=lambda s: None,
                           extra_meta={'home': True})
            rec.set_joint_limits(FR3_LOWER, FR3_UPPER)
            arm.recorder = rec
            seq = 0
            bridge.on_arm_feedback('right', [0.0] * 6)
            bridge.on_arm_feedback('left', [0.0] * 6)
            # 10 s：前 4 s z 从 0 往下走到 -0.30 后贴边 2 s；后面 yaw 转到 2.8 rad，关节 7 顶到上限，实际跟不上
            for i in range(1000):
                clock.t += 0.01
                tt = i * 0.01
                z = max(-0.08 * tt, box_z_lo) if tt < 6 else box_z_lo + 0.05
                yaw = 0.0 if tt < 6 else min((tt - 6) * 1.0, 2.8)
                seq += 1
                bridge.handle_packet(ctrl(seq, [0.0, 0.0, z, 0.0, 0.0, yaw]), ('127.0.0.1', 1))
                bridge.tick()
                cmd = bridge.arms['right'].cmd
                qv = READY_Q.copy()
                qv[6] = min(READY_Q[6] + cmd[5], FR3_UPPER[6] - 0.02)   # 关节 7 跟着 yaw 转，到上限前停住
                act = list(cmd[:6])
                act[5] = qv[6] - READY_Q[6]
                robot.ee = arm.zero.to_base(act)
                robot.state = types.SimpleNamespace(q=list(qv), O_F_ext_hat_K=[0.0] * 6)
                arm._record(runner, clock.t)   # 真机跟踪循环里调用的同一个函数
            rec.close()
            out = subprocess.run([sys.executable, '-B', os.path.join(HERE, '..', 'tools', 'analyze_workspace.py'), path],
                                 capture_output=True, text=True, check=True).stdout
        self.assertIn('| z | -0.30 ~ +0.30', out)
        z_row = [l for l in out.splitlines() if l.startswith('| z |')][0]
        self.assertIn('1 次', z_row)                # z 下面顶到一次
        self.assertIn('z-：顶到 1 次', out)
        self.assertIn('关节到限位', out)            # yaw 那段跟不上的原因
        self.assertIn('关节 7 余', out)
        self.assertIn('z: [-0.35, 0.30]', out)      # 建议往下放 5 cm


if __name__ == '__main__':
    unittest.main()
