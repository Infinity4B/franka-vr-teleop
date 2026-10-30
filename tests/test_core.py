# -- coding: UTF-8
"""python3 -m unittest discover -s tests（在仓库根目录下）。取自 arx-vr-analysis，只改了包名。"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from franka_vr_bridge import protocol as P  # noqa: E402
from franka_vr_bridge.core import Bridge, interp_pose, quat_to_rpy, rpy_to_quat, step_pose  # noqa: E402

VR = ('10.0.0.2', 5000)
OTHER = ('10.0.0.3', 5000)


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Harness:
    def __init__(self, **cfg):
        self.clock = Clock()
        self.sent = []
        self.logs = []
        self.b = Bridge(cfg, sendto=lambda d, a: self.sent.append((d, a)), ros_version=2, clock=self.clock,
                        log=self.logs.append)
        self.seq = 0

    def ctrl(self, flags, left=None, right=None, session=1, addr=VR, mode1=2, ch=(0, 0, 0), height=0.0, seq=None):
        if seq is None:
            self.seq += 1
            seq = self.seq
        self.b.handle_packet(P.pack_ctrl({
            'session': session, 'seq': seq, 't_vr_us': int(self.clock.t * 1e6), 'flags': flags, 'mode1': mode1,
            'left': left or [0.0] * 6 + [5.0], 'right': right or [0.0] * 6 + [5.0],
            'chx': ch[0], 'chy': ch[1], 'chz': ch[2], 'height': height}), addr)

    def run(self, seconds, flags=None, **kw):
        """每 1/200 s tick 一次，flags 不为 None 时每 1/90 s 发一包。返回最后一次输出。"""
        out = None
        n = int(round(seconds * 200))
        next_pkt = self.clock.t
        for _ in range(n):
            if flags is not None and self.clock.t >= next_pkt:
                self.ctrl(flags, **kw)
                next_pkt += 1 / 90
            out = self.b.tick()
            self.clock.t += 1 / 200
        return out


ON = P.F_UNLOCKED | P.F_GATE


class TestMath(unittest.TestCase):
    def test_rpy_roundtrip(self):
        for rpy in [(0.1, -0.4, 2.0), (-3.0, 1.2, -1.0), (0, 0, 0)]:
            back = quat_to_rpy(rpy_to_quat(*rpy))
            for a, b in zip(rpy, back):
                self.assertAlmostEqual(a, b, places=9)

    def test_step_pose_limits(self):
        cur = [0.0] * 6 + [0.0]
        tgt = [1.0, 0, 0, 0, 0, 1.0, 5.0]
        out, limited = step_pose(cur, tgt, 0.01, 0.02, 0.1)
        self.assertTrue(limited)
        self.assertAlmostEqual(out[0], 0.01)
        self.assertAlmostEqual(out[5], 0.02, places=6)
        self.assertAlmostEqual(out[6], 0.1)
        out, limited = step_pose(tgt, tgt, 0.01, 0.02, 0.1)
        self.assertFalse(limited)
        self.assertEqual(out, tgt)

    def test_interp(self):
        a = [0, 0, 0, 0, 0, 0, 0]
        b = [1, 0, 0, 0, 0, 1.0, 4]
        mid = interp_pose(a, b, 0.5)
        self.assertAlmostEqual(mid[0], 0.5)
        self.assertAlmostEqual(mid[5], 0.5, places=6)
        self.assertAlmostEqual(mid[6], 2)


class TestBridge(unittest.TestCase):
    def test_not_publishing_until_unlocked_and_gate(self):
        h = Harness()
        self.assertIsNone(h.run(0.5, flags=0))
        self.assertIsNone(h.run(0.5, flags=P.F_UNLOCKED))
        self.assertIsNotNone(h.run(0.1, flags=ON))
        self.assertEqual(h.b.watchdog, P.WD_ACTIVE)

    def test_start_from_feedback_then_reset_slowly(self):
        h = Harness()
        h.b.on_arm_feedback('left', (0.2, 0.0, 0.1, 0.0, 0.0, 0.0))
        h.b.on_arm_feedback('right', (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        out = h.run(0.01, flags=ON | P.F_LEFT_RESET | P.F_RIGHT_RESET)
        left, _ = out
        self.assertGreater(left['x'], 0.19)  # 从反馈起步，没有跳到 0
        out = h.run(0.2, flags=ON)
        speed = (0.2 - out[0]['x']) / 0.21
        self.assertLess(speed, 0.25 * 1.01 * math.sqrt(1 + 0.25))  # 复位速度上限
        out = h.run(2.0, flags=ON)
        self.assertAlmostEqual(out[0]['x'], 0.0, places=6)
        self.assertFalse(h.b.arms['left'].resetting)

    def test_soft_timeout_holds_arm_and_stops_chassis(self):
        h = Harness()
        tgt = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0]
        h.run(1.0, flags=ON, left=tgt, mode1=1, ch=(0.5, 0.2, -0.3))
        out = h.run(0.01)
        self.assertEqual(out[0]['mode1'], 1)
        self.assertAlmostEqual(out[0]['chx'], 0.5)
        out = h.run(0.15)
        self.assertEqual(h.b.watchdog, P.WD_HOLD)
        self.assertEqual(out[0]['mode1'], 2)
        self.assertEqual((out[0]['chx'], out[0]['chy'], out[0]['chz']), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(out[0]['x'], 0.1, places=6)  # 不发全 0
        self.assertAlmostEqual(out[0]['gripper'], 3.0, places=6)

    def test_chassis_stops_within_150ms(self):
        h = Harness()
        h.run(0.5, flags=ON, mode1=1, ch=(1.0, 0, 0))
        stopped_at = None
        for i in range(60):
            out = h.b.tick()
            h.clock.t += 1 / 200
            if out and out[0]['chx'] == 0.0 and stopped_at is None:
                stopped_at = (i + 1) / 200
        self.assertIsNotNone(stopped_at)
        self.assertLessEqual(stopped_at, 0.15)

    def test_hard_timeout_sends_stop_frames_then_silent(self):
        h = Harness()
        h.run(0.5, flags=ON, left=[0.05, 0, 0, 0, 0, 0, 2.0], mode1=1, ch=(0.5, 0, 0))
        outs = []
        for _ in range(int(1.5 * 200)):
            outs.append(h.b.tick())
            h.clock.t += 1 / 200
        published = [o for o in outs if o]
        tail = outs[-50:]
        self.assertTrue(all(o is None for o in tail))
        stop_frames = [o for o in published[-10:]]
        for left, _ in stop_frames:
            self.assertEqual(left['mode1'], 2)
            self.assertEqual(left['chx'], 0.0)
            self.assertAlmostEqual(left['x'], 0.05, places=6)
        self.assertEqual(h.b.watchdog, P.WD_STOPPED)

    def test_gate_off_stops_publishing_and_resumes(self):
        h = Harness()
        h.run(0.5, flags=ON)
        out = h.run(0.2, flags=P.F_UNLOCKED)
        self.assertIsNone(out)
        self.assertEqual(h.b.watchdog, P.WD_IDLE)
        self.assertIsNotNone(h.run(0.05, flags=ON))

    def test_pause_holds(self):
        h = Harness()
        h.run(0.5, flags=ON, mode1=1, ch=(0.5, 0, 0))
        out = h.run(0.05, flags=ON | P.F_PAUSED, mode1=1, ch=(0.5, 0, 0))
        self.assertEqual(h.b.watchdog, P.WD_HOLD)
        self.assertEqual(out[0]['chx'], 0.0)
        self.assertEqual(out[0]['mode1'], 2)

    def test_single_owner(self):
        h = Harness()
        h.run(0.2, flags=ON)
        h.sent.clear()
        h.ctrl(ON, session=2, addr=OTHER, left=[0.3, 0, 0, 0, 0, 0, 0])
        self.assertEqual(h.b.arms['left'].target[0], 0.0)
        rejected = [P.unpack_stat(d) for d, a in h.sent if a == OTHER]
        self.assertEqual(rejected[0]['owner'], P.OWNER_OTHER)
        h.run(2.5)  # 原控制者超时释放
        h.ctrl(ON, session=2, addr=OTHER, left=[0.3, 0, 0, 0, 0, 0, 0], seq=1)
        self.assertEqual(h.b.link.session, 2)
        self.assertAlmostEqual(h.b.arms['left'].target[0], 0.3)

    def test_stale_packets_dropped(self):
        h = Harness()
        h.ctrl(ON, left=[0.1, 0, 0, 0, 0, 0, 5], seq=10)
        h.ctrl(ON, left=[0.2, 0, 0, 0, 0, 0, 5], seq=9)
        self.assertAlmostEqual(h.b.arms['left'].target[0], 0.1)

    def test_workspace_clamp_and_nan(self):
        h = Harness()
        h.ctrl(ON, left=[5.0, 0, 0, 0, 0, 0, 9.0])
        a = h.b.arms['left']
        self.assertAlmostEqual(a.target[0], 0.65)
        self.assertEqual(a.target[6], 5.0)
        self.assertTrue(a.limited)
        h.ctrl(ON, left=[float('nan'), 0, 0, 0, 0, 0, 1.0])
        self.assertAlmostEqual(a.target[0], 0.65)

    def test_discover_hello(self):
        h = Harness()
        h.b.handle_packet(P.pack_discover(7, 'quest'), VR)
        d, a = h.sent[-1]
        hello = P.unpack_hello(d)
        self.assertEqual(hello['session'], 7)
        self.assertEqual(hello['owner'], P.OWNER_FREE)
        self.assertEqual(hello['ros_version'], 2)
        self.assertEqual(len(hello['workspace']), 24)
        self.assertAlmostEqual(hello['workspace'][3], 0.55, places=5)  # 左臂 y 上限
        old = P.pack_hello(7, 47800, 2, 0, 'x', '1', (1, 4, 0, 20))  # 旧版不带工作空间
        self.assertIsNone(P.unpack_hello(old)['workspace'])

    def test_stat_rtt_echo(self):
        h = Harness()
        h.run(0.2, flags=ON)
        stats = [P.unpack_stat(d) for d, a in h.sent if P.parse_header(d) == P.T_STAT]
        self.assertGreater(len(stats), 3)
        s = stats[-1]
        self.assertEqual(s['owner'], P.OWNER_YOU)
        self.assertGreaterEqual(s['echo_seq'], h.seq - 3)  # STAT 30 Hz，回显最近收到的一包
        self.assertTrue(s['status'] & P.S_PUBLISHING)

    def test_height_starts_from_feedback(self):
        h = Harness()
        h.b.on_body_feedback(10.0)
        out = h.run(0.05, flags=ON, height=0.0)
        self.assertGreater(out[0]['height'], 9.0)

    def test_time_count_increments(self):
        h = Harness()
        h.run(0.05, flags=ON)
        a = h.b.tick()
        b = h.b.tick()
        self.assertEqual(b[0]['time_count'], a[0]['time_count'] + 1)
        self.assertEqual(b[1]['time_count'], b[0]['time_count'])

    def test_resume_without_feedback_starts_from_last_cmd(self):
        h = Harness()
        h.run(0.5, flags=ON, left=[0.1, 0, 0, 0, 0, 0, 5])
        h.run(1.5)  # 硬超时停发
        self.assertFalse(h.b.publishing)
        out = h.run(0.01, flags=ON, left=[0.3, 0, 0, 0, 0, 0, 5])
        self.assertLess(out[0]['x'], 0.1 + 2 * 1.0 / 200 + 1e-6)  # 2 个 tick，每 tick 最多 5 mm

    def test_same_session_survives_owner_timeout(self):
        h = Harness()
        h.run(0.2, flags=ON)
        link = h.b.link
        h.run(2.5)
        h.ctrl(ON)
        self.assertIs(h.b.link, link)
        self.assertFalse(any('控制权' in s for s in h.logs[1:]))

    def test_shutdown_frames(self):
        h = Harness()
        h.run(0.2, flags=ON, mode1=1, ch=(0.4, 0, 0))
        left, _ = h.b.shutdown_frames()
        self.assertEqual(left['mode1'], 2)
        self.assertEqual(left['chx'], 0.0)


if __name__ == '__main__':
    unittest.main()
