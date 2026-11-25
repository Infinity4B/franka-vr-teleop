#!/usr/bin/env python3
# -- coding: UTF-8
"""分析 --record 录下的工作空间数据，回答“方框该开多大”。

  uv run --frozen python tools/analyze_workspace.py recordings/ws_20261125_153000.csv [--margin 0.05]

看四件事：
  1. 方框每一面顶到了几次、多久（头显目标贴在边界上 = 手还想往外走被拦住）；
  2. 实际用到的范围（相对零点和基座系），离方框还有多少余量；
  3. 跟不上的时段（指令和实际偏差大）：那一刻哪个关节离限位最近、外力多大——这种“限位”放宽方框也没用；
  4. 每个关节离限位最近到多少度。
最后给出建议的 workspace，可以直接抄进 config.yaml。
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

AXES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
POS = ('x', 'y', 'z')
FACE_TOL = 1e-4
FR3_LOWER = [-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508]
FR3_UPPER = [2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508]
NEAR_JOINT_DEG = 10.0      # 离关节限位小于它算“到限位附近”
LAG_POS = 0.02             # m，指令与实际偏差超过它算跟不上
LAG_ROT = math.radians(8)  # rad
MIN_EPISODE = 0.1          # s，短于它的贴边 / 跟不上不计


def load(path):
    with open(path, newline='') as f:
        r = csv.reader(f)
        head = next(r)
        rows = [[float(v) if v != '' else math.nan for v in row] for row in r if row]
    if not rows:
        sys.exit(f'{path} 里没有采样')
    a = np.array(rows, dtype=float)
    d = {k: a[:, i] for i, k in enumerate(head)}
    meta_path = os.path.splitext(path)[0] + '.json'
    meta = {}
    if os.path.exists(meta_path):
        with open(meta_path, encoding='utf-8') as f:
            meta = json.load(f)
    return d, meta


def episodes(mask, t, min_len=MIN_EPISODE, merge=0.2):
    """mask 为真的连续段 [(起, 止)]，相隔小于 merge 的合并，短于 min_len 的丢掉。"""
    segs = []
    start = None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            segs.append([t[start], t[i - 1]])
            start = None
    if start is not None:
        segs.append([t[start], t[-1]])
    merged = []
    for s in segs:
        if merged and s[0] - merged[-1][1] < merge:
            merged[-1][1] = s[1]
        else:
            merged.append(s)
    return [s for s in merged if s[1] - s[0] >= min_len]


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot_angle(a, b):
    """两组 rpy 之间的转角（rad）。"""
    m = rpy_to_matrix(*a).T @ rpy_to_matrix(*b)
    return math.acos(max(-1.0, min(1.0, (np.trace(m) - 1) / 2)))


def pct(v, q):
    v = v[np.isfinite(v)]
    return float(np.percentile(v, q)) if v.size else math.nan


def fmt(v, nd=3, sign=True):
    if v is None or not math.isfinite(v):
        return '—'
    return f'{v:+.{nd}f}' if sign else f'{v:.{nd}f}'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv')
    ap.add_argument('--margin', type=float, default=0.05, help='顶到的面建议往外放多少 m（默认 0.05）')
    args = ap.parse_args()
    d, meta = load(args.csv)
    t = d['t']
    dt = np.clip(np.diff(t, append=t[-1]), 0, 0.1)
    pub = d['publishing'] > 0.5
    fol = pub & (d['follow'] > 0.5) & (d['reset'] < 0.5) & (d['paused'] < 0.5)
    box = meta.get('workspace') or {}
    has_robot = np.isfinite(d['ax']).any()
    lower = np.array(meta.get('joint_lower') or FR3_LOWER)
    upper = np.array(meta.get('joint_upper') or FR3_UPPER)
    out = []
    p = out.append

    # ---------------------------------------------------------------- 概况
    p(f'# 工作空间分析：{os.path.basename(args.csv)}')
    p(f'录制 {meta.get("started", "?")}，手 {meta.get("arm", "?")}，'
      f'{"有" if has_robot else "没有（dry-run）"} FR3 实际位姿，'
      f'{"--home 归位后取零点" if meta.get("home") else "零点 = 启动时位置（没加 --home）"}')
    if meta.get('zero_p0'):
        z = meta['zero_p0']
        p(f'零点（基座系）：x {z[0]:.3f}  y {z[1]:+.3f}  z {z[2]:.3f} m')
    if meta.get('ws_override'):
        p('本次 --ws 覆盖：' + '  '.join(f'{k} {v[0]:+.2f}~{v[1]:+.2f}' for k, v in meta['ws_override'].items()))
    p(f'总时长 {t[-1] - t[0]:.0f} s，遥操开启 {dt[pub].sum():.0f} s，按住扳机跟随 {dt[fol].sum():.0f} s，'
      f'跟随 {len(episodes(fol, t, 0.05, 0.0))} 段')
    if dt[fol].sum() < 5:
        p('\n跟随时间太短，没法分析。长按 B 开启遥操后按住扳机移动。')
        print('\n'.join(out))
        return

    # ---------------------------------------------------------------- 方框
    p('\n## 1. 方框：每一面顶到几次（相对零点，m）')
    p('| 轴 | 方框 | 目标用到 min / max | 实际用到 1% / 99% | 下面顶到 | 上面顶到 |')
    p('|---|---|---|---|---|---|')
    face_info = {}
    for i, k in enumerate(POS):
        lo, hi = box.get(k, [math.nan, math.nan])
        tv = d['t' + k]
        at_lo = fol & (tv <= lo + FACE_TOL)
        at_hi = fol & (tv >= hi - FACE_TOL)
        e_lo, e_hi = episodes(at_lo, t), episodes(at_hi, t)
        face_info[k] = (e_lo, e_hi, at_lo, at_hi)
        a = d['a' + k][fol]
        s = lambda e, m: f'{len(e)} 次 / {dt[m].sum():.1f} s' if e else '—'
        p(f'| {k} | {lo:+.2f} ~ {hi:+.2f} | {fmt(np.nanmin(tv[fol]))} / {fmt(np.nanmax(tv[fol]))} | '
          f'{fmt(pct(a, 1))} / {fmt(pct(a, 99))} | {s(e_lo, at_lo)} | {s(e_hi, at_hi)} |')
    if has_robot:
        p('\n实际末端（基座系，m）：' + '  '.join(
            f'{k} {fmt(np.nanmin(d["b" + k][pub]))} ~ {fmt(np.nanmax(d["b" + k][pub]))}' for k in POS)
          + '（z 最低点就是这次夹爪下到离基座多高）')

    # ---------------------------------------------------------------- 姿态
    p('\n## 2. 姿态（相对零点的基座系增量）')
    tr = np.array([d['t' + k] for k in ('roll', 'pitch', 'yaw')]).T
    ang = np.array([rot_angle((0, 0, 0), r) if np.isfinite(r).all() else math.nan for r in tr[fol]])
    p(f'目标转角：中位 {math.degrees(pct(ang, 50)):.0f}°，99% {math.degrees(pct(ang, 99)):.0f}°，'
      f'最大 {math.degrees(np.nanmax(ang)):.0f}°')
    for k in ('roll', 'pitch', 'yaw'):
        lo, hi = box.get(k, [-math.pi, math.pi])
        v = d['t' + k][fol]
        clipped = np.sum((v < lo) | (v > hi))
        p(f'- {k}：{math.degrees(np.nanmin(v)):+.0f}° ~ {math.degrees(np.nanmax(v)):+.0f}°'
          + (f'，超出适配器姿态范围 {clipped} 个采样' if clipped else ''))

    # ---------------------------------------------------------------- 关节
    near_joint = np.zeros_like(t, dtype=bool)
    worst_joint = np.full(t.shape, -1)
    worst_margin = np.full(t.shape, math.nan)
    if has_robot and np.isfinite(d['q1']).any():
        q = np.array([d[f'q{i}'] for i in range(1, 8)]).T
        margin = np.degrees(np.minimum(q - lower, upper - q))   # 每个关节离较近那一端的度数
        worst_joint = np.nanargmin(np.where(np.isfinite(margin), margin, np.inf), axis=1)
        worst_margin = margin[np.arange(len(t)), worst_joint]
        near_joint = worst_margin < NEAR_JOINT_DEG
        p('\n## 3. 关节离限位多远（遥操开启期间）')
        p('| 关节 | 限位 (°) | 用到 (°) | 离下限最近 | 离上限最近 | <10° 的时间 |')
        p('|---|---|---|---|---|---|')
        for j in range(7):
            qj = q[pub, j]
            if not np.isfinite(qj).any():
                continue
            dl = math.degrees(np.nanmin(qj) - lower[j])
            du = math.degrees(upper[j] - np.nanmax(qj))
            near = pub & (margin[:, j] < NEAR_JOINT_DEG)
            flag = ' ⚠️' if min(dl, du) < NEAR_JOINT_DEG else ''
            p(f'| {j + 1}{flag} | {math.degrees(lower[j]):+.0f} ~ {math.degrees(upper[j]):+.0f} | '
              f'{math.degrees(np.nanmin(qj)):+.0f} ~ {math.degrees(np.nanmax(qj)):+.0f} | {dl:.0f}° | {du:.0f}° | '
              f'{dt[near].sum():.1f} s |')

    # ---------------------------------------------------------------- 跟不上
    lag_bad = np.zeros_like(t, dtype=bool)
    if has_robot:
        cpos = np.array([d['c' + k] for k in POS]).T
        apos = np.array([d['a' + k] for k in POS]).T
        perr = np.linalg.norm(cpos - apos, axis=1)
        rerr = np.array([rot_angle(c, a) if np.isfinite(c).all() and np.isfinite(a).all() else math.nan
                         for c, a in zip(np.array([d['c' + k] for k in ('roll', 'pitch', 'yaw')]).T,
                                         np.array([d['a' + k] for k in ('roll', 'pitch', 'yaw')]).T)])
        tpos = np.array([d['t' + k] for k in POS]).T
        speed_lag = np.linalg.norm(tpos - cpos, axis=1)
        lag_bad = pub & ((perr > LAG_POS) | (rerr > LAG_ROT))
        force = np.linalg.norm(np.array([d['fx'], d['fy'], d['fz']]).T, axis=1)
        p('\n## 4. 机器人跟不上的时段（指令与实际偏差 > 2 cm 或 > 8°）')
        p(f'位置偏差：中位 {pct(perr[pub], 50) * 100:.1f} cm，99% {pct(perr[pub], 99) * 100:.1f} cm；'
          f'姿态偏差 99% {math.degrees(pct(rerr[pub], 99)):.1f}°；'
          f'速度上限造成的滞后 > {meta.get("lag_warn", 0.02) * 100:.0f} cm 的时间 '
          f'{dt[pub & (speed_lag > meta.get("lag_warn", 0.02))].sum():.1f} s；'
          f'外力最大 {np.nanmax(force[pub]):.0f} N')
        segs = episodes(lag_bad, t, 0.2)
        if not segs:
            p('没有跟不上的时段。')
        else:
            p(f'共 {len(segs)} 段，按时长排序（前 10）：')
            p('| 时刻 (s) | 时长 | 最大偏差 | 最近的关节限位 | 外力最大 | 目标在方框边上 | 判断 |')
            p('|---|---|---|---|---|---|---|')
            for s0, s1 in sorted(segs, key=lambda s: s[0] - s[1])[:10]:
                m = (t >= s0) & (t <= s1)
                i_w = np.flatnonzero(m)[np.nanargmin(worst_margin[m])] if np.isfinite(worst_margin[m]).any() else None
                jtxt = f'关节 {worst_joint[i_w] + 1} 余 {worst_margin[i_w]:.0f}°' if i_w is not None else '—'
                faces = [f'{k}{s}' for k in POS for s, msk in (('-', face_info[k][2]), ('+', face_info[k][3]))
                         if msk[m].any()]
                fmax = np.nanmax(force[m])
                if i_w is not None and worst_margin[i_w] < NEAR_JOINT_DEG:
                    why = '关节到限位'
                elif fmax > 15:
                    why = '可能顶住了东西'
                else:
                    why = '可能是奇异 / 伸太远 / 动太快'
                p(f'| {s0:.1f} | {s1 - s0:.1f} s | {np.nanmax(perr[m]) * 100:.1f} cm / '
                  f'{math.degrees(np.nanmax(rerr[m])):.0f}° | {jtxt} | {fmax:.0f} N | {" ".join(faces) or "—"} | {why} |')

    # ---------------------------------------------------------------- 事件
    if meta.get('events'):
        p('\n## 事件')
        for te, txt in meta['events']:
            p(f'- {te:.1f} s：{txt}')

    # ---------------------------------------------------------------- 建议
    p('\n## 建议的末端范围（相对零点）')
    sug = {}
    notes = []
    for i, k in enumerate(POS):
        lo, hi = box.get(k, [math.nan, math.nan])
        e_lo, e_hi, at_lo, at_hi = face_info[k]
        a = d['a' + k][fol] if has_robot else d['t' + k][fol]
        new = [lo, hi]
        for side, e, msk, bound, used in (('-', e_lo, at_lo, lo, pct(a, 0.5)), ('+', e_hi, at_hi, hi, pct(a, 99.5))):
            name = f'{k}{side}'
            if e:
                stuck = (msk & (near_joint | lag_bad)).sum() / max(msk.sum(), 1)
                if has_robot and stuck > 0.5:
                    notes.append(f'{name}：顶到 {len(e)} 次，但其中大半时间机器人本身已到关节限位或跟不上，放宽方框也走不远，先别放宽')
                else:
                    new[0 if side == '-' else 1] = bound + (-args.margin if side == '-' else args.margin)
                    notes.append(f'{name}：顶到 {len(e)} 次、共 {dt[msk].sum():.1f} s，范围不够，建议往外放 {args.margin * 100:.0f} cm'
                                 '（放宽后再录一次，直到不再频繁顶到）')
            else:
                room = abs(bound - used) if math.isfinite(used) else math.nan
                notes.append(f'{name}：没顶到，实际最远到 {fmt(used)}，离边界还有 {room * 100:.0f} cm，保持')
        sug[k] = new
    for n in notes:
        p(f'- {n}')
    if has_robot and meta.get('zero_p0'):
        z0 = meta['zero_p0'][2]
        p(f'\n⚠️ z 下限 {sug["z"][0]:+.2f} 对应基座上方 {z0 + sug["z"][0]:.2f} m。往下放之前确认桌面高度，别让夹爪撞桌子。')
    if not meta.get('home'):
        p('⚠️ 这次没加 --home，零点不是 ready 姿态；方框是相对零点的，换零点后要换算（用上面的基座系数值）。')
    p('\n```yaml\n  workspace:\n    right:')
    for k in POS:
        p(f'      {k}: [{sug[k][0]:.2f}, {sug[k][1]:.2f}]')
    for k in ('roll', 'pitch', 'yaw'):
        lo, hi = box.get(k, [-3.1416, 3.1416])
        p(f'      {k}: [{lo:.5g}, {hi:.5g}]')
    p('```')
    print('\n'.join(out))


if __name__ == '__main__':
    main()
