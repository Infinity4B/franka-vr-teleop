# -- coding: UTF-8
"""VR ⇄ 机器人 UDP 协议 v1（小端）。字段说明见 docs/协议.md，VR 端实现在 vr-app/scripts/protocol.gd。

所有包共用 8 字节包头：magic "AXVR"、version、type、2 字节保留。
发现、控制、状态走同一个 UDP 端口（默认 47800）。
"""
import struct

MAGIC = b'AXVR'
VERSION = 1
DEFAULT_PORT = 47800

T_DISCOVER = 1
T_HELLO = 2
T_CTRL = 3
T_STAT = 4

HEADER = struct.Struct('<4sBBH')
DISCOVER = struct.Struct('<I16s')
HELLO = struct.Struct('<IHBB32s16s4f')
HELLO_WS = struct.Struct('<24f')  # 追加在 HELLO 后：左右臂各 x y z roll pitch yaw 的 [下限, 上限]
CTRL = struct.Struct('<IIQHBB7f7f4f')
STAT = struct.Struct('<IIQIfHBBfff')

# CTRL.flags
F_UNLOCKED = 1 << 0      # 手持解锁（A+B+X+Y）
F_GATE = 1 << 1          # 总闸门（双摇杆按下）
F_LEFT_FOLLOW = 1 << 2   # 左扳机按住
F_RIGHT_FOLLOW = 1 << 3
F_LEFT_RESET = 1 << 4    # 左臂复位中（X 或首次开闸自动归零）
F_RIGHT_RESET = 1 << 5
F_PAUSED = 1 << 6        # 面板打开 / 失去焦点 / 摘下头显
F_SMOOTH = 1 << 7        # 适配器在两包之间插值

# STAT.status
S_LEFT_FB = 1 << 0       # follow1_pos_back 在线
S_RIGHT_FB = 1 << 1      # follow2_pos_back 在线
S_BODY_FB = 1 << 2       # body_information 在线
S_LEFT_LIMITED = 1 << 3  # 左臂指令被工作空间钳位或速度限幅
S_RIGHT_LIMITED = 1 << 4
S_PUBLISHING = 1 << 5    # 正在发布 /ARX_VR_L /ARX_VR_R
S_DRY_RUN = 1 << 6

# STAT.watchdog
WD_IDLE = 0      # 未发布（未解锁 / 闸门关 / 从未连接）
WD_ACTIVE = 1    # 正常跟随
WD_HOLD = 2      # 保持：手臂停在当前指令、底盘停车（暂停或超过软超时）
WD_STOPPED = 3   # 超过硬超时，已补发停车帧并停止发布

# HELLO / STAT.owner
OWNER_FREE = 0
OWNER_YOU = 1
OWNER_OTHER = 2


class ProtocolError(ValueError):
    pass


def _cstr(raw):
    return raw.split(b'\0', 1)[0].decode('utf-8', 'replace')


def _pack(ptype, body):
    return HEADER.pack(MAGIC, VERSION, ptype, 0) + body


def parse_header(data):
    if len(data) < HEADER.size:
        raise ProtocolError('包太短')
    magic, version, ptype, _ = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ProtocolError('magic 不符')
    if version != VERSION:
        raise ProtocolError(f'协议版本 {version} 不支持（需要 {VERSION}）')
    return ptype


def pack_discover(session, name):
    return _pack(T_DISCOVER, DISCOVER.pack(session, name.encode()[:16]))


def unpack_discover(data):
    _need(data, DISCOVER)
    session, name = DISCOVER.unpack_from(data, HEADER.size)
    return {'session': session, 'name': _cstr(name)}


def pack_hello(session, port, ros_version, owner, robot_name, bridge_version, limits, workspace=None):
    body = HELLO.pack(session, port, ros_version, owner, robot_name.encode()[:32], bridge_version.encode()[:16],
                      *limits)
    if workspace is not None:
        body += HELLO_WS.pack(*workspace)
    return _pack(T_HELLO, body)


def unpack_hello(data):
    _need(data, HELLO)
    v = HELLO.unpack_from(data, HEADER.size)
    d = {'session': v[0], 'port': v[1], 'ros_version': v[2], 'owner': v[3],
         'robot_name': _cstr(v[4]), 'bridge_version': _cstr(v[5]), 'limits': list(v[6:10]), 'workspace': None}
    off = HEADER.size + HELLO.size
    if len(data) >= off + HELLO_WS.size:
        d['workspace'] = list(HELLO_WS.unpack_from(data, off))
    return d


def pack_ctrl(c):
    return _pack(T_CTRL, CTRL.pack(c['session'], c['seq'], c['t_vr_us'], c['flags'], c['mode1'], 0,
                                   *c['left'], *c['right'], c['chx'], c['chy'], c['chz'], c['height']))


def unpack_ctrl(data):
    _need(data, CTRL)
    v = CTRL.unpack_from(data, HEADER.size)
    return {'session': v[0], 'seq': v[1], 't_vr_us': v[2], 'flags': v[3], 'mode1': v[4],
            'left': list(v[6:13]), 'right': list(v[13:20]),
            'chx': v[20], 'chy': v[21], 'chz': v[22], 'height': v[23]}


def pack_stat(s):
    return _pack(T_STAT, STAT.pack(s['session'], s['echo_seq'], s['echo_t_vr_us'], s['hold_us'], s['loss'],
                                   s['status'], s['watchdog'], s['owner'], s['height_fb'], s['height_cmd'],
                                   s['rx_rate']))


def unpack_stat(data):
    _need(data, STAT)
    v = STAT.unpack_from(data, HEADER.size)
    keys = ('session', 'echo_seq', 'echo_t_vr_us', 'hold_us', 'loss', 'status', 'watchdog', 'owner',
            'height_fb', 'height_cmd', 'rx_rate')
    return dict(zip(keys, v))


def _need(data, st):
    if len(data) < HEADER.size + st.size:
        raise ProtocolError(f'包长 {len(data)}，至少需要 {HEADER.size + st.size}')
