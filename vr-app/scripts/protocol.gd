## VR ⇄ 机器人 UDP 协议 v1（小端）。与 franka_vr_bridge/protocol.py 一一对应。
## 取自 arx-vr-analysis，只改了 magic（AXVR → FRVR），和 ARX 的应用、适配器互不相认。
extends RefCounted

const MAGIC := "FRVR"
const VERSION := 1
const DEFAULT_PORT := 47800

const T_DISCOVER := 1
const T_HELLO := 2
const T_CTRL := 3
const T_STAT := 4

const HEADER_SIZE := 8
const HELLO_SIZE := HEADER_SIZE + 72
const HELLO_WS_SIZE := 96  # 可选：左右臂各 x y z roll pitch yaw 的 [下限, 上限]
const STAT_SIZE := HEADER_SIZE + 40

# CTRL.flags
const F_UNLOCKED := 1 << 0
const F_GATE := 1 << 1
const F_LEFT_FOLLOW := 1 << 2
const F_RIGHT_FOLLOW := 1 << 3
const F_LEFT_RESET := 1 << 4
const F_RIGHT_RESET := 1 << 5
const F_PAUSED := 1 << 6
const F_SMOOTH := 1 << 7

# STAT.status
const S_LEFT_FB := 1 << 0
const S_RIGHT_FB := 1 << 1
const S_BODY_FB := 1 << 2
const S_LEFT_LIMITED := 1 << 3
const S_RIGHT_LIMITED := 1 << 4
const S_PUBLISHING := 1 << 5
const S_DRY_RUN := 1 << 6

# STAT.watchdog
const WD_IDLE := 0
const WD_ACTIVE := 1
const WD_HOLD := 2
const WD_STOPPED := 3

const OWNER_FREE := 0
const OWNER_YOU := 1
const OWNER_OTHER := 2


static func _begin(ptype: int) -> StreamPeerBuffer:
	var b := StreamPeerBuffer.new()
	b.big_endian = false
	b.put_data(MAGIC.to_ascii_buffer())
	b.put_u8(VERSION)
	b.put_u8(ptype)
	b.put_u16(0)
	return b


static func _fixed(s: String, n: int) -> PackedByteArray:
	var raw := s.to_utf8_buffer()
	raw.resize(n)
	return raw


static func _cstr(raw: PackedByteArray) -> String:
	var end := raw.find(0)
	return (raw if end < 0 else raw.slice(0, end)).get_string_from_utf8()


static func pack_discover(session: int, device_name: String) -> PackedByteArray:
	var b := _begin(T_DISCOVER)
	b.put_u32(session)
	b.put_data(_fixed(device_name, 16))
	return b.data_array


## c: session seq t_vr_us flags mode1 left(7) right(7) chx chy chz height
static func pack_ctrl(c: Dictionary) -> PackedByteArray:
	var b := _begin(T_CTRL)
	b.put_u32(c.session)
	b.put_u32(c.seq)
	b.put_u64(c.t_vr_us)
	b.put_u16(c.flags)
	b.put_u8(c.mode1)
	b.put_u8(0)
	for v in c.left:
		b.put_float(v)
	for v in c.right:
		b.put_float(v)
	b.put_float(c.chx)
	b.put_float(c.chy)
	b.put_float(c.chz)
	b.put_float(c.height)
	return b.data_array


## 返回 {} 表示不是本协议的包。
static func parse(data: PackedByteArray) -> Dictionary:
	if data.size() < HEADER_SIZE or data.slice(0, 4) != MAGIC.to_ascii_buffer() or data[4] != VERSION:
		return {}
	var b := StreamPeerBuffer.new()
	b.big_endian = false
	b.data_array = data
	b.seek(HEADER_SIZE)
	var ptype := data[5]
	if ptype == T_HELLO and data.size() >= HELLO_SIZE:
		var d := {"type": T_HELLO}
		d.session = b.get_u32()
		d.port = b.get_u16()
		d.ros_version = b.get_u8()
		d.owner = b.get_u8()
		d.robot_name = _cstr(b.get_data(32)[1])
		d.bridge_version = _cstr(b.get_data(16)[1])
		d.limits = [b.get_float(), b.get_float(), b.get_float(), b.get_float()]
		d.workspace = []
		if data.size() >= HELLO_SIZE + HELLO_WS_SIZE:
			for i in 24:
				d.workspace.append(b.get_float())
		return d
	if ptype == T_STAT and data.size() >= STAT_SIZE:
		var s := {"type": T_STAT}
		s.session = b.get_u32()
		s.echo_seq = b.get_u32()
		s.echo_t_vr_us = b.get_u64()
		s.hold_us = b.get_u32()
		s.loss = b.get_float()
		s.status = b.get_u16()
		s.watchdog = b.get_u8()
		s.owner = b.get_u8()
		s.height_fb = b.get_float()
		s.height_cmd = b.get_float()
		s.rx_rate = b.get_float()
		return s
	return {}
