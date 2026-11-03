## 遥操状态机（只用右手柄）：输入 → 遥操开关 / 离合 / 复位 / 夹爪 → CTRL 字段。
##
##   长按 B 1 s：遥操开 / 关（每次切换都要重新按）；第一次开启时自动回零点
##   食指扳机：按住跟随，松开冻结（离合）；开启遥操、暂停恢复后要先松开再按
##   握持键：夹爪。默认按一下闭合、再按一下张开（gripper_toggle）；关掉后按跟握持深度连续变化
##   A：回零点
## 暂停（面板打开、失去焦点、未对齐）：手臂、夹爪冻结。
## 协议沿用 ARX 的双臂格式：CTRL 的左臂恒为零点、夹爪张开，底盘 / 升降恒为 0。
extends RefCounted

const ArmSolver := preload("res://scripts/arm_solver.gd")
const Proto := preload("res://scripts/protocol.gd")

enum ArmState { STAY, OPERATE, RESET }

const TRIGGER_ON := 0.6
const TRIGGER_OFF := 0.4
const RESET_HOLD := 0.3   # 复位标志至少保持这么久，防止丢包
const TOGGLE_HOLD := 1.0  # 长按 B 多久切换遥操开关
const GRIP_ON := 0.7      # 握持键按过它算一次按下，松到 GRIP_OFF 以下才算松开
const GRIP_OFF := 0.3
const IDLE_LEFT := [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0]

signal enabled_changed(on: bool)
signal arm_state_changed(state: int)

var settings  # settings.gd
var enabled := false
var ever_enabled := false
var paused := false
var solver := ArmSolver.new()
var arm_state: int = ArmState.STAY
var gripper := 5.0

var _b_down_at := -1.0
var _b_fired := false
var _trigger_on := false
var _need_release := true
var _reset_until := -1.0
var _grip_down := false
var _grip_closed := false


func _init(p_settings) -> void:
	settings = p_settings


func engaged() -> bool:
	return enabled and not paused


## B 按住的进度 0~1（HUD 显示用），没按时为 0。
func toggle_progress(now: float) -> float:
	if _b_down_at < 0 or _b_fired:
		return 0.0
	return clampf((now - _b_down_at) / TOGGLE_HOLD, 0.0, 1.0)


## ws：HELLO 里的 24 个值（左右臂各 x y z roll pitch yaw 的 [下限, 上限]），只用右臂的位置部分。
## 范围在发布坐标系里，解算器在标定符号之前，所以按 pos_sign 翻转。
func set_workspace(ws: Array) -> void:
	if ws.size() != 24:
		return
	var ps: Vector3 = settings.get_value("calib", "pos_sign")
	var lo := Vector3(ws[12], ws[14], ws[16])
	var hi := Vector3(ws[13], ws[15], ws[17])
	for k in 3:
		if ps[k] < 0:
			solver.box_lo[k] = -hi[k]
			solver.box_hi[k] = -lo[k]
		else:
			solver.box_lo[k] = lo[k]
			solver.box_hi[k] = hi[k]


func set_reference(r_ref: Basis) -> void:
	solver.set_reference(r_ref)


## 跟踪空间变化（recenter、重新对齐）后调用。
func break_follow() -> void:
	solver.break_follow()
	_need_release = true


## h：右手 pose tracked trigger grip ax by
func step(h: Dictionary, now: float) -> Dictionary:
	if h.by:
		if _b_down_at < 0:
			_b_down_at = now
			_b_fired = false
		elif not _b_fired and now - _b_down_at >= TOGGLE_HOLD:
			_b_fired = true
			enabled = not enabled
			_need_release = true
			if enabled and not ever_enabled:
				ever_enabled = true
				_do_reset(now)
			enabled_changed.emit(enabled)
	else:
		_b_down_at = -1.0

	var flags := 0
	if enabled:
		flags |= Proto.F_UNLOCKED | Proto.F_GATE
	if paused:
		flags |= Proto.F_PAUSED
	if settings.get_value("control", "smooth"):
		flags |= Proto.F_SMOOTH

	var trig := _trigger_on
	trig = h.trigger > TRIGGER_OFF if trig else h.trigger > TRIGGER_ON
	_trigger_on = trig
	if not trig:
		_need_release = false
	if paused:
		_need_release = true
	# 握持键边沿一直跟踪：暂停、关闭遥操期间按下又松开不算，恢复时还按着也不算新的一下
	var grip_was_down := _grip_down
	_grip_down = h.grip > GRIP_OFF if _grip_down else h.grip > GRIP_ON
	var grip_pressed := _grip_down and not grip_was_down

	var state: int = ArmState.STAY
	var follow := false
	if engaged():
		if h.ax:
			_do_reset(now)
		if now < _reset_until:
			state = ArmState.RESET
		elif trig and h.tracked and not _need_release:
			follow = true
			state = ArmState.OPERATE
		if settings.get_value("control", "gripper_toggle"):
			if grip_pressed:
				_grip_closed = not _grip_closed
			gripper = _gripper_value(1.0 if _grip_closed else 0.0)
		else:
			gripper = _gripper_value(h.grip)
	solver.update(h.pose, h.tracked, follow, settings.get_value("control", "pos_scale"),
			settings.get_value("control", "rot_enabled"), settings.get_value("control", "rot_scale"))
	if state != arm_state:
		arm_state = state
		arm_state_changed.emit(state)
	if solver.following:
		flags |= Proto.F_RIGHT_FOLLOW
	if now < _reset_until:
		flags |= Proto.F_RIGHT_RESET

	return {
		"flags": flags,
		"mode1": 2,
		"left": PackedFloat32Array(IDLE_LEFT),
		"right": _arm_fields(),
		"chx": 0.0,
		"chy": 0.0,
		"chz": 0.0,
		"height": 0.0,
	}


func _do_reset(now: float) -> void:
	solver.reset()
	_reset_until = now + RESET_HOLD


func _gripper_value(grip: float) -> float:
	var open_v: float = settings.get_value("control", "gripper_open")
	var close_v: float = settings.get_value("control", "gripper_close")
	if settings.get_value("control", "gripper_invert"):
		var t := open_v
		open_v = close_v
		close_v = t
	return lerpf(open_v, close_v, clampf(grip, 0.0, 1.0))


func _arm_fields() -> PackedFloat32Array:
	var ps: Vector3 = settings.get_value("calib", "pos_sign")
	var rs: Vector3 = settings.get_value("calib", "rot_sign")
	var p: Vector3 = solver.P * ps
	var e: Vector3 = solver.rpy() * rs
	return PackedFloat32Array([p.x, p.y, p.z, e.x, e.y, e.z, gripper])
