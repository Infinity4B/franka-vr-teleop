## 单臂离合解算：扳机按住期间手柄的位移和转动映射成末端位姿，松开冻结。
##
## 参考系：对齐时记录头显 yaw，得到只含 yaw 的 R_ref，之后固定（转头不影响手臂）。
## 轴向 M：x前 = −z_godot，y左 = −x_godot，z上 = y_godot。C = M·R_ref⁻¹。
## 位置：P = P0 + k·C·(p_t − p_0)                （等价于逐帧累加 k·C·Δp，但没有累积误差）
## 姿态：Q = (C·Δq·C⁻¹)·Q0，Δq = q_t·q_0⁻¹       （世界系增量，换到机器人系后左乘）
## 输出 rpy 按 ZYX（R = Rz(yaw)·Ry(pitch)·Rx(roll)），M4 用 follow*_pos_back 回读核对。
## p_0 / q_0 / P0 / Q0 是按下扳机（或重新对齐）那一刻的锚点。
extends RefCounted

const M := Basis(Vector3(0, -1, 0), Vector3(0, 0, 1), Vector3(-1, 0, 0))

var P := Vector3.ZERO
var Q := Quaternion.IDENTITY
var following := false
## 位置范围（机器人系，来自适配器 HELLO）。碰到边界后锚点跟着移，往回动立即生效，不会积出“死区”
var box_lo := Vector3(-INF, -INF, -INF)
var box_hi := Vector3(INF, INF, INF)
var clamped := false

var _c := M
var _cq := Quaternion(M)
var _anchor_p := Vector3.ZERO
var _anchor_q := Quaternion.IDENTITY
var _p0 := Vector3.ZERO
var _q0 := Quaternion.IDENTITY
var _last_p := Vector3.ZERO
var _last_q := Quaternion.IDENTITY
var _pos_scale := 1.0
var _rot_scale := 1.0

## 每次松开扳机记录一笔：手柄实际移动 / 转动与指令变化，用于核对精度（main.gd 打印到 logcat）
var strokes := 0
var last_stroke := {}
var _stroke_p := Vector3.ZERO
var _stroke_q := Quaternion.IDENTITY
var _stroke_P := Vector3.ZERO
var _stroke_Q := Quaternion.IDENTITY


## r_ref：只含 yaw 的参考朝向（XROrigin 空间）。
func set_reference(r_ref: Basis) -> void:
	_c = M * r_ref.inverse()
	_cq = Quaternion(_c.orthonormalized())
	if following:
		_reanchor()


## pose：手柄 grip 位姿（XROrigin 空间）。follow 为假或丢失跟踪时冻结。
func update(pose: Transform3D, tracked: bool, follow: bool, pos_scale: float, rot_enabled: bool,
		rot_scale: float) -> void:
	if not follow or not tracked:
		if following:
			_end_stroke()
		following = false
		return
	_last_p = pose.origin
	_last_q = pose.basis.get_rotation_quaternion()
	if not following:
		_stroke_p = _last_p
		_stroke_q = _last_q
		_stroke_P = P
		_stroke_Q = Q
	if not following or pos_scale != _pos_scale or rot_scale != _rot_scale:
		_reanchor()  # 缩放改变时也重新起算，否则整段位移按新倍数重算会跳变
		_pos_scale = pos_scale
		_rot_scale = rot_scale
		following = true
		return
	P = _p0 + _c * (_last_p - _anchor_p) * pos_scale
	var pc := P.clamp(box_lo, box_hi)
	clamped = pc != P
	if clamped:
		_p0 += pc - P
		P = pc
	if rot_enabled:
		var d := _last_q * _anchor_q.inverse()
		if d.w < 0:
			d = -d
		if not is_equal_approx(rot_scale, 1.0):
			d = Quaternion.IDENTITY.slerp(d, rot_scale)
		Q = (_cq * d * _cq.inverse() * _q0).normalized()
	else:
		_anchor_q = _last_q
		_q0 = Q


## 复位到零点；正在跟随时以当前手柄位姿为新锚点，不会跳回去。
func reset() -> void:
	P = Vector3.ZERO
	Q = Quaternion.IDENTITY
	if following:
		_reanchor()


## 跟踪空间变化（重新对齐、recenter）后调用：下一帧从当前手柄位姿重新起算。
func break_follow() -> void:
	if following:
		_end_stroke()
	following = false


func _end_stroke() -> void:
	strokes += 1
	last_stroke = {
		"raw_m": (_last_p - _stroke_p).length(),
		"raw_deg": _angle_deg(_last_q * _stroke_q.inverse()),
		"cmd": P - _stroke_P,
		"cmd_deg": _angle_deg(Q * _stroke_Q.inverse()),
	}


static func _angle_deg(q: Quaternion) -> float:
	var a := q.normalized().get_angle()
	return rad_to_deg(TAU - a if a > PI else a)


func _reanchor() -> void:
	_anchor_p = _last_p
	_anchor_q = _last_q
	_p0 = P
	_q0 = Q


## 机器人系 RPY（ZYX），弧度。
func rpy() -> Vector3:
	var w := Q.w
	var x := Q.x
	var y := Q.y
	var z := Q.z
	var roll := atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
	var pitch := asin(clampf(2.0 * (w * y - z * x), -1.0, 1.0))
	var yaw := atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
	return Vector3(roll, pitch, yaw)


## 由头显位姿得到只含 yaw 的参考朝向。取姿态绕竖直轴的分量（swing-twist），
## 头显低头挂在胸前时也准；只有接近倒置时才退回前方向量的水平投影。
static func yaw_reference(head: Transform3D) -> Basis:
	var q := head.basis.get_rotation_quaternion()
	if q.w * q.w + q.y * q.y > 1e-4:
		return Basis(Vector3.UP, 2.0 * atan2(q.y, q.w))
	return yaw_from_direction(-head.basis.z)


## 水平方向 f（XROrigin 空间）作为“前方”时的参考朝向。
static func yaw_from_direction(f: Vector3) -> Basis:
	return Basis(Vector3.UP, atan2(-f.x, -f.z))
