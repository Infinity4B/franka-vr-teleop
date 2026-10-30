## godot --headless --xr-mode off --path vr-app -s res://tests/run_tests.gd
## 解算精度（验收标准：20 cm → 20±0.5 cm，90° → 90±1°，静止不漂移）、右手状态机、协议与 Python 交叉校验。
## 解算部分的测试原样取自 arx-vr-analysis；状态机和协议部分是 Franka 版的。
extends SceneTree

const ArmSolver := preload("res://scripts/arm_solver.gd")
const Teleop := preload("res://scripts/teleop.gd")
const Settings := preload("res://scripts/settings.gd")
const Proto := preload("res://scripts/protocol.gd")

var failures := 0
var checks := 0


func _init() -> void:
	test_yaw_reference_pitched()
	test_align_by_direction()
	test_solver_translation()
	test_solver_rotation()
	test_solver_clutch_and_drift()
	test_solver_reset_while_following()
	test_solver_workspace_wall()
	test_teleop_workspace()
	test_teleop_toggle()
	test_teleop_follow_gripper_reset()
	test_teleop_pause_requires_release()
	test_protocol_xcheck()
	print("\n%d 项检查，%d 项失败" % [checks, failures])
	quit(1 if failures else 0)

func check(cond: bool, what: String) -> void:
	checks += 1
	if not cond:
		failures += 1
		print("  ✗ ", what)


func near(a: float, b: float, tol: float, what: String) -> void:
	check(absf(a - b) <= tol, "%s：%.5f，期望 %.5f ± %.5f" % [what, a, b, tol])


func head_facing(yaw_deg: float) -> Transform3D:
	return Transform3D(Basis(Vector3.UP, deg_to_rad(yaw_deg)), Vector3(0.3, 1.6, -0.2))


func follow_path(s, frames: Array, start_basis := Basis.IDENTITY) -> void:
	s.update(Transform3D(start_basis, frames[0]), true, true, 1.0, true, 1.0)
	for p in frames:
		s.update(Transform3D(start_basis, p), true, true, 1.0, true, 1.0)


func test_yaw_reference_pitched() -> void:
	print("对齐：头显低头挂在胸前")
	for yaw in [0.0, 40.0, -150.0]:
		for pitch in [0.0, -45.0, -80.0, -89.0]:
			for roll in [0.0, 10.0]:
				var b := Basis(Vector3.UP, deg_to_rad(yaw)) * Basis(Vector3.RIGHT, deg_to_rad(pitch)) \
						* Basis(Vector3.BACK, deg_to_rad(roll))
				var ref := ArmSolver.yaw_reference(Transform3D(b, Vector3(0, 1.3, 0)))
				var got := rad_to_deg(ref.get_euler().y)
				var tol := 0.5 if roll == 0.0 else 12.0  # 侧倾时竖直轴转角与朝向有小差别
				near(wrapf(got - yaw, -180, 180), 0.0, tol, "yaw=%d° 俯 %d° 侧倾 %d°" % [yaw, pitch, roll])


func test_align_by_direction() -> void:
	print("对齐：手柄指向")
	for yaw in [0.0, 47.0, -120.0]:
		var f := Basis(Vector3.UP, deg_to_rad(yaw)) * Vector3.FORWARD
		var s := ArmSolver.new()
		s.set_reference(ArmSolver.yaw_from_direction(f + Vector3(0, -0.3, 0)))  # 略向下指也行
		follow_path(s, [Vector3.ZERO, f * 0.2])
		near(s.P.x, 0.2, 1e-5, "指向 yaw=%d° 后沿该方向推 20 cm → x" % yaw)
		near(s.P.y, 0.0, 1e-5, "指向 yaw=%d° → y 不变" % yaw)


func test_solver_translation() -> void:
	print("解算：平移")
	for yaw in [0.0, 30.0, -120.0]:
		var head := head_facing(yaw)
		var s := ArmSolver.new()
		s.set_reference(ArmSolver.yaw_reference(head))
		var f := -head.basis.z          # 头显前方
		var left := -head.basis.x       # 头显左方
		var start := Vector3(0.1, 1.2, -0.4)
		var path := []
		for i in 101:
			path.append(start + f * 0.20 * i / 100.0)
		follow_path(s, path)
		near(s.P.x, 0.20, 0.005, "yaw=%d° 前移 20 cm → x" % yaw)
		near(s.P.y, 0.0, 1e-4, "yaw=%d° 前移 → y" % yaw)
		s.update(Transform3D(Basis.IDENTITY, path[-1]), true, false, 1.0, true, 1.0)
		var p2 := []
		for i in 51:
			p2.append(start + left * 0.1 * i / 50.0 + Vector3.UP * 0.05 * i / 50.0)
		follow_path(s, p2)
		near(s.P.y, 0.10, 1e-4, "yaw=%d° 左移 10 cm → y" % yaw)
		near(s.P.z, 0.05, 1e-4, "yaw=%d° 上移 5 cm → z" % yaw)
		near(s.P.x, 0.20, 1e-4, "yaw=%d° 离合后 x 保持" % yaw)
		s.update(Transform3D(Basis.IDENTITY, p2[-1]), true, false, 1.0, true, 1.0)
		check(s.strokes == 2, "记录两次跟随")
		near(s.last_stroke.raw_m, Vector2(0.1, 0.05).length(), 1e-5, "跟随记录：手柄移动距离")
		near(s.last_stroke.cmd.length(), s.last_stroke.raw_m, 1e-5, "跟随记录：指令变化 = 手柄移动")


func test_solver_rotation() -> void:
	print("解算：旋转")
	for yaw in [0.0, 45.0]:
		var head := head_facing(yaw)
		var f := -head.basis.z
		var left := -head.basis.x
		var cases := [[Vector3.UP, 2, "绕竖直轴 90° → yaw"], [f, 0, "绕前方轴 90° → roll"],
				[left, 1, "绕左方轴 60° → pitch"]]
		for c in cases:
			var s := ArmSolver.new()
			s.set_reference(ArmSolver.yaw_reference(head))
			var deg := 60.0 if c[1] == 1 else 90.0
			var start := Basis(Vector3(0.3, 0.2, 0.9).normalized(), 0.7)  # 任意初始手腕姿态
			s.update(Transform3D(start, Vector3.ZERO), true, true, 1.0, true, 1.0)
			for i in 61:
				var b := Basis(c[0], deg_to_rad(deg) * i / 60.0) * start
				s.update(Transform3D(b, Vector3.ZERO), true, true, 1.0, true, 1.0)
			var e: Vector3 = s.rpy()
			near(rad_to_deg(e[c[1]]), deg, 1.0, "yaw=%d° %s" % [yaw, c[2]])
			for k in 3:
				if k != c[1]:
					near(rad_to_deg(e[k]), 0.0, 0.5, "yaw=%d° %s，其他轴" % [yaw, c[2]])


func test_solver_clutch_and_drift() -> void:
	print("解算：离合与漂移")
	var s := ArmSolver.new()
	s.set_reference(Basis.IDENTITY)
	var rng := RandomNumberGenerator.new()
	rng.seed = 7
	var center := Vector3(0, 1.2, -0.3)
	s.update(Transform3D(Basis.IDENTITY, center), true, true, 1.0, true, 1.0)
	for i in 5000:  # 静止 + 0.5 mm / 0.2° 跟踪噪声
		var n := Vector3(rng.randf_range(-1, 1), rng.randf_range(-1, 1), rng.randf_range(-1, 1)) * 0.0005
		var b := Basis(Vector3(rng.randf(), rng.randf(), rng.randf()).normalized(), deg_to_rad(rng.randf_range(-0.2, 0.2)))
		s.update(Transform3D(b, center + n), true, true, 1.0, true, 1.0)
	check(s.P.length() < 0.001, "静止 5000 帧位置漂移 %.5f m < 1 mm" % s.P.length())
	check(rad_to_deg(s.Q.get_angle()) < 0.5, "静止 5000 帧姿态漂移 %.3f° < 0.5°" % rad_to_deg(s.Q.get_angle()))

	var s2 := ArmSolver.new()
	s2.set_reference(Basis.IDENTITY)
	follow_path(s2, [Vector3.ZERO, Vector3(0, 0, -0.1)])
	s2.update(Transform3D(Basis.IDENTITY, Vector3(0, 0, -0.1)), true, false, 1.0, true, 1.0)
	s2.update(Transform3D(Basis.IDENTITY, Vector3(0.5, 0.3, 0.4)), true, false, 1.0, true, 1.0)  # 松开后乱动
	follow_path(s2, [Vector3(0.5, 0.3, 0.4), Vector3(0.5, 0.3, 0.3)])
	near(s2.P.x, 0.2, 1e-5, "两次各前移 10 cm，中间松开乱动 → x = 20 cm")
	near(s2.P.y, 0.0, 1e-5, "离合 → y 不变")

	var s3 := ArmSolver.new()
	s3.set_reference(Basis.IDENTITY)
	follow_path(s3, [Vector3.ZERO, Vector3(0, 0, -0.1)])
	s3.update(Transform3D(Basis.IDENTITY, Vector3(0, 0, -0.1)), false, true, 1.0, true, 1.0)  # 丢失跟踪
	s3.update(Transform3D(Basis.IDENTITY, Vector3(0, 2, -0.1)), true, true, 1.0, true, 1.0)   # 恢复时跳变
	near(s3.P.z, 0.0, 1e-6, "丢失跟踪后恢复不跳变")

	var s4 := ArmSolver.new()
	s4.set_reference(Basis.IDENTITY)
	follow_path(s4, [Vector3.ZERO, Vector3(0, 0, -0.1)])
	s4.update(Transform3D(Basis.IDENTITY, Vector3(0, 0, -0.1)), true, true, 2.0, true, 1.0)
	s4.update(Transform3D(Basis.IDENTITY, Vector3(0, 0, -0.2)), true, true, 2.0, true, 1.0)
	near(s4.P.x, 0.3, 1e-5, "跟随中改缩放：之前 10 cm 不变，之后 10 cm × 2")


func test_solver_workspace_wall() -> void:
	print("解算：碰到工作空间边界")
	var s := ArmSolver.new()
	s.set_reference(Basis.IDENTITY)
	s.box_hi = Vector3(0.3, 0.55, 0.6)
	var path := []
	for i in 51:
		path.append(Vector3(0, 0, -0.5 * i / 50.0))  # 前推 50 cm，超出 x 上限 30 cm
	follow_path(s, path)
	near(s.P.x, 0.3, 1e-6, "超出时钳在边界")
	check(s.clamped, "钳位标志")
	s.update(Transform3D(Basis.IDENTITY, Vector3(0, 0, -0.4)), true, true, 1.0, true, 1.0)  # 往回 10 cm
	near(s.P.x, 0.2, 1e-6, "往回 10 cm 立即回到 20 cm，没有死区")
	check(not s.clamped, "离开边界后钳位标志清除")


func test_solver_reset_while_following() -> void:
	print("解算：跟随中复位")
	var s := ArmSolver.new()
	s.set_reference(Basis.IDENTITY)
	follow_path(s, [Vector3.ZERO, Vector3(0, 0, -0.1)])
	s.reset()
	check(s.P == Vector3.ZERO and s.Q.is_equal_approx(Quaternion.IDENTITY), "复位后为零点")
	s.update(Transform3D(Basis.IDENTITY, Vector3(0, 0, -0.15)), true, true, 1.0, true, 1.0)
	near(s.P.x, 0.05, 1e-5, "复位后从当前手柄位置继续，不跳回")


func hand(overrides := {}) -> Dictionary:
	var h := {"pose": Transform3D.IDENTITY, "tracked": true, "trigger": 0.0, "grip": 0.0, "ax": false, "by": false}
	for k in overrides:
		h[k] = overrides[k]
	return h


func make_teleop():
	return Teleop.new(Settings.new())


func test_teleop_workspace() -> void:
	print("状态机：工作空间（只取右臂）与标定符号")
	var st := Settings.new()
	st.set_value("calib", "pos_sign", Vector3(1, -1, 1))
	var t = Teleop.new(st)
	var ws := []
	for i in 24:
		ws.append(0.0)
	ws[2] = -9.0   # 左臂 y 下限：不该被用到
	ws[12] = -0.25  # 右臂 x
	ws[13] = 0.3
	ws[14] = -0.4  # 右臂 y
	ws[15] = 0.2
	t.set_workspace(ws)
	near(t.solver.box_lo.x, -0.25, 1e-6, "右臂 x 下限")
	near(t.solver.box_hi.x, 0.3, 1e-6, "右臂 x 上限")
	near(t.solver.box_lo.y, -0.2, 1e-6, "y 取反时下限 = −上限")
	near(t.solver.box_hi.y, 0.4, 1e-6, "y 取反时上限 = −下限")


func test_teleop_toggle() -> void:
	print("状态机：长按 B 开关遥操")
	var t = make_teleop()
	var c: Dictionary = t.step(hand({"by": true}), 0.0)
	c = t.step(hand({"by": true}), 0.5)
	check(not t.enabled, "按住 0.5 s 不切换")
	near(t.toggle_progress(0.5), 0.5, 1e-6, "进度 50%")
	check(c.flags & (Proto.F_UNLOCKED | Proto.F_GATE) == 0, "关闭时 flags 无 UNLOCKED / GATE")
	c = t.step(hand({"by": true}), 1.0)
	check(t.enabled, "按住 1 s 开启")
	check(c.flags & Proto.F_UNLOCKED and c.flags & Proto.F_GATE, "开启时 UNLOCKED + GATE")
	check(c.flags & Proto.F_RIGHT_RESET != 0, "第一次开启自动回零点")
	c = t.step(hand({"by": true}), 2.5)
	check(t.enabled, "按住不放不会反复切换")
	near(t.toggle_progress(2.5), 0.0, 1e-6, "切换后进度清零")
	t.step(hand(), 2.6)
	t.step(hand({"by": true}), 2.7)
	c = t.step(hand({"by": true}), 3.2)
	check(t.enabled, "短按 B 不关闭")
	c = t.step(hand({"by": true}), 3.75)
	check(not t.enabled, "再长按 1 s 关闭")
	check(c.flags & Proto.F_GATE == 0, "关闭后无 GATE")
	t.step(hand(), 4.0)
	t.step(hand({"by": true}), 4.1)
	c = t.step(hand({"by": true}), 5.2)
	check(t.enabled, "再次开启")
	check(c.flags & Proto.F_RIGHT_RESET == 0, "第二次开启不自动回零点")
	check(c.left.size() == 7 and c.left[6] == 5.0 and c.left[0] == 0.0, "左臂恒为零点、夹爪张开")
	check(c.mode1 == 2 and c.chx == 0.0 and c.chy == 0.0 and c.chz == 0.0 and c.height == 0.0, "底盘 / 升降恒为 0")


func test_teleop_follow_gripper_reset() -> void:
	print("状态机：跟随、夹爪、回零点")
	var t = make_teleop()
	var pose := Transform3D(Basis.IDENTITY, Vector3(0, 1, -0.3))
	var c: Dictionary = t.step(hand({"trigger": 1.0, "pose": pose}), 0.0)
	check(c.flags & Proto.F_RIGHT_FOLLOW == 0, "遥操关闭时不跟随")
	t.enabled = true
	t.ever_enabled = true
	c = t.step(hand({"trigger": 1.0, "pose": pose}), 0.1)
	check(c.flags & Proto.F_RIGHT_FOLLOW == 0, "开启前已按住扳机 → 不跟随")
	t.step(hand({"pose": pose}), 0.2)
	t.step(hand({"trigger": 1.0, "pose": pose}), 0.3)
	c = t.step(hand({"trigger": 1.0, "pose": pose.translated(Vector3(0, 0, -0.1)), "grip": 1.0}), 0.4)
	check(c.flags & Proto.F_RIGHT_FOLLOW != 0, "松开再按 → 跟随")
	check(t.arm_state == Teleop.ArmState.OPERATE, "状态：跟随")
	near(c.right[0], 0.1, 1e-5, "前推 10 cm → x")
	near(c.right[6], 0.0, 1e-5, "握持到底 → 夹爪 0")
	c = t.step(hand({"trigger": 0.0, "grip": 0.0, "pose": pose.translated(Vector3(0.2, 0, 0))}), 0.5)
	near(c.right[0], 0.1, 1e-5, "松开扳机 → 位置冻结")
	near(c.right[1], 0.0, 1e-5, "松开扳机后乱动 → y 不变")
	near(c.right[6], 5.0, 1e-5, "松开握持 → 夹爪 5")
	c = t.step(hand({"ax": true}), 0.6)
	near(c.right[0], 0.0, 1e-6, "A 回零点")
	check(c.flags & Proto.F_RIGHT_RESET != 0, "回零点标志")
	check(t.arm_state == Teleop.ArmState.RESET, "状态：回零点")
	c = t.step(hand(), 1.0)
	check(c.flags & Proto.F_RIGHT_RESET == 0, "回零点标志 0.3 s 后清除")
	t.enabled = false
	c = t.step(hand({"grip": 1.0, "ax": true}), 1.1)
	near(c.right[6], 5.0, 1e-5, "遥操关闭时夹爪不动")


func test_teleop_pause_requires_release() -> void:
	print("状态机：暂停")
	var t = make_teleop()
	t.enabled = true
	t.ever_enabled = true
	var pose := Transform3D(Basis.IDENTITY, Vector3.ZERO)
	t.step(hand(), 0.0)
	t.step(hand({"trigger": 1.0, "pose": pose}), 0.1)
	var c: Dictionary = t.step(hand({"trigger": 1.0, "pose": pose.translated(Vector3(0, 0, -0.05)), "grip": 0.5}), 0.2)
	near(c.right[0], 0.05, 1e-5, "跟随")
	t.paused = true
	c = t.step(hand({"trigger": 1.0, "pose": pose.translated(Vector3(0, 0, -0.3)), "grip": 0.0}), 0.3)
	check(c.flags & Proto.F_PAUSED != 0, "暂停标志")
	near(c.right[0], 0.05, 1e-5, "暂停时手臂冻结")
	near(c.right[6], 2.5, 1e-5, "暂停时夹爪冻结")
	t.paused = false
	c = t.step(hand({"trigger": 1.0, "pose": pose.translated(Vector3(0, 0, -0.4))}), 0.4)
	near(c.right[0], 0.05, 1e-5, "恢复后扳机仍按着 → 不跟随")


func test_protocol_xcheck() -> void:
	print("协议：与 Python 交叉校验")
	var tool := ProjectSettings.globalize_path("res://").path_join("../tools/proto_xcheck.py").simplify_path()
	var out := []
	if OS.execute("python3", [tool, "hello"], out) != 0:
		check(false, "运行 proto_xcheck.py 失败（需要 python3）")
		return
	var raw: PackedByteArray = out[0].strip_edges().hex_decode()
	var h := Proto.parse(raw)
	check(h.get("type") == Proto.T_HELLO, "HELLO 解析")
	check(h.get("session") == 0xDEADBEEF and h.get("port") == 47800 and h.get("ros_version") == 2, "HELLO 字段")
	check(h.get("robot_name") == "Franka-测试" and h.get("owner") == Proto.OWNER_OTHER, "HELLO 名称 / owner")
	check(h.get("workspace", []).size() == 24, "HELLO 工作空间 24 个值")
	if h.get("workspace", []).size() == 24:
		near(h.workspace[23], 1.3, 1e-6, "HELLO 工作空间最后一个值")
	var arx := raw.duplicate()
	for i in 4:
		arx[i] = "AXVR".to_ascii_buffer()[i]
	check(Proto.parse(arx).is_empty(), "ARX 的包（magic AXVR）被丢弃")

	out = []
	OS.execute("python3", [tool, "stat"], out)
	var s := Proto.parse(out[0].strip_edges().hex_decode())
	check(s.get("type") == Proto.T_STAT, "STAT 解析")
	check(s.get("echo_seq") == 777 and s.get("echo_t_vr_us") == 123456789012 and s.get("hold_us") == 4321,
			"STAT 回显字段")
	check(s.get("watchdog") == Proto.WD_HOLD and s.get("owner") == Proto.OWNER_YOU, "STAT 状态字段")
	near(s.get("loss", 0.0), 0.125, 1e-6, "STAT loss")

	var t = make_teleop()
	var c: Dictionary = t.step(hand(), 0.0)
	c.session = 42
	c.seq = 1001
	c.t_vr_us = 9876543210
	c.flags = Proto.F_GATE | Proto.F_SMOOTH
	c.right = PackedFloat32Array([0.1, -0.2, 0.3, 0.4, -0.5, 0.6, 4.5])
	var pkt := Proto.pack_ctrl(c)
	check(pkt.size() == 100, "CTRL 长度 100 字节（实际 %d）" % pkt.size())
	check(pkt.slice(0, 4) == "FRVR".to_ascii_buffer(), "CTRL magic = FRVR")
	out = []
	OS.execute("python3", [tool, "ctrl", pkt.hex_encode()], out)
	var d = JSON.parse_string(out[0])
	check(d != null and d.seq == 1001 and d.t_vr_us == 9876543210 and d.mode1 == 2 and d.session == 42,
			"Python 解码 CTRL 头")
	if d != null:
		near(d.right[1], -0.2, 1e-6, "Python 解码 right.y")
		near(d.right[6], 4.5, 1e-6, "Python 解码 right.gripper")
		near(d.left[6], 5.0, 1e-6, "Python 解码 left.gripper（恒张开）")
		check(int(d.flags) == Proto.F_GATE | Proto.F_SMOOTH, "Python 解码 flags")
