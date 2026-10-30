## 入口：XR 启动与生命周期、读右手柄、跑状态机、每帧发 CTRL、HUD / 面板 / 语音 / 振动。
## 只用右手柄：右摇杆短按开关设置面板，长按 1 s 以右手柄指向重新对齐前方。
extends Node3D

const Settings := preload("res://scripts/settings.gd")
const Link := preload("res://scripts/link.gd")
const Teleop := preload("res://scripts/teleop.gd")
const ArmSolver := preload("res://scripts/arm_solver.gd")
const Hud := preload("res://scripts/hud.gd")
const Panel3D := preload("res://scripts/panel_3d.gd")
const SettingsPanel := preload("res://scripts/settings_panel.gd")
const Voice := preload("res://scripts/voice.gd")
const Proto := preload("res://scripts/protocol.gd")

const FONT_PATH := "res://assets/fonts/NotoSansSC-UI.ttf"
const STICK_LONG_PRESS := 1.0

@onready var camera: XRCamera3D = $XROrigin3D/XRCamera3D
@onready var right_hand: XRController3D = $XROrigin3D/RightHand
@onready var right_aim: XRController3D = $XROrigin3D/RightAim
@onready var world_env: WorldEnvironment = $WorldEnvironment

var xr: OpenXRInterface
var settings := Settings.new()
var link := Link.new()
var teleop: Teleop
var hud: Hud
var panel: SettingsPanel
var panel3d: Panel3D
var voice: Voice

var focused := false
var panel_open := false
var _aligned := false
var _align_wait := 0
var _stick_down := false
var _stick_down_at := 0.0
var _stick_long_done := false
var _last_link_state := -1
var _was_paused := false
var _next_rate_check := 0.0
var _strokes_seen := 0
var _indicator: StandardMaterial3D


func _ready() -> void:
	settings.load_file()
	var font: Font = load(FONT_PATH) if ResourceLoader.exists(FONT_PATH) else ThemeDB.fallback_font

	teleop = Teleop.new(settings)
	teleop.enabled_changed.connect(_on_enabled)
	teleop.arm_state_changed.connect(_on_arm_state)
	link.state_changed.connect(_on_link_state)
	link.device_name = OS.get_model_name().left(16)

	hud = Hud.new(font, 3)
	camera.add_child(hud)
	voice = Voice.new()
	add_child(voice)
	panel = SettingsPanel.new(settings, link, font)
	panel.changed.connect(_on_setting_changed)
	panel.connect_requested.connect(_on_connect_requested)
	panel.realign_requested.connect(func(): _request_align(1))
	panel.defaults_requested.connect(_on_defaults)
	panel.close_requested.connect(_close_panel)
	panel3d = Panel3D.new(panel)
	add_child(panel3d)

	var m := MeshInstance3D.new()
	var s := SphereMesh.new()
	s.radius = 0.012
	s.height = 0.024
	m.mesh = s
	_indicator = StandardMaterial3D.new()
	_indicator.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.material_override = _indicator
	m.position = Vector3(0, 0.03, -0.02)
	right_hand.add_child(m)
	_on_arm_state(Teleop.ArmState.STAY)

	_apply_settings()
	link.set_target(settings.get_value("connection", "host"), settings.get_value("connection", "port"))

	xr = XRServer.find_interface("OpenXR")
	if xr and xr.is_initialized():
		get_viewport().use_xr = true
		DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
		xr.session_begun.connect(_on_session_begun)
		xr.session_focussed.connect(func(): _set_focus(true))
		xr.session_visible.connect(func(): _set_focus(false))
		xr.session_stopping.connect(func(): _set_focus(false))
		xr.pose_recentered.connect(_on_pose_recentered)
	else:
		push_warning("OpenXR 未初始化，以桌面模式运行（只用于调试）")
		focused = true


func _notification(what: int) -> void:
	# 恢复由 OpenXR 的 session_focussed 负责
	if what == NOTIFICATION_APPLICATION_PAUSED:
		_set_focus(false)
	elif what == NOTIFICATION_WM_CLOSE_REQUEST:
		settings.save()


func _process(_delta: float) -> void:
	var now := link.now_s()
	link.poll()
	_auto_connect()
	# 系统会把刷新率改回 72 Hz（实测启动 1.5 s 后），定期检查并重新请求
	if focused and now >= _next_rate_check:
		_next_rate_check = now + 2.0
		_apply_refresh_rate()

	if not _aligned:
		_align_wait -= 1
		if _align_wait <= 0 and _head_tracked():
			teleop.set_reference(ArmSolver.yaw_reference(camera.transform))
			teleop.break_follow()
			_aligned = true
			voice.say("aligned")
			print("已对齐前方：参考 yaw = %.1f°" % rad_to_deg(ArmSolver.yaw_reference(camera.transform).get_euler().y))

	# 右摇杆按下：短按开关面板；长按 1 s 以右手柄指向重新对齐前方
	var stick := right_hand.is_button_pressed("primary_click")
	if stick and not _stick_down:
		_stick_down_at = now
		_stick_long_done = false
	elif stick and not _stick_long_done and now - _stick_down_at >= STICK_LONG_PRESS:
		_stick_long_done = true
		_align_to_right_aim()
	elif not stick and _stick_down and not _stick_long_done:
		if panel_open:
			_close_panel()
		else:
			_open_panel()
	_stick_down = stick

	teleop.paused = panel_open or not focused or not _aligned
	var robot: Dictionary = link.robots.get(link.host, {})
	teleop.set_workspace(robot.get("workspace", []))
	link.send_ctrl(teleop.step(_read_right(), now))
	_log_strokes()
	# 头显挂在胸前时看不到 HUD，暂停 / 恢复用语音提示（只在遥操开启时）
	if teleop.enabled and teleop.paused != _was_paused:
		voice.say("paused" if teleop.paused else "resumed")
	_was_paused = teleop.paused

	if panel_open:
		var trig := right_aim.get_float("trigger") > 0.5
		panel3d.process_pointer(right_aim.global_transform, right_aim.get_has_tracking_data(), trig)
	_update_hud(now)


func _read_right() -> Dictionary:
	return {
		"pose": right_hand.transform,
		"tracked": right_hand.get_has_tracking_data(),
		"trigger": right_hand.get_float("trigger"),
		"grip": right_hand.get_float("grip"),
		"ax": right_hand.is_button_pressed("ax_button"),
		"by": right_hand.is_button_pressed("by_button"),
	}


func _log_strokes() -> void:
	var s = teleop.solver
	if s.strokes == _strokes_seen:
		return
	_strokes_seen = s.strokes
	var k: Dictionary = s.last_stroke
	var c: Vector3 = k.cmd * 100.0
	print("一次跟随：手柄移动 %.1f cm、转动 %.1f°；指令变化 %.1f cm (x %+.1f y %+.1f z %+.1f)、转动 %.1f°" % [
			k.raw_m * 100.0, k.raw_deg, c.length(), c.x, c.y, c.z, k.cmd_deg])


func _head_tracked() -> bool:
	if xr == null or not xr.is_initialized():
		return true
	return not camera.transform.is_equal_approx(Transform3D.IDENTITY)


# ------------------------------------------------------------------ 生命周期
func _on_session_begun() -> void:
	_apply_settings()


func _set_focus(on: bool) -> void:
	if focused == on:
		return
	focused = on
	print("焦点：", "获得" if on else "失去（遥操暂停）")


func _on_pose_recentered() -> void:
	print("收到重置视野（Meta 长按）")
	# 长按 Meta 键重置视野：跟踪空间变了，重新对齐参考方向
	XRServer.center_on_hmd(XRServer.RESET_BUT_KEEP_TILT, true)
	teleop.break_follow()
	_request_align(3)


func _align_to_right_aim() -> void:
	if not right_aim.get_has_tracking_data():
		_haptic(1.0, 0.3)
		return
	var ref := ArmSolver.yaw_from_direction(-right_aim.transform.basis.z)
	teleop.set_reference(ref)
	teleop.break_follow()
	_aligned = true
	voice.say("aligned")
	_haptic(0.6, 0.15)
	print("已对齐前方（右手柄指向）：参考 yaw = %.1f°" % rad_to_deg(ref.get_euler().y))


func _request_align(wait_frames: int) -> void:
	_aligned = false
	_align_wait = wait_frames


# ------------------------------------------------------------------ 面板
func _open_panel() -> void:
	panel.refresh()
	panel3d.open_in_front(camera.global_transform)
	panel_open = true
	hud.visible = false  # HUD 不做深度测试，会叠在面板上


func _close_panel() -> void:
	panel3d.close()
	panel_open = false
	hud.visible = settings.get_value("display", "hud")
	settings.save()


func _on_setting_changed(section: String, key: String) -> void:
	if section == "display" or (section == "control" and key == "send_rate"):
		_apply_settings()


func _on_connect_requested(host: String, port: int) -> void:
	if port <= 0 or port > 65535:
		port = Proto.DEFAULT_PORT
	settings.set_value("connection", "host", host)
	settings.set_value("connection", "port", port)
	settings.save()
	link.set_target(host, port)
	panel.refresh()


func _on_defaults() -> void:
	settings.reset_defaults()
	settings.save()
	_apply_settings()
	panel.refresh()


func _auto_connect() -> void:
	if not str(settings.get_value("connection", "host")).is_empty():
		return
	if link.host.is_empty() or (link.state == Link.State.BUSY) or not link.robots.has(link.host):
		var ip := link.auto_pick()
		if not ip.is_empty() and ip != link.host:
			link.set_target(ip, link.robots[ip].port)


func _apply_settings() -> void:
	hud.visible = settings.get_value("display", "hud") and not panel_open
	voice.enabled = settings.get_value("display", "voice")
	var pt: bool = settings.get_value("display", "passthrough")
	get_viewport().transparent_bg = pt
	var env := world_env.environment
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0, 0, 0, 0) if pt else Color(0.12, 0.12, 0.14, 1)
	if xr and xr.is_initialized():
		var mode := XRInterface.XR_ENV_BLEND_MODE_ALPHA_BLEND if pt else XRInterface.XR_ENV_BLEND_MODE_OPAQUE
		if mode in xr.get_supported_environment_blend_modes():
			xr.environment_blend_mode = mode
		_apply_refresh_rate()


func _apply_refresh_rate() -> void:
	if xr == null or not xr.is_initialized():
		return
	var want := float(settings.get_value("control", "send_rate"))
	var rates: Array = xr.get_available_display_refresh_rates()
	var best := 0.0
	for r in rates:
		if absf(r - want) < absf(best - want):
			best = r
	var cur := xr.get_display_refresh_rate()
	if best > 0 and absf(best - cur) > 0.5:
		print("刷新率 %.0f Hz → 请求 %.0f Hz" % [cur, best])
		xr.set_display_refresh_rate(best)


# ------------------------------------------------------------------ 提示
func _haptic(amplitude: float, duration: float) -> void:
	if settings.get_value("display", "haptics"):
		right_hand.trigger_haptic_pulse("haptic", 0.0, amplitude, duration, 0.0)


func _on_enabled(on: bool) -> void:
	voice.say("gate_on" if on else "gate_off")
	_haptic(0.8 if on else 0.5, 0.15)


func _on_arm_state(state: int) -> void:
	var colors := {Teleop.ArmState.STAY: Color(0.6, 0.6, 0.6), Teleop.ArmState.OPERATE: Color(0.3, 1.0, 0.4),
			Teleop.ArmState.RESET: Color(1.0, 0.8, 0.2)}
	_indicator.albedo_color = colors[state]
	if state == Teleop.ArmState.OPERATE:
		_haptic(0.3, 0.03)
	elif state == Teleop.ArmState.RESET:
		_haptic(0.5, 0.08)


func _on_link_state(state: int) -> void:
	if state == Link.State.OK and _last_link_state in [-1, Link.State.NONE, Link.State.DOWN]:
		voice.say("connected")
	elif state == Link.State.DOWN and _last_link_state in [Link.State.OK, Link.State.DELAY]:
		voice.say("link_down")
		_haptic(1.0, 0.4)
	_last_link_state = state


func _update_hud(now: float) -> void:
	if not hud.visible:
		return
	var hold := teleop.toggle_progress(now)
	if hold > 0.0:
		hud.set_line(0, "%s遥操…%d%%" % ["关闭" if teleop.enabled else "开启", int(hold * 100)], Hud.C_WARN)
	elif not teleop.enabled:
		hud.set_line(0, "遥操关：长按 B 1 秒开启", Hud.C_WARN)
	elif teleop.paused:
		hud.set_line(0, "已暂停（%s）" % ("设置面板" if panel_open else "未对齐" if not _aligned else "失去焦点"),
				Hud.C_WARN)
	else:
		hud.set_line(0, "遥操中：按住扳机跟随，A 回零点", Hud.C_OK)

	var names := {Teleop.ArmState.STAY: "保持", Teleop.ArmState.OPERATE: "跟随", Teleop.ArmState.RESET: "回零点"}
	var st: int = link.stat.get("status", 0)
	var lim := ""
	if teleop.solver.clamped:
		lim = "  到边界"
	elif st & Proto.S_RIGHT_LIMITED:
		lim = "  限速"
	hud.set_line(1, "手臂 %s%s    夹爪 %s" % [names[teleop.arm_state], lim, "闭合" if teleop.gripper < 2.5 else "张开"],
			Hud.C_WARN if lim else Hud.C_DIM)

	match link.state:
		Link.State.NONE:
			hud.set_line(2, "链路：未发现 Franka（短按右摇杆打开设置）", Hud.C_BAD)
		Link.State.DOWN:
			hud.set_line(2, "链路：断开  %s" % link.host, Hud.C_BAD)
		Link.State.BUSY:
			hud.set_line(2, "链路：被其他头显占用  %s" % link.host, Hud.C_BAD)
		_:
			var wd := {Proto.WD_IDLE: "", Proto.WD_ACTIVE: "", Proto.WD_HOLD: "  适配器保持中",
					Proto.WD_STOPPED: "  适配器已停发"}
			var fb := "正常" if st & Proto.S_RIGHT_FB else "无"
			var dry := "  dry-run" if st & Proto.S_DRY_RUN else ""
			hud.set_line(2, "链路：%s  RTT %.0f ms  丢包 %.1f%%  反馈 %s%s%s" % [
					"正常" if link.state == Link.State.OK else "延迟", link.rtt_ms, link.loss() * 100.0, fb,
					wd.get(link.stat.watchdog, ""), dry],
					Hud.C_OK if link.state == Link.State.OK else Hud.C_WARN)
