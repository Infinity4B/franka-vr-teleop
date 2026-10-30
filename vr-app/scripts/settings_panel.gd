## 设置面板（2D，渲染到 SubViewport 再贴到 3D 四边形上）。打开期间遥操暂停。
extends PanelContainer

const Proto := preload("res://scripts/protocol.gd")
const Link := preload("res://scripts/link.gd")

signal changed(section: String, key: String)
signal connect_requested(host: String, port: int)
signal realign_requested
signal defaults_requested
signal close_requested

var settings
var link

var _status: Label
var _robot_list: VBoxContainer
var _ip_edit: LineEdit
var _port_edit: LineEdit
var _edit_target: LineEdit
var _refreshers: Array[Callable] = []
var _grabber: ImageTexture
var _track: StyleBoxFlat
var _fill: StyleBoxFlat


func _init(p_settings, p_link, font: Font) -> void:
	settings = p_settings
	link = p_link
	var theme_ := Theme.new()
	theme_.default_font = font
	theme_.default_font_size = 30
	theme = theme_
	var bg := StyleBoxFlat.new()
	bg.bg_color = Color(0.1, 0.11, 0.13, 0.96)
	bg.set_corner_radius_all(18)
	bg.set_content_margin_all(24)
	add_theme_stylebox_override("panel", bg)
	# 默认主题的滑块和开关在 VR 里太小，换成粗轨道 + 大圆点
	_grabber = _circle(40, Color(0.85, 0.92, 1.0))
	_track = StyleBoxFlat.new()
	_track.bg_color = Color(0.25, 0.27, 0.3)
	_track.set_corner_radius_all(6)
	_track.content_margin_top = 6
	_track.content_margin_bottom = 6
	_fill = _track.duplicate()
	_fill.bg_color = Color(0.35, 0.6, 0.95)


func _ready() -> void:
	var root := VBoxContainer.new()
	root.add_theme_constant_override("separation", 14)
	add_child(root)

	var title := HBoxContainer.new()
	var t := Label.new()
	t.text = "Franka 遥操设置（打开期间遥操暂停）"
	t.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	title.add_child(t)
	title.add_child(_button("关闭", func(): close_requested.emit()))
	root.add_child(title)

	var tabs := TabContainer.new()
	tabs.size_flags_vertical = Control.SIZE_EXPAND_FILL
	root.add_child(tabs)
	tabs.add_child(_page_connection())
	tabs.add_child(_page_control())
	tabs.add_child(_page_display())
	tabs.add_child(_page_align())
	for i in tabs.get_tab_count():
		tabs.set_tab_title(i, ["连接", "操控", "显示提示", "对齐"][i])

	link.robots_changed.connect(_rebuild_robot_list)
	_rebuild_robot_list()


func _process(_delta: float) -> void:
	if not is_visible_in_tree():
		return
	var state_names := {Link.State.NONE: "未选择机器人", Link.State.OK: "正常", Link.State.DELAY: "延迟",
			Link.State.DOWN: "断开", Link.State.BUSY: "被其他头显占用"}
	var s := "当前：%s   %s" % [("%s:%d" % [link.host, link.port]) if link.host else "—",
			state_names.get(link.state, "?")]
	if link.state in [Link.State.OK, Link.State.DELAY]:
		s += "\nRTT %.1f ms（P95 %.1f）   丢包 %.1f%%   适配器收包 %.0f Hz" % [link.rtt_ms, link.rtt_p95(),
				link.loss() * 100.0, link.stat.get("rx_rate", 0.0)]
	_status.text = s


func refresh() -> void:
	for f in _refreshers:
		f.call()


# ------------------------------------------------------------------ 页面
func _page_connection() -> Control:
	var page := _page("连接")
	_status = Label.new()
	page.add_child(_status)

	page.add_child(_heading("自动发现（点选连接）"))
	_robot_list = VBoxContainer.new()
	page.add_child(_robot_list)
	page.add_child(_button("自动选择（清除手动 IP）", func(): connect_requested.emit("", int(_port_edit.text))))

	page.add_child(_heading("手动连接"))
	var row := HBoxContainer.new()
	_ip_edit = LineEdit.new()
	_ip_edit.placeholder_text = "机器人 IP"
	_ip_edit.editable = false
	_ip_edit.custom_minimum_size.x = 380
	_ip_edit.add_theme_color_override("font_uneditable_color", Color.WHITE)
	_port_edit = LineEdit.new()
	_port_edit.editable = false
	_port_edit.custom_minimum_size.x = 150
	_port_edit.add_theme_color_override("font_uneditable_color", Color.WHITE)
	_edit_target = _ip_edit
	row.add_child(_toggle_target("IP", _ip_edit))
	row.add_child(_ip_edit)
	row.add_child(_toggle_target("端口", _port_edit))
	row.add_child(_port_edit)
	row.add_child(_button("连接", func(): connect_requested.emit(_ip_edit.text.strip_edges(),
			int(_port_edit.text))))
	page.add_child(row)

	var pad := GridContainer.new()
	pad.columns = 6
	for k in ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0", ".", "⌫"]:
		var key: String = k
		var b := _button(key, func(): _key(key))
		b.custom_minimum_size = Vector2(110, 64)
		pad.add_child(b)
	page.add_child(pad)
	_refreshers.append(func():
		_ip_edit.text = settings.get_value("connection", "host")
		_port_edit.text = str(settings.get_value("connection", "port")))
	return _scroll(page)


func _page_control() -> Control:
	var page := _page("操控")
	page.add_child(_slider("位置缩放", "control", "pos_scale", 0.2, 3.0, 0.05))
	page.add_child(_check("旋转跟随", "control", "rot_enabled"))
	page.add_child(_slider("旋转缩放", "control", "rot_scale", 0.2, 2.0, 0.05))
	page.add_child(_choice("发送频率 Hz", "control", "send_rate", [72, 90, 120]))
	page.add_child(_check("平滑（适配器在两包之间插值）", "control", "smooth"))
	page.add_child(_slider("夹爪张开值", "control", "gripper_open", 0.0, 5.0, 0.1))
	page.add_child(_slider("夹爪闭合值", "control", "gripper_close", 0.0, 5.0, 0.1))
	page.add_child(_check("夹爪反向", "control", "gripper_invert"))
	return _scroll(page)


func _page_display() -> Control:
	var page := _page("显示提示")
	page.add_child(_check("HUD", "display", "hud"))
	page.add_child(_check("语音提示", "display", "voice"))
	page.add_child(_check("状态切换振动", "display", "haptics"))
	page.add_child(_check("透视", "display", "passthrough"))
	return _scroll(page)


func _page_align() -> Control:
	var page := _page("对齐")
	var tip := Label.new()
	tip.text = "面向机器人的正前方站好，再按“重新对齐前方”。\n之后转头不会影响手臂。不开面板也能对齐：右手柄指向机器人正前方，长按右摇杆 1 秒。"
	tip.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	page.add_child(tip)
	page.add_child(_button("重新对齐前方", func(): realign_requested.emit()))
	page.add_child(_heading(""))
	page.add_child(_button("恢复默认设置（保留连接和标定）", func(): defaults_requested.emit()))
	return _scroll(page)


# ------------------------------------------------------------------ 控件
func _page(page_name: String) -> VBoxContainer:
	var v := VBoxContainer.new()
	v.name = page_name
	v.add_theme_constant_override("separation", 12)
	v.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	return v


func _scroll(page: Control) -> Control:
	var s := ScrollContainer.new()
	s.name = page.name
	s.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	s.add_child(page)
	return s


func _heading(text: String) -> Label:
	var l := Label.new()
	l.text = text
	l.add_theme_color_override("font_color", Color(0.6, 0.8, 1.0))
	return l


func _button(text: String, cb: Callable) -> Button:
	var b := Button.new()
	b.text = text
	b.custom_minimum_size.y = 60
	b.pressed.connect(cb)
	return b


func _toggle_target(text: String, edit: LineEdit) -> Button:
	return _button(text, func(): _edit_target = edit)


func _key(k: String) -> void:
	if k == "⌫":
		_edit_target.text = _edit_target.text.left(-1)
	elif _edit_target == _port_edit and k == ".":
		return
	else:
		_edit_target.text += k


func _slider(label: String, section: String, key: String, lo: float, hi: float, step: float) -> Control:
	var row := HBoxContainer.new()
	var l := Label.new()
	l.text = label
	l.custom_minimum_size.x = 420
	var s := HSlider.new()
	s.min_value = lo
	s.max_value = hi
	s.step = step
	s.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	s.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	s.custom_minimum_size.y = 56
	s.add_theme_icon_override("grabber", _grabber)
	s.add_theme_icon_override("grabber_highlight", _grabber)
	s.add_theme_stylebox_override("slider", _track)
	s.add_theme_stylebox_override("grabber_area", _fill)
	s.add_theme_stylebox_override("grabber_area_highlight", _fill)
	var v := Label.new()
	v.custom_minimum_size.x = 110
	v.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	row.add_child(l)
	row.add_child(s)
	row.add_child(v)
	_refreshers.append(func():
		s.set_value_no_signal(settings.get_value(section, key))
		v.text = "%.2f" % s.value)
	s.value_changed.connect(func(val: float):
		v.text = "%.2f" % val
		settings.set_value(section, key, val)
		changed.emit(section, key))
	return row


func _check(label: String, section: String, key: String) -> Control:
	var row := HBoxContainer.new()
	var l := Label.new()
	l.text = label
	l.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(l)
	var b := Button.new()
	b.toggle_mode = true
	b.custom_minimum_size = Vector2(150, 60)
	var show := func(on: bool):
		b.text = "开" if on else "关"
		b.modulate = Color(0.6, 1.0, 0.65) if on else Color(0.75, 0.75, 0.75)
	row.add_child(b)
	_refreshers.append(func():
		var on: bool = settings.get_value(section, key)
		b.set_pressed_no_signal(on)
		show.call(on))
	b.toggled.connect(func(on: bool):
		show.call(on)
		settings.set_value(section, key, on)
		changed.emit(section, key))
	return row


static func _circle(d: int, c: Color) -> ImageTexture:
	var img := Image.create(d, d, false, Image.FORMAT_RGBA8)
	var r := d / 2.0
	for y in d:
		for x in d:
			var dist := Vector2(x + 0.5 - r, y + 0.5 - r).length()
			img.set_pixel(x, y, Color(c, clampf(r - dist, 0.0, 1.0)))
	return ImageTexture.create_from_image(img)


func _choice(label: String, section: String, key: String, values: Array) -> Control:
	var row := HBoxContainer.new()
	var l := Label.new()
	l.text = label
	l.custom_minimum_size.x = 420
	row.add_child(l)
	var group := ButtonGroup.new()
	var buttons := []
	for val in values:
		var b := Button.new()
		b.text = str(val)
		b.toggle_mode = true
		b.button_group = group
		b.custom_minimum_size = Vector2(120, 60)
		var value = val
		b.pressed.connect(func():
			settings.set_value(section, key, value)
			changed.emit(section, key))
		row.add_child(b)
		buttons.append([b, val])
	_refreshers.append(func():
		for pair in buttons:
			pair[0].set_pressed_no_signal(pair[1] == settings.get_value(section, key)))
	return row


func _rebuild_robot_list() -> void:
	if _robot_list == null:
		return
	for c in _robot_list.get_children():
		c.queue_free()
	if link.robots.is_empty():
		var l := Label.new()
		l.text = "（没有发现。确认 lab 上已运行 ./start.sh、防火墙已放行 UDP 47800，且在同一网段；否则用下方手动 IP）"
		l.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		_robot_list.add_child(l)
		return
	var owners := {Proto.OWNER_FREE: "空闲", Proto.OWNER_YOU: "本机控制中", Proto.OWNER_OTHER: "被占用"}
	for ip in link.robots:
		var r: Dictionary = link.robots[ip]
		var ros := "dry-run" if r.ros_version == 0 else "真机"
		var host: String = ip
		var port: int = r.port
		_robot_list.add_child(_button("%s   %s   %s   %s" % [r.robot_name, ip, ros, owners.get(r.owner, "?")],
				func(): connect_requested.emit(host, port)))
