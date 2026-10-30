## 冒烟测试：加载 main.tscn（无 XR），跑若干帧、开关设置面板、模拟射线点击，连 dry-run 适配器。
## godot --headless --xr-mode off --path vr-app -s res://tests/smoke_main.gd -- --port=47913
extends SceneTree

var main: Node
var frame := 0


func _init() -> void:
	main = load("res://main.tscn").instantiate()
	root.add_child(main)


func _process(_delta: float) -> bool:
	frame += 1
	if frame == 2:
		var port := 47800
		for a in OS.get_cmdline_user_args():
			if a.begins_with("--port="):
				port = int(a.trim_prefix("--port="))
		main.settings.set_value("connection", "host", "127.0.0.1")
		main.settings.set_value("connection", "port", port)
		main.link.set_target("127.0.0.1", port)
	if frame == 30:
		main._open_panel()
		check(main.panel_open and main.panel3d.visible, "面板打开")
	if frame == 31:
		check(main.teleop.paused, "面板打开时遥操暂停")
	if frame >= 40 and frame < 60:
		# 射线从面板正前方 0.5 m 指向中心偏下，扳机在第 45~47 帧按下
		var p: Node3D = main.panel3d
		var aim := Transform3D(p.global_transform.basis, p.global_transform * Vector3(0.0, -0.25, 0.5))
		p.process_pointer(aim, true, frame >= 45 and frame < 48)
	if frame == 70:
		main.panel.refresh()
		main._on_setting_changed("display", "hud")
		main._on_setting_changed("control", "send_rate")
		main._close_panel()
		check(not main.panel_open, "面板关闭")
	if frame == 90:
		print("link.state=%d robots=%d host=%s" % [main.link.state, main.link.robots.size(), main.link.host])
		check(main.link.state == 1, "连上 dry-run 适配器（状态 OK）")
		check(main.link.robots.has("127.0.0.1"), "发现列表里有适配器")
		check(main.link.robots.get("127.0.0.1", {}).get("robot_name", "") == "Franka-FR3", "适配器名 Franka-FR3")
		check(main.teleop.solver.box_hi.x > 0.29 and main.teleop.solver.box_hi.x < 0.31, "从 HELLO 取到右臂工作空间")
		check(main.hud._lines[0].text.begins_with("遥操关"), "HUD 第 1 行：" + main.hud._lines[0].text)
		check(main.hud._lines[2].text.begins_with("链路：正常"), "HUD 链路行：" + main.hud._lines[2].text)
		print("HUD：")
		for l in main.hud._lines:
			print("  ", l.text)
		print("SMOKE_DONE")
		DirAccess.remove_absolute(ProjectSettings.globalize_path("user://settings.cfg"))  # 关面板时保存的测试设置
		return true
	return false


func check(cond: bool, what: String) -> void:
	print(("  ✓ " if cond else "  ✗ ") + what)
