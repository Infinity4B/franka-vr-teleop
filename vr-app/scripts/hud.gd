## HUD：挂在 XRCamera3D 下方视野边缘的几行状态文字。
extends Node3D

const C_OK := Color(0.45, 1.0, 0.55)
const C_WARN := Color(1.0, 0.82, 0.3)
const C_BAD := Color(1.0, 0.4, 0.35)
const C_DIM := Color(0.85, 0.85, 0.85)

var _lines: Array[Label3D] = []


func _init(font: Font, line_count := 4) -> void:
	position = Vector3(0, -0.2, -0.85)
	rotation_degrees = Vector3(12, 0, 0)
	for i in line_count:
		var l := Label3D.new()
		l.font = font
		l.font_size = 40
		l.pixel_size = 0.0006
		l.outline_size = 10
		l.outline_modulate = Color(0, 0, 0, 0.85)
		l.no_depth_test = true
		l.render_priority = 10
		l.outline_render_priority = 9
		l.position = Vector3(0, -i * 0.034, 0)
		l.modulate = C_DIM
		add_child(l)
		_lines.append(l)


func set_line(i: int, text: String, color := C_DIM) -> void:
	_lines[i].text = text
	_lines[i].modulate = color
