## 把 2D 面板渲染进 SubViewport，贴到 3D 四边形上；右手 aim 射线 + 扳机点击。
extends Node3D

const SIZE_M := Vector2(1.0, 0.72)
const SIZE_PX := Vector2i(1280, 922)

var viewport := SubViewport.new()
var _quad := MeshInstance3D.new()
var _laser := MeshInstance3D.new()
var _dot := MeshInstance3D.new()
var _pressed := false
var _last_px := Vector2(-1, -1)


func _init(content: Control) -> void:
	viewport.size = SIZE_PX
	viewport.disable_3d = true
	viewport.transparent_bg = true
	viewport.gui_embed_subwindows = true
	viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	content.set_anchors_preset(Control.PRESET_FULL_RECT)
	viewport.add_child(content)
	add_child(viewport)

	var mesh := QuadMesh.new()
	mesh.size = SIZE_M
	_quad.mesh = mesh
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_quad.material_override = mat
	add_child(_quad)

	var laser_mesh := BoxMesh.new()
	laser_mesh.size = Vector3(0.003, 0.003, 1.0)
	_laser.mesh = laser_mesh
	_laser.material_override = _unshaded(Color(0.4, 0.8, 1.0, 0.8))
	_laser.top_level = true
	add_child(_laser)

	var dot_mesh := SphereMesh.new()
	dot_mesh.radius = 0.008
	dot_mesh.height = 0.016
	_dot.mesh = dot_mesh
	_dot.material_override = _unshaded(Color(1, 1, 1))
	_dot.top_level = true
	add_child(_dot)
	visible = false


func _ready() -> void:
	(_quad.material_override as StandardMaterial3D).albedo_texture = viewport.get_texture()


func _unshaded(c: Color) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.albedo_color = c
	if c.a < 1.0:
		m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	m.no_depth_test = true
	return m


## 在头显前方 0.75 m 处打开，面向用户（head 为世界坐标）。
func open_in_front(head: Transform3D) -> void:
	var f := -head.basis.z
	f.y = 0
	if f.length() < 1e-3:
		f = Vector3.FORWARD
	f = f.normalized()
	var pos := head.origin + f * 0.75 + Vector3(0, -0.1, 0)
	global_transform = Transform3D(Basis.looking_at(f, Vector3.UP), pos)  # 四边形正面（+Z）朝向用户
	visible = true
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_pressed = false


func close() -> void:
	if _pressed:
		_send_button(_last_px, false)
	visible = false
	viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED


## aim：右手 aim 位姿（世界坐标）；press：扳机是否按下。
func process_pointer(aim: Transform3D, tracked: bool, press: bool) -> void:
	_laser.visible = false
	_dot.visible = false
	if not visible or not tracked:
		return
	var dir := -aim.basis.z
	var inv := global_transform.affine_inverse()
	var o := inv * aim.origin
	var d := inv.basis * dir
	var length := 2.0
	var px := Vector2(-1, -1)
	if d.z < -1e-4 and o.z > 0:
		var t := -o.z / d.z
		var hit := o + d * t
		if absf(hit.x) <= SIZE_M.x / 2 and absf(hit.y) <= SIZE_M.y / 2:
			length = t
			px = Vector2((hit.x / SIZE_M.x + 0.5) * SIZE_PX.x, (0.5 - hit.y / SIZE_M.y) * SIZE_PX.y)
			_dot.global_position = global_transform * hit
			_dot.visible = true
	_laser.global_transform = Transform3D(aim.basis.orthonormalized() * Basis.from_scale(Vector3(1, 1, length)),
			aim.origin + dir * length / 2)
	_laser.visible = true

	if px.x >= 0:
		if px != _last_px:
			var m := InputEventMouseMotion.new()
			m.position = px
			m.global_position = px
			m.relative = px - _last_px if _last_px.x >= 0 else Vector2.ZERO
			m.button_mask = MOUSE_BUTTON_MASK_LEFT if _pressed else 0
			viewport.push_input(m)
			_last_px = px
		if press != _pressed:
			_pressed = press
			_send_button(px, press)
	elif _pressed and not press:
		_pressed = false
		_send_button(_last_px, false)


func _send_button(px: Vector2, down: bool) -> void:
	var b := InputEventMouseButton.new()
	b.button_index = MOUSE_BUTTON_LEFT
	b.pressed = down
	b.position = px
	b.global_position = px
	b.button_mask = MOUSE_BUTTON_MASK_LEFT if down else 0
	viewport.push_input(b)
