## 设置。Quest 上存在 /sdcard/Android/data/com.infinity4b.frankateleop/files/settings.cfg（adb 可读写）；
## Godot 的 user:// 在 Android 上是应用私有目录，发布版 adb 读不到，所以不用它。
extends RefCounted

const USER_PATH := "user://settings.cfg"

const DEFAULTS := {
	"connection": {
		"host": "",           # 空 = 自动发现
		"port": 47800,
	},
	"control": {
		"pos_scale": 1.0,     # 手柄位移 → 末端位移
		"rot_enabled": true,
		"rot_scale": 1.0,
		"send_rate": 90,      # 72 / 90 / 120，同时设置头显刷新率
		"smooth": true,       # 适配器在两包之间插值
		"gripper_open": 5.0,  # 不按握持键时的夹爪值（适配器：高于 3.5 张开）
		"gripper_close": 0.0, # 握持键按到底时的夹爪值（适配器：低于 1.5 抓取）
		"gripper_invert": false,
	},
	"display": {
		"hud": true,
		"voice": true,
		"haptics": true,
		"passthrough": true,
	},
	# 标定用：轴向不对时把对应分量改成 -1（只在文件里改，面板不显示）
	"calib": {
		"pos_sign": Vector3(1, 1, 1),
		"rot_sign": Vector3(1, 1, 1),
	},
}

var _cfg := ConfigFile.new()


func path() -> String:
	if OS.get_name() != "Android":
		return USER_PATH
	# user_data_dir 形如 /data/user/0/<包名>/files
	var pkg := OS.get_user_data_dir().get_base_dir().get_file()
	if pkg.is_empty():
		pkg = "com.infinity4b.frankateleop"
	return "/storage/emulated/0/Android/data/%s/files/settings.cfg" % pkg


func load_file() -> void:
	_cfg = ConfigFile.new()
	var p := path()
	if FileAccess.file_exists(p):
		_cfg.load(p)
		return
	if p != USER_PATH and FileAccess.file_exists(USER_PATH):
		_cfg.load(USER_PATH)  # 旧版本存在私有目录里，迁移过来
	# 首次运行写出标定段，方便 adb pull 后手改
	for key in DEFAULTS.calib:
		if not _cfg.has_section_key("calib", key):
			_cfg.set_value("calib", key, DEFAULTS.calib[key])
	save()


func save() -> void:
	var err := _cfg.save(path())
	if err != OK:
		push_warning("保存设置到 %s 失败：%s，改存 user://" % [path(), error_string(err)])
		_cfg.save(USER_PATH)


func get_value(section: String, key: String) -> Variant:
	return _cfg.get_value(section, key, DEFAULTS[section][key])


func set_value(section: String, key: String, value: Variant) -> void:
	_cfg.set_value(section, key, value)


## 恢复默认：保留连接和标定。
func reset_defaults() -> void:
	var host = get_value("connection", "host")
	var calib := {}
	for key in DEFAULTS.calib:
		calib[key] = get_value("calib", key)
	_cfg = ConfigFile.new()
	_cfg.set_value("connection", "host", host)
	for key in calib:
		_cfg.set_value("calib", key, calib[key])
