## 语音提示（assets/voice/*.wav，由 tools/make_voice.sh 用 macOS say -v Tingting 生成）。
extends AudioStreamPlayer

const CLIPS := ["gate_on", "gate_off", "connected", "link_down", "paused", "resumed", "aligned"]

var enabled := true
var _streams := {}
var _queue: Array[String] = []


func _ready() -> void:
	for clip in CLIPS:
		var path := "res://assets/voice/%s.wav" % clip
		if ResourceLoader.exists(path):
			_streams[clip] = load(path)
	finished.connect(_next)


func say(clip: String) -> void:
	if not enabled or not _streams.has(clip):
		return
	if _queue.has(clip):
		return
	_queue.append(clip)
	if not playing:
		_next()


func _next() -> void:
	if _queue.is_empty():
		return
	stream = _streams[_queue.pop_front()]
	play()
