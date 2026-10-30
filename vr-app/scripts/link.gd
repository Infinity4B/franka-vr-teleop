## UDP 链路：广播发现、每帧发 CTRL、收 STAT 算 RTT / 丢包，给出链路三态。
extends RefCounted

const Proto := preload("res://scripts/protocol.gd")

enum State { NONE, OK, DELAY, DOWN, BUSY }

const STAT_TIMEOUT := 1.0     # s 没收到 STAT 视为断开
const DELAY_STAT_AGE := 0.2   # s
const DELAY_RTT_MS := 50.0
const DELAY_LOSS := 0.05
const DISCOVER_FAST := 1.0    # 未连接时的发现间隔
const DISCOVER_SLOW := 5.0

signal robots_changed
signal state_changed(state: int)

var udp := PacketPeerUDP.new()
var session := randi()
var host := ""
var port := Proto.DEFAULT_PORT
var robots := {}            # ip -> HELLO 字典 + seen
var seq := 0
var stat := {}
var stat_time := -INF
var rtt_ms := 0.0
var rtt_samples: Array[float] = []
var state: int = State.NONE
var device_name := "Quest3S"

var _next_discover := 0.0


func _init() -> void:
	var err := udp.bind(0)
	if err != OK:
		push_error("UDP bind 失败：%s" % error_string(err))
	udp.set_broadcast_enabled(true)


func set_target(p_host: String, p_port: int) -> void:
	if p_host != host or p_port != port:
		host = p_host
		port = p_port
		stat = {}
		stat_time = -INF
		rtt_samples.clear()
		_next_discover = 0.0  # 马上发现一次，拿到该机器人的限幅


func now_s() -> float:
	return Time.get_ticks_usec() / 1e6


func send_ctrl(c: Dictionary) -> void:
	if host.is_empty():
		return
	seq += 1
	c.session = session
	c.seq = seq
	c.t_vr_us = Time.get_ticks_usec()
	udp.set_dest_address(host, port)
	udp.put_packet(Proto.pack_ctrl(c))


func discover() -> void:
	var pkt := Proto.pack_discover(session, device_name)
	var targets := ["255.255.255.255"]
	for ip in IP.get_local_addresses():
		var parts: PackedStringArray = ip.split(".")
		if parts.size() == 4 and not ip.begins_with("127."):
			targets.append("%s.%s.%s.255" % [parts[0], parts[1], parts[2]])  # 按 /24 估计
	if not host.is_empty():
		targets.append(host)
	for t in targets:
		udp.set_dest_address(t, port)
		udp.put_packet(pkt)


func poll() -> void:
	var now := now_s()
	if now >= _next_discover:
		discover()
		_next_discover = now + (DISCOVER_SLOW if state == State.OK else DISCOVER_FAST)
	var changed := false
	while udp.get_available_packet_count() > 0:
		var data := udp.get_packet()
		var ip := udp.get_packet_ip()
		var d := Proto.parse(data)
		if d.is_empty():
			continue
		if d.type == Proto.T_HELLO:
			d.seen = now
			d.ip = ip
			robots[ip] = d
			changed = true
		elif d.type == Proto.T_STAT and ip == host and d.session == session:
			stat = d
			stat_time = now
			if d.owner == Proto.OWNER_YOU and d.echo_t_vr_us > 0:
				var rtt: float = (Time.get_ticks_usec() - d.echo_t_vr_us - d.hold_us) / 1000.0
				if rtt >= 0 and rtt < 5000:
					rtt_ms = rtt
					rtt_samples.append(rtt)
					if rtt_samples.size() > 300:
						rtt_samples.pop_front()
	for ip in robots.keys():
		if now - robots[ip].seen > 3 * DISCOVER_SLOW:
			robots.erase(ip)
			changed = true
	if changed:
		robots_changed.emit()
	_update_state(now)


func _update_state(now: float) -> void:
	var s: int
	if host.is_empty():
		s = State.NONE
	elif now - stat_time > STAT_TIMEOUT:
		s = State.DOWN
	elif stat.owner == Proto.OWNER_OTHER:
		s = State.BUSY
	elif now - stat_time > DELAY_STAT_AGE or rtt_ms > DELAY_RTT_MS or stat.loss > DELAY_LOSS:
		s = State.DELAY
	else:
		s = State.OK
	if s != state:
		state = s
		state_changed.emit(s)


func rtt_p95() -> float:
	if rtt_samples.is_empty():
		return NAN
	var a := rtt_samples.duplicate()
	a.sort()
	return a[int(a.size() * 0.95)]


func loss() -> float:
	return stat.get("loss", 0.0)


func robot_height() -> float:
	if stat.is_empty():
		return NAN
	return stat.height_fb if not is_nan(stat.height_fb) else stat.height_cmd


## 没有手动 IP 时自动选第一个空闲的机器人。
func auto_pick() -> String:
	for ip in robots:
		if robots[ip].owner != Proto.OWNER_OTHER:
			return ip
	return ""
