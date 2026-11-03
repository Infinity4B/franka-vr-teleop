# -- coding: UTF-8
"""python -m franka_vr_bridge [--config config.yaml] [--home] [--dry-run] [--csv out.csv] [--host IP] [--port N]

Quest 3S 无线遥操 Franka FR3 的适配器。头显用本仓库 vr-app/ 的 Franka Teleop（协议同 arx-vr-analysis，magic 为 FRVR）。
"""
import argparse
import os
import signal
import sys
import threading

from .core import DEFAULT_CONFIG, __version__ as core_version, merge_config

FRANKA_DEFAULTS = {
    'ip': '192.168.3.1',
    'arm': 'right',
    'loop_rate': 200.0,
    'feedback_rate': 50.0,
    'stiffness': {'translational': 600.0, 'rotational': 40.0},
    'error_clip': {'translational': 0.05, 'rotational': 0.3},
    'posture': {'enabled': True, 'stiffness': 2.0,
                'target': [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]},
    'collision': {'torque': [20, 20, 18, 18, 16, 14, 12], 'force': [20, 20, 20, 25, 25, 25]},
    'max_recover': 3,
    'home_joints': [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
    'home_speed': 0.05,
    'gripper': {'enabled': True, 'mode': 'binary', 'grasp_below': 0.5, 'release_above': 1.0, 'deadband': 0.005,
                'close_below': 1.5, 'open_above': 3.5, 'speed': 0.1, 'force': 30.0},
}


def load_config(path):
    cfg = {}
    if path and os.path.exists(path):
        import yaml
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
    cfg = merge_config(DEFAULT_CONFIG, cfg)
    cfg['franka'] = merge_config(FRANKA_DEFAULTS, cfg.get('franka'))
    return cfg


def main(argv=None):
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(prog='franka_vr_bridge', description='Quest 3S 无线遥操 Franka FR3')
    ap.add_argument('--config', default=os.path.join(here, 'config.yaml'))
    ap.add_argument('--host', help='FR3 的 IP，覆盖 config.yaml 和环境变量 FRANKA_IP')
    ap.add_argument('--port', type=int, help='覆盖 config.yaml 里的 UDP 端口')
    ap.add_argument('--home', action='store_true', help='启动时先把 FR3 归位到 home_joints，再取零点（会动）')
    ap.add_argument('--yes', action='store_true', help='--home 时不再确认')
    ap.add_argument('--dry-run', action='store_true', help='不连机器人，只打印将要执行的指令（测头显和网络）')
    ap.add_argument('--csv', help='把每次发布的指令写入 CSV（延迟分析）')
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    if args.port:
        cfg['network']['port'] = args.port
    ip = args.host or os.environ.get('FRANKA_IP') or cfg['franka']['ip']
    log = lambda s: print(s, flush=True)
    log(f'franka_vr_bridge（core {core_version}）  配置：{args.config if os.path.exists(args.config) else "内置默认值"}')

    stop = threading.Event()

    def watch_signals():
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.set())

    from .runner import DryRunPrinter, Runner
    if args.dry_run:
        watch_signals()
        runner = Runner(cfg, DryRunPrinter(log=log), ros_version=0, log=log, csv_path=args.csv)
        runner.start()
        log('dry-run：不连机器人，Ctrl+C 退出')
        while not stop.is_set():
            stop.wait(0.2)
        runner.stop()
        return

    import franky
    import numpy as np

    from .franka import FrankaArm

    f = cfg['franka']
    log(f'连接 FR3 {ip} ...')
    robot = franky.Robot(ip)
    robot.recover_from_errors()
    robot.set_collision_behavior(torque_thresholds=f['collision']['torque'],
                                 force_thresholds=f['collision']['force'])
    robot.joint_acceleration_limit.set([5.0] * 7)

    gripper = None
    if f['gripper']['enabled']:
        try:
            gripper = franky.Gripper(ip)
            if gripper.max_width <= 0.0:
                log('夹爪没做过 homing（max_width = 0），本次不控夹爪')
                gripper = None
        except Exception as e:  # noqa: BLE001
            log(f'连不上夹爪（{e}），本次不控夹爪')
            gripper = None

    if args.home:
        q = np.asarray(robot.current_joint_state.position)
        target = np.asarray(f['home_joints'], dtype=float)
        log(f'归位到 home_joints，最大单关节转动 {np.degrees(np.max(np.abs(target - q))):.1f}°，'
            f'速度系数 {f["home_speed"]}')
        if not args.yes and input('确认周围无障碍、急停在手边，输入 yes 继续：').strip().lower() != 'yes':
            sys.exit('已取消')
        robot.move(franky.JointMotion(target.tolist(), relative_dynamics_factor=float(f['home_speed'])))

    watch_signals()   # 归位确认之后再接管 Ctrl+C，否则 input() 时按 Ctrl+C 退不出去
    arm = FrankaArm(robot, cfg, log=log, gripper=gripper)
    arm.set_zero_here()
    runner = Runner(cfg, arm.publish, ros_version=2, log=log, csv_path=args.csv)
    runner.start()
    log('头显（Franka Teleop，右手柄）：长按 B 1 秒开/关遥操，按住扳机跟随，握持键夹爪，A 回零点。Ctrl+C 退出')
    try:
        arm.run(runner, stop)
    finally:
        stop.set()
        runner.stop()
        log('已退出。FR3 停在当前位置')


if __name__ == '__main__':
    main()
