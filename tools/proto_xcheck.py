#!/usr/bin/env python3
"""协议交叉校验（vr-app/tests/run_tests.gd 调用）：用 Python 实现编码 / 解码，对照 GDScript 实现。

  proto_xcheck.py hello              打印一个 HELLO 包的 hex
  proto_xcheck.py stat               打印一个 STAT 包的 hex
  proto_xcheck.py ctrl <hex>         解码 CTRL，打印 JSON
"""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from franka_vr_bridge import protocol as P  # noqa: E402


def main():
    cmd = sys.argv[1]
    if cmd == 'hello':
        print(P.pack_hello(0xDEADBEEF, 47800, 2, P.OWNER_OTHER, 'Franka-测试', '1.0.0', (1.0, 4.0, 0.0, 20.0),
                          [i * 0.1 - 1.0 for i in range(24)]).hex())
    elif cmd == 'stat':
        print(P.pack_stat({'session': 0x01020304, 'echo_seq': 777, 'echo_t_vr_us': 123456789012, 'hold_us': 4321,
                           'loss': 0.125, 'status': P.S_LEFT_FB | P.S_PUBLISHING, 'watchdog': P.WD_HOLD,
                           'owner': P.OWNER_YOU, 'height_fb': 7.5, 'height_cmd': 8.0, 'rx_rate': 90.0}).hex())
    elif cmd == 'ctrl':
        print(json.dumps(P.unpack_ctrl(bytes.fromhex(sys.argv[2]))))


if __name__ == '__main__':
    main()
