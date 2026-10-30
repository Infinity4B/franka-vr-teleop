# franka-vr-teleop —— Quest 3S 无线遥操 Franka FR3

头显沿用 [arx-vr-analysis](../arx-vr-analysis) 的 `ArxTeleop.apk`，协议不变；这里只换机器人端：
lab 主机上的适配器收头显的 UDP 指令，经过同一套看门狗、限速、工作空间钳位后，交给 franky 的笛卡尔阻抗跟踪器驱动 FR3。

```
Quest 3S（ArxTeleop.apk）──CTRL 90Hz / UDP 47800──▶ lab 主机 franka_vr_bridge
   HUD / 语音 / 振动       ◀──STAT 30Hz──           看门狗 → 限速/钳位 200 Hz → CartesianImpedanceTracker
                                                    （libfranka 1 kHz 实时环）→ FR3 + Franka Hand
```

## 文件

| 路径 | 说明 |
|---|---|
| `franka_vr_bridge/protocol.py` `core.py` `runner.py` | 原样取自 arx-vr-analysis（`teleop/robot-bridge/arx_vr_bridge`，提交 `71376c4`）：协议、控制权、看门狗、限速插值、UDP 收发 |
| `franka_vr_bridge/franka.py` | Franka 端：零点换算、笛卡尔阻抗跟踪、夹爪、反馈、掉出控制后的恢复 |
| `franka_vr_bridge/__main__.py` | 入口，Franka 相关配置的默认值 |
| `config.yaml` | 机器人端硬上限（头显改不了） |
| `start.sh` | lab 上的启动脚本（用 `franky_ctl` 的环境，套 `low_latency.sh`） |
| `tools/fake_vr.py` | 模拟头显，不戴头显测网络和适配器 |
| `tests/` | 离线测试：`test_core.py`（取自 arx-vr-analysis）、`test_franka.py` |

## 坐标与零点

- 头显发的是相对零点的末端位姿，x 前 / y 左 / z 上，和 FR3 基座系同向；姿态是基座系下的增量：
  `p = p0 + (x, y, z)`，`R = Rzyx(roll, pitch, yaw) · R0`。手柄在世界里怎么转，夹爪就怎么转。
- **零点 = 适配器启动时的末端位姿**（加 `--home` 时先归位到 ready 姿态再取）。头显上按 A（右臂）复位、
  第一次开闸自动归零，都是慢速回到这里。
- 只用一只手柄控制 FR3（`config.yaml` 的 `franka.arm`，默认右手）。另一只手柄的动作会被忽略。
- 头显里的“底盘、升降”不起作用。

## 部署（lab）

代码在 Mac 上管理，同步到 lab 运行：

```bash
rsync -avP --exclude .git --exclude __pycache__ ~/Documents/Code/franka-vr-teleop/ lab:Documents/Code/franka/franka_vr_teleop/
```

lab 上用 `~/Documents/Code/franka/franky_ctl` 的 uv 环境（Python 3.11、franky 2.0.0、numpy、PyYAML），不另建环境。

防火墙：lab 的 ufw 默认拒绝入站，要放行无线网卡上的 UDP 47800（`~/sudo_scripts/franka_vr_ufw_sudo.sh`）。

## 使用

1. FR3：`net_up.sh` 切网口、Desk 里解锁并激活 FCI（或 `franky_ctl/enable_fci.py`）。
2. 头显和 lab 主机连同一个局域网（lab 的无线网卡 `wlp129s0f0`）。路由器隔离广播时在头显面板里手动填 lab 的 IP。
3. lab 上启动适配器：
   ```bash
   cd ~/Documents/Code/franka/franka_vr_teleop
   ./start.sh --home      # 先慢速归位到 ready 再取零点（会动，要输入 yes）
   ./start.sh             # 以当前位置为零点
   ./start.sh --dry-run   # 不连机器人，只测头显和网络
   ```
4. 头显打开 ARX Teleop，发现列表里选 `Franka-FR3`。对齐、解锁、开闸、跟随的操作和 ARX 一样，见
   arx-vr-analysis 的 `teleop/docs/使用说明.md` 第 3 节：

   | 操作 | 作用 |
   |---|---|
   | 右手柄指向机器人正前方，长按左手菜单键 1 秒 | 对齐“前方” |
   | A+B+X+Y 同时按 | 解锁 |
   | 双摇杆同时按下 | 总闸门开/关；第一次开启时 FR3 慢速回零点 |
   | 右手食指扳机 | 按住跟随，松开冻结（离合） |
   | 右手握持键 | 按下过 70% 抓取（30 N），松开到 30% 以下张开 |
   | A | 慢速回零点 |

5. 退出：关闸，再在 lab 上 Ctrl+C。FR3 停在当前位置。

## 安全行为

| 情况 | FR3 |
|---|---|
| 未解锁 / 闸门关 / 头显没连 | 停在最后的目标（启动时就是当前位置） |
| 0.12 s 没收到包、面板打开、摘下头显 | 停在当前指令 |
| 1 s 没收到包 | 停止发布，停在最后的目标 |
| 碰撞触发 reflex | 自动清错，从实际位置重新起步（不按旧指令冲回去）；一分钟超过 3 次就退出 |
| 目标离实际位置太远 | 阻抗误差被限在 5 cm / 0.3 rad 以内，弹簧力有上限 |
| Ctrl+C | 跟踪器平滑停止，FR3 停住 |

速度上限 0.5 m/s、1.5 rad/s，复位 0.15 m/s。工作空间默认收得很紧（z 下限约为基座上方 0.19 m，碰不到和基座同高的桌面），
抓桌面上的东西前先按实际桌面高度放宽 `config.yaml` 的 `arm.workspace`。

## 测试

```bash
python3 -m unittest discover -s tests          # 不需要 franky 和机器人（需要 numpy、PyYAML）
python3 -m franka_vr_bridge --dry-run &         # 适配器 dry-run
python3 tools/fake_vr.py --host 127.0.0.1       # 模拟头显：解锁、开闸、右手画圆、握持开合、关闸
```

真机上第一次先用 `fake_vr.py` 代替头显（在 lab 本机连 127.0.0.1，不经过防火墙）：

```bash
./start.sh                                                                 # 终端 1
uv run --project ../franky_ctl python tools/fake_vr.py --host 127.0.0.1 --scale 0.3   # 终端 2
```

FR3 应该先回零点（就在原地），再画一个直径 6 cm、上下起伏 ±1.5 cm 的小圈，同时末端俯仰 ±5°，夹爪开合几次，6 s 后停住。
