# franka-vr-teleop —— Quest 3S 无线遥操 Franka FR3

戴上 Quest 3S，用右手柄无线遥操 Franka FR3 和 Franka Hand。两部分：

- **头显应用 Franka Teleop**（`vr-app/`，Godot 4.6.2 + OpenXR）：读右手柄位姿和按键，显示 HUD、语音提示。
- **主机适配器 franka_vr_bridge**（Python，跑在接 FR3 的 Ubuntu 实时内核主机上）：看门狗、限速、工作空间钳位，
  用 franky 的笛卡尔阻抗跟踪器驱动 FR3。

```
Quest 3S（Franka Teleop）──CTRL 90 Hz / UDP 47800──▶ 主机 franka_vr_bridge
   HUD / 语音 / 振动        ◀──STAT 30 Hz──           看门狗 → 限速/钳位 200 Hz → CartesianImpedanceTracker
                                                    （libfranka 1 kHz 实时环）→ FR3 + Franka Hand
```

## 快速开始

第一次用先按 [docs/install.md](docs/install.md) 装好主机和头显，下面只是命令清单。

**主机，只做一次**（Ubuntu 22.04 + PREEMPT_RT 内核，仓库放在任意目录）：

```bash
uv sync --frozen                         # Python 3.11 环境，franky 的 wheel 在 wheels/ 里，不用连 GitHub
sudo bash host/setup_rt_sudo.sh          # realtime 组、CPU governor、cpu_dma_latency；之后注销重新登录
host/net_up.sh create                    # 新建连 FR3 的网口配置（在主机桌面的终端里运行）
sudo bash host/setup_firewall_sudo.sh    # ufw 放行 FR3 的 UDP 和头显的 UDP 47800
bash host/check_host.sh                  # 只读自检
./start.sh --dry-run                     # 不连机器人，测头显和网络
```

**头显，只做一次**：开开发者模式，USB 连电脑，`bash tools/install_apk.sh`（APK 从 GitHub Actions 下载或本地构建），
在头显里设好静止边界。

**每次使用**：

1. FR3 开机，主机桌面终端运行 `host/net_up.sh`；Desk 里解锁关节、激活 FCI。
2. 主机上 `./start.sh`（加 `--home` 先慢速归位到 ready 姿态）。
3. 面向机器人打开头显里的 Franka Teleop，HUD 链路和反馈都显示“正常”后，**长按 B** 开启遥操，**按住扳机**跟随。
4. 结束：长按 B 关闭遥操，主机上 Ctrl+C。

操作说明、安全行为、调参和排障见 [docs/usage.md](docs/usage.md)。

## 文档

| 文档 | 内容 |
|---|---|
| [docs/install.md](docs/install.md) | 主机：实时内核、uv 环境、实时权限、FR3 网络、防火墙、自检；头显：开发者模式、装 APK、边界和网络 |
| [docs/usage.md](docs/usage.md) | 每次上机的步骤、手柄操作、HUD、坐标与零点、安全行为、`config.yaml` 调参、第一次上真机、排障 |
| [docs/development.md](docs/development.md) | 代码结构和来源、离线测试、构建 APK（GitHub Actions / 本地）、签名、从 Mac 同步到主机 |

## 目录

| 路径 | 说明 |
|---|---|
| `start.sh` | 启动适配器（仓库自带的 uv 环境，套 `host/low_latency.sh`） |
| `config.yaml` | 适配器的硬上限：速度、工作空间、阻抗、碰撞阈值、夹爪。头显改不了 |
| `franka_vr_bridge/` | 适配器源码 |
| `host/` | 主机配置：`host.env` 参数（网口、IP、端口），`setup_rt_sudo.sh`、`setup_firewall_sudo.sh` 要 sudo，`net_up.sh` 切网口，`check_host.sh` 自检，`low_latency.sh` |
| `pyproject.toml` `uv.lock` `wheels/` | Python 3.11 环境：franky-control 2.0.0（自带 libfranka 0.17.0，对应机器人系统 5.7.2–5.8.x）、numpy、PyYAML |
| `vr-app/` | 头显应用的 Godot 工程，`vr-app/tests/` 是它的离线测试 |
| `tests/` | 适配器离线测试（不需要 franky 和机器人） |
| `tools/` | 头显：`install_apk.sh` 安装、`build_apk.sh` 构建、`quest_awake.sh` 关佩戴检测；测试：`fake_vr.py` 模拟头显、`run_vr_tests.sh` 全部离线测试、`gripper_latency.py` 测夹爪延迟；`analyze_workspace.py` 分析 `--record` 录的工作空间 |
| `.github/workflows/build-apk.yml` | GitHub Actions：离线测试 + 构建 APK |

## 版本要求

| 组件 | 版本 |
|---|---|
| FR3 机器人系统 | 5.7.2–5.8.x（Robot Server 9）。5.9.0 及以上要换 wheel，见 [docs/install.md](docs/install.md#机器人系统-59-及以上) |
| 主机 | Ubuntu 22.04，PREEMPT_RT 内核，x86_64（wheel 要求 glibc ≥ 2.34） |
| 头显 | Meta Quest 3S（Quest 3 应该也能用，没测过） |
| 构建 APK | Godot 4.6.2、OpenXR Vendors 5.1.0、JDK 17、Android SDK 35 / NDK 28.1（只有本地构建需要） |

协议、看门狗、限速、离合解算沿用 arx-vr-analysis（Quest 遥操 ARX X7s），包头 magic 改成 `FRVR`（ARX 是 `AXVR`），
两套应用和适配器互不相认，不会连错机器人。
