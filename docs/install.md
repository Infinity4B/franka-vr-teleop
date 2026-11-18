# 安装

分主机和头显两部分，都只需做一次。实验室那台主机（`ssh lab`，`~/Documents/Code/franka/franka_vr_teleop`）已经全部装好，
换主机或重装系统时才需要从头走一遍。

## 主机

### 0. 前提

- **Ubuntu 22.04，x86_64，PREEMPT_RT 内核。** 内核按 Franka 官方文档（<https://frankarobotics.github.io/docs/>，
  libfranka → Installation on Linux → Setting up the real-time kernel）编译安装。`cat /sys/kernel/realtime` 输出 1 才是实时内核。
  - 有 NVIDIA 显卡时，DKMS 默认拒绝在 RT 内核上编译驱动，要在 `/etc/dkms/framework.conf` 里加
    `export IGNORE_PREEMPT_RT_PRESENCE=1`。
  - 新平台（如 Arrow Lake）上原版内核可能有驱动问题，造成实时延迟尖峰。lab 主机遇到过的问题和修法记在
    lab 的 `~/Documents/Code/franka/README.md`。
- **FR3 机器人系统版本**：本仓库自带的 franky wheel 内含 libfranka 0.17.0，对应系统 5.7.2–5.8.x（Robot Server 9）。
  版本在 Desk 的设置页里能看到。5.9.0 及以上见[后面](#机器人系统-59-及以上)。
- 主机有两个网口：一个用网线直连 FR3 的 Control 柜，一个（一般是无线网卡）和头显连同一个局域网。

### 1. 装 uv、拿到代码

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh        # 装完重开终端
git clone <仓库地址> franka-vr-teleop && cd franka-vr-teleop
```

还没推到 GitHub 时，从 Mac 同步过去（见 [development.md](development.md#同步到主机)）。仓库可以放在任意目录。

### 2. Python 环境

```bash
uv sync --frozen
```

建出 `.venv/`（Python 3.11、franky-control 2.0.0、numpy、PyYAML、websockets）。franky 的 wheel 在 `wheels/` 里，
不用连 GitHub；其它包从 PyPI 或你配置的镜像下载。没有 Python 3.11 时 uv 会自动下载，从 GitHub 下载慢的话
先 `sudo apt install python3.11`（deadsnakes PPA），或设 `UV_PYTHON_INSTALL_MIRROR`。

`--frozen` 表示严格按 `uv.lock` 安装、不改写它（`uv.lock` 里记的是 pypi.org，配了镜像的机器不加 `--frozen` 会把整个
lock 改写成镜像地址）。`start.sh` 也用 `--frozen`。改了 `pyproject.toml` 的依赖后才需要 `uv lock`。

### 3. 实时权限和 CPU 调优

```bash
sudo bash host/setup_rt_sudo.sh
```

做三件事（可重复运行，`undo` 参数撤销后两项）：

1. 建 `realtime` 组并把你加进去，组内 rtprio 99、memlock 100 MB（已有 `@realtime` 配置就不重复写）；
2. 装一个 systemd 服务，RT 内核开机后把 CPU governor 设为 performance；
3. 让 realtime 组可以写 `/dev/cpu_dma_latency`。`start.sh` 通过 `host/low_latency.sh` 在运行期间把它设成 0，禁止 CPU 深度休眠。

**第一次加入 realtime 组后要注销重新登录**（ssh 要断开重连）才生效。

### 4. 连 FR3 的网络

1. FR3 的 Control 柜（不是手臂底座）用网线接主机网口。
2. 在 Desk（浏览器打开机器人 IP）→ Settings → Network 里给 Shop Floor 网络设一个静态 IP。出厂默认是 `172.16.0.2`；
   lab 的两台 FR3 都改成了 `192.168.3.1`。
3. 改 `host/host.env`：`FRANKA_IFACE`（网口名，`ip -br link` 查看）、`FRANKA_IP`、`HOST_CIDR`（本机地址，和机器人同网段）。
   `config.yaml` 里的 `franka.ip` 也要一致（或用环境变量 `FRANKA_IP` / 启动参数 `--host`）。
4. **在主机桌面上的终端里**运行（ssh 里运行 nmcli 会报 `Not authorized to control networking`）：
   ```bash
   host/net_up.sh create     # 新建 NetworkManager 连接 franka_control：静态 IP、不自动连接
   host/net_up.sh            # 切过去，并 ping 一下 FR3
   ```

不知道机器人 IP 时：`ping -I <网口> <候选IP>` 后看 `ip neigh show dev <网口>`，有 MAC（Franka 的是 `00:30:59:` 开头）就是对的。

### 5. 防火墙

Ubuntu 的 ufw 开启时默认拒绝入站，会挡住两类包：FR3 发回的 UDP 状态包（症状是 franky 报 `UDP receive: Timeout`，
TCP 能连上），和头显发给适配器的 UDP 47800（症状是头显发现不了主机）。

```bash
sudo bash host/setup_firewall_sudo.sh
```

只放行 `FRANKA_IFACE` 上来自 `FRANKA_IP` 的 UDP，和无线网卡上来自本网段的 UDP 47800。要先连上和头显同一个 Wi-Fi，
脚本才知道本网段是什么；换了 Wi-Fi 要重新运行。ufw 没开时脚本什么都不做。`undo` 撤销。

### 6. 自检

```bash
bash host/check_host.sh
```

只读，检查实时内核、realtime 组和 limits、governor、`cpu_dma_latency`、`.venv` 里的 franky、ufw、到 FR3 的网口 / 路由 / ping /
TCP 1337，以及无线网卡的 IP（头显发现不了时手动填这个）。FAIL 项后面写了怎么修。

最后跑一次 dry-run，确认环境能启动（不连机器人）：

```bash
./start.sh --dry-run                                         # 终端 1
uv run --frozen python tools/fake_vr.py --host 127.0.0.1     # 终端 2：模拟头显，终端 1 应打印“控制权交给 127.0.0.1…”和右手的指令
```

### 机器人系统 5.9 及以上

Robot Server 10 要换内含 libfranka 0.21.x 的 wheel：

1. 从 <https://timschneider42.github.io/franky/whl/by-robot-server-version/10/franky-control/> 下载
   `franky_control-2.0.0+libfranka.0.21.2-cp311-cp311-manylinux_2_34_x86_64.whl`，核对页面链接上的 sha256，放进 `wheels/`。
   主机直连 GitHub 慢的话在别的机器上下好再拷过去。
2. 改 `pyproject.toml`：依赖里的版本号改成 `2.0.0+libfranka.0.21.2`，`[tool.uv.sources]` 的路径改成新文件名。
3. `uv lock && uv sync --frozen`，再 `bash host/check_host.sh` 确认版本。

版本不对时 franky 连机器人会报版本不兼容（`Incompatible library version`）。

## 头显（Quest 3S）

第 1–3 步只在第一次做。电脑可以是 Mac 或主机本身，只要能用 adb。

1. **打开开发者模式**：手机上的 Meta Horizon 应用 → 设备 → 选这台头显 → 头显设置 → 开发者模式 → 打开。
   Meta 账号要先在 <https://developers.meta.com/horizon/> 创建或加入一个组织（免费）。打开后重启头显。
2. **装 adb**：Mac `brew install --cask android-platform-tools`；Ubuntu `sudo apt install adb`。
3. **USB 连接并授权**：用能传数据的 USB-C 线连电脑和头显，戴上头显，在“允许 USB 调试”弹窗里勾选“始终允许”后确认。
   `adb devices` 能看到一行 `device` 就行。
4. **拿到 APK**，任选其一：
   - GitHub：仓库的 Actions → 最近一次成功的 “Build Quest APK” → Artifacts 里下载 `FrankaTeleop`（zip，解压得到 APK）；
     打了 `v*` 标签的版本在 Releases 里直接下载。
   - Mac 上的仓库里已经构建过的 `build/FrankaTeleop.apk`（不进 git）。
   - 本地构建：`bash tools/build_apk.sh`，见 [development.md](development.md#构建-apk)。
5. **安装并启动**：
   ```bash
   bash tools/install_apk.sh                                   # 安装 build/FrankaTeleop.apk 并启动
   APK=~/Downloads/FrankaTeleop.apk bash tools/install_apk.sh  # 安装下载的 APK
   adb install -r FrankaTeleop.apk                             # 不用脚本也行
   ```
   签名不同（比如本地构建的换成 GitHub 调试版）时不能覆盖安装，脚本会先卸载旧版再装，头显里的设置会清空。
6. **在头显里打开**：应用库右上角的筛选选“未知来源”（Unknown Sources），点 **Franka Teleop**。

之后可以无线安装：USB 连着时执行一次 `bash tools/install_apk.sh --wifi <头显 IP>`，以后拔线也能用同一条命令。
看日志：`bash tools/install_apk.sh --log`。不想用命令行也可以用 SideQuest 之类的工具安装（同样要先开开发者模式）。

### 头显的系统设置

- **设好边界**（静止边界即可）。没设边界时应用会一直“失去焦点”，遥操一直处于暂停。恢复出厂后要重新设。
- **连上和主机同一个 Wi-Fi。** 路由器隔离广播时自动发现不了主机，在应用的设置面板里手动填主机无线网卡的 IP
  （`host/check_host.sh` 会打印）。
- 国内网络下头显时间不同步、Wi-Fi 一直显示“无法连接互联网”时，用 adb 换成国内的 NTP 和联网检测地址：
  ```bash
  adb shell settings put global ntp_server ntp.aliyun.com
  adb shell settings put global captive_portal_http_url http://connect.rom.miui.com/generate_204
  adb shell settings put global captive_portal_https_url https://connect.rom.miui.com/generate_204
  ```
  激活新头显和系统更新需要能访问 Meta 的网络。
- 头显挂在脖子上用时，`bash tools/quest_awake.sh`（或 `install_apk.sh --awake`）关闭佩戴检测，头显重启后失效。
  关闭后“摘头显即暂停”的保护没了，放下手柄前先松开扳机或长按 B 关闭遥操。
