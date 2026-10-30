# franka-vr-teleop —— Quest 3S 无线遥操 Franka FR3

两部分：Quest 3S 上的头显应用 **Franka Teleop**（`vr-app/`，Godot 4.6.2 + OpenXR），和 lab 主机上的适配器
`franka_vr_bridge`（Python，franky 的笛卡尔阻抗跟踪器驱动 FR3）。两边走局域网 UDP。只用右手柄，没有底盘 / 升降。

```
Quest 3S（Franka Teleop）──CTRL 90Hz / UDP 47800──▶ lab 主机 franka_vr_bridge
   HUD / 语音 / 振动        ◀──STAT 30Hz──           看门狗 → 限速/钳位 200 Hz → CartesianImpedanceTracker
                                                    （libfranka 1 kHz 实时环）→ FR3 + Franka Hand
```

由 [arx-vr-analysis](../arx-vr-analysis)（Quest 遥操 ARX X7s）精简而来：协议、看门狗、限速、离合解算都沿用，
包头 magic 改成 `FRVR`（ARX 是 `AXVR`），两套应用和适配器互不相认，不会连错机器人。

## 目录

| 路径 | 说明 |
|---|---|
| `vr-app/` | 头显应用（Godot 工程）。`scripts/arm_solver.gd` `link.gd` `panel_3d.gd` `hud.gd` 原样取自 arx-vr-analysis，`protocol.gd` 只改 magic，`main.gd` `teleop.gd` 是右手单手版 |
| `vr-app/tests/` | 头显离线测试：解算精度、状态机、协议与 Python 交叉校验、冒烟测试 |
| `franka_vr_bridge/` | 适配器。`core.py` `runner.py` 原样取自 arx-vr-analysis（提交 `71376c4`），`protocol.py` 只改 magic；`franka.py` 是 FR3 端 |
| `config.yaml` | 适配器的硬上限（速度、工作空间、阻抗、夹爪），头显改不了 |
| `start.sh` | lab 上启动适配器（`franky_ctl` 的环境，套 `low_latency.sh`） |
| `tests/` | 适配器离线测试 |
| `tools/` | `build_apk.sh` 构建、`install_apk.sh` 安装、`fetch_addons.sh` 下载 OpenXR 插件、`run_vr_tests.sh` 全部离线测试、`fake_vr.py` 模拟头显、`quest_awake.sh` 关闭佩戴检测、`make_voice.sh` / `make_font.py` 生成语音和字体子集 |
| `.github/workflows/build-apk.yml` | GitHub Actions：测试 + 构建 APK |

## 头显操作（只用右手柄）

| 右手柄 | 作用 |
|---|---|
| 长按 B 1 秒 | 遥操开 / 关（HUD 显示进度）。**第一次开启时 FR3 慢速回零点** |
| 按住食指扳机 | 跟随：手柄怎么动，末端就怎么动；松开冻结（离合，可以把手放回舒服的位置再按） |
| 握持键 | 夹爪：按下超过 70% 抓取（30 N），松开到 30% 以下张开 |
| A | 慢速回零点 |
| 短按右摇杆 | 打开 / 关闭设置面板（打开时遥操暂停；右手射线指着、扣扳机点击） |
| 长按右摇杆 1 秒 | 以右手柄当前指向为“前方”重新对齐 |
| 戴着头显长按 Meta 键 | 重置视野，并按头显朝向重新对齐 |

- 启动时按头显朝向自动对齐“前方”：**面向机器人正前方**打开应用；之后转头不影响手臂。站位变了就重新对齐。
- 以下情况遥操立即暂停（手臂、夹爪冻结），恢复后**要先松开扳机再按**才会跟随：打开设置面板、按 Meta 键、
  摘下头显、应用失去焦点。开启遥操时扳机已经按着，也要先松开。
- 手柄上的小球：灰 = 保持，绿 = 跟随，黄 = 回零点。
- HUD（视野下方三行）：遥操状态 / 手臂与夹爪 / 链路（RTT、丢包、适配器有无 FR3 反馈、dry-run）。
- 语音：遥操开启 / 关闭、暂停 / 继续、已连接、连接断开、已对齐前方。
- 设置面板：连接（自动发现列表、手动 IP 数字键盘）、操控（位置 / 旋转缩放、发送频率、平滑、夹爪映射）、显示提示、对齐。
  设置存在头显的 `/sdcard/Android/data/com.infinity4b.frankateleop/files/settings.cfg`。

## 安装到 Quest 3S

只需第一次做 1–3 步。

1. **打开开发者模式**：手机上的 Meta Horizon 应用 → 设备 → 选这台头显 → 头显设置 → 开发者模式 → 打开。
   需要 Meta 账号先在 <https://developers.meta.com/horizon/> 创建或加入一个组织（免费）。打开后重启头显。
2. **装 adb**：Mac `brew install --cask android-platform-tools`；Ubuntu `sudo apt install adb`。
3. **USB 连接并授权**：用能传数据的 USB-C 线连电脑和头显，戴上头显，在“允许 USB 调试”弹窗里勾选“始终允许”后确认。
   `adb devices` 能看到一行 `device` 就行。
4. **拿到 APK**，任选其一：
   - GitHub：仓库的 Actions → 最近一次成功的 “Build Quest APK” → Artifacts 里下载 `FrankaTeleop`（zip，解压得到 APK）；
     打了 `v*` 标签的版本在 Releases 里直接下载 APK。
   - 本地构建：`bash tools/build_apk.sh`，产物在 `build/FrankaTeleop.apk`（见下节）。
5. **安装并启动**：
   ```bash
   bash tools/install_apk.sh                                   # 安装 build/FrankaTeleop.apk 并启动
   APK=~/Downloads/FrankaTeleop.apk bash tools/install_apk.sh  # 安装下载的 APK
   adb install -r FrankaTeleop.apk                             # 不用脚本也行
   ```
   签名不同（比如本地构建的换成 GitHub 调试版）时不能覆盖安装，脚本会先卸载旧版再装，头显里的设置会清空。
6. **在头显里打开**：应用库右上角的筛选选 “未知来源”（Unknown Sources），点 **Franka Teleop**。

之后可以无线安装：USB 连着时执行一次 `bash tools/install_apk.sh --wifi <头显 IP>`，以后拔线也能用同一条命令。
头显挂在脖子上用时，`bash tools/install_apk.sh --awake`（或 `bash tools/quest_awake.sh`）关闭佩戴检测，头显重启后失效；
关闭后“摘头显即暂停”的保护没了，放下手柄前先松开扳机或长按 B 关闭遥操。
看日志：`bash tools/install_apk.sh --log`。

不想用命令行也可以用 SideQuest 之类的工具把 APK 拖进去安装（同样要先开开发者模式）。

## 构建 APK

### GitHub Actions

推到 GitHub 后，每次 push 到 `main`、提 PR 或手动触发（Actions → Build Quest APK → Run workflow）都会：
下载 Godot 4.6.2 与 Android 导出模板、OpenXR Vendors 5.1.0 插件 → 跑全部离线测试 → 构建 APK → 上传为 Artifact。
推 `v*` 标签（如 `git tag v0.1.0 && git push --tags`）时额外发布到 Release。

签名：不配置时构建调试版（每次签名不同，换版本要先卸载）。要固定签名，在仓库 Settings → Secrets and variables →
Actions 里加三个 secret，用本地 `build_apk.sh` 生成的 keystore：

```bash
gh secret set FRANKA_KEYSTORE_BASE64 < <(base64 -i keystore/franka-teleop.keystore)
gh secret set FRANKA_KEYSTORE_USER --body frankateleop
gh secret set FRANKA_KEYSTORE_PASS < keystore/franka-teleop.pass
```

这样本地构建和 GitHub 构建的 APK 可以互相覆盖安装。`keystore/` 不进 git，**请另外备份**，丢了就只能卸载重装。

### 本地

需要 Godot 4.6.2（含导出模板）、JDK 17、Android SDK（build-tools 35.0.1、platform 35、NDK 28.1.13356709），
Godot 编辑器设置里填好 Android SDK 和 Java 路径（`build_apk.sh` 发现为空时会自动填）。

```bash
bash tools/build_apk.sh          # 发布版，第一次会生成 keystore/franka-teleop.keystore
bash tools/build_apk.sh debug    # 调试版，用 Godot 的调试签名
```

缺 OpenXR 插件时自动下载（`tools/fetch_addons.sh`，核对 sha256）；第一次导出会生成 `vr-app/android/` 构建模板（约 1 GB，不进 git）
并下载 Gradle 依赖。界面文字改动后要重新生成字体子集（`tools/make_font.py`，用法见文件头），否则新字显示成方框。

## 适配器（lab 主机）

### 坐标与零点

- 头显发的是相对零点的末端位姿，x 前 / y 左 / z 上，和 FR3 基座系同向；姿态是基座系下的增量：
  `p = p0 + (x, y, z)`，`R = Rzyx(roll, pitch, yaw) · R0`。手柄在世界里怎么转，夹爪就怎么转。
- **零点 = 适配器启动时的末端位姿**（加 `--home` 时先归位到 ready 姿态再取）。按 A、第一次开启遥操时都慢速回到这里。

### 部署

代码在 Mac 上管理，同步到 lab 运行（lab 不需要头显工程）：

```bash
rsync -avP --exclude .git --exclude __pycache__ --exclude vr-app --exclude build --exclude keystore --exclude .github \
  ~/Documents/Code/franka-vr-teleop/ lab:Documents/Code/franka/franka_vr_teleop/
```

lab 上用 `~/Documents/Code/franka/franky_ctl` 的 uv 环境（Python 3.11、franky 2.0.0、numpy、PyYAML），不另建环境。
防火墙：lab 的 ufw 默认拒绝入站，要放行无线网卡上的 UDP 47800（`sudo bash ~/sudo_scripts/franka_vr_ufw_sudo.sh`）。

### 使用

1. FR3：`net_up.sh` 切网口、Desk 里解锁并激活 FCI（或 `franky_ctl/enable_fci.py`）。
2. 头显和 lab 主机连同一个局域网（lab 的无线网卡 `wlp129s0f0`）。路由器隔离广播时在设置面板里手动填 lab 的 IP。
3. lab 上启动适配器：
   ```bash
   cd ~/Documents/Code/franka/franka_vr_teleop
   ./start.sh --home      # 先慢速归位到 ready 再取零点（会动，要输入 yes）
   ./start.sh             # 以当前位置为零点
   ./start.sh --dry-run   # 不连机器人，只测头显和网络
   ```
4. 面向机器人打开 Franka Teleop，HUD 链路行显示“正常”、反馈“正常”后，长按 B 开启遥操。
5. 退出：长按 B 关闭遥操，再在 lab 上 Ctrl+C。FR3 停在当前位置。

### 安全行为

| 情况 | FR3 |
|---|---|
| 遥操关闭 / 头显没连 | 停在最后的目标（启动时就是当前位置） |
| 0.12 s 没收到包、面板打开、摘下头显 | 停在当前指令 |
| 1 s 没收到包 | 停止发布，停在最后的目标 |
| 碰撞触发 reflex | 自动清错，从实际位置重新起步（不按旧指令冲回去）；一分钟超过 3 次就退出 |
| 目标离实际位置太远 | 阻抗误差被限在 5 cm / 0.3 rad 以内，弹簧力有上限 |
| Ctrl+C | 跟踪器平滑停止，FR3 停住 |

速度上限 0.5 m/s、1.5 rad/s，回零点 0.15 m/s。工作空间默认收得很紧（z 下限约为基座上方 0.19 m，碰不到和基座同高的桌面），
抓桌面上的东西前先按实际桌面高度放宽 `config.yaml` 的 `arm.workspace`。

## 测试

```bash
GODOT=godot PYTHON=python3 bash tools/run_vr_tests.sh   # 全部离线测试：适配器、头显状态机与解算、冒烟（需要 numpy、PyYAML）
python3 -m unittest discover -s tests                   # 只测适配器
python3 -m franka_vr_bridge --dry-run &                  # 适配器 dry-run
python3 tools/fake_vr.py --host 127.0.0.1                # 模拟头显：开启遥操、右手画圆、握持开合、关闭
```

真机上第一次先用 `fake_vr.py` 代替头显（在 lab 本机连 127.0.0.1，不经过防火墙）：

```bash
./start.sh                                                                 # 终端 1
uv run --project ../franky_ctl python tools/fake_vr.py --host 127.0.0.1 --scale 0.3   # 终端 2
```

FR3 应该先回零点（就在原地），再画一个直径 6 cm、上下起伏 ±1.5 cm 的小圈，同时末端俯仰 ±5°，夹爪开合几次，6 s 后停住。
