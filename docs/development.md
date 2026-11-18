# 开发

## 代码结构和来源

| 路径 | 说明 |
|---|---|
| `franka_vr_bridge/core.py` `runner.py` | 看门狗、限速、钳位、离合，原样取自 arx-vr-analysis（提交 `71376c4`） |
| `franka_vr_bridge/protocol.py` | 协议 v1，只把 magic 改成 `FRVR` |
| `franka_vr_bridge/franka.py` | FR3 端：franky `CartesianImpedanceTracker`、反馈、reflex 后重新起步、Franka Hand |
| `franka_vr_bridge/__main__.py` | 命令行入口、`FRANKA_DEFAULTS`（`config.yaml` 没写的项用它） |
| `vr-app/scripts/` | 头显应用。`arm_solver.gd` `link.gd` `panel_3d.gd` `hud.gd` 原样取自 arx-vr-analysis，`protocol.gd` 只改 magic，`main.gd` `teleop.gd` 是右手单手版 |
| `vr-app/tests/` | 头显离线测试：解算精度、状态机、协议与 Python 交叉校验（`tools/proto_xcheck.py`）、冒烟测试 |
| `tests/` | 适配器离线测试，用假的 franky 模块和假机器人 |

## 离线测试

不需要头显和机器人。

```bash
uv sync --frozen
uv run --frozen python -B -m unittest discover -s tests                 # 只测适配器（Mac 上也能跑，不装 franky）
GODOT=godot PYTHON=.venv/bin/python bash tools/run_vr_tests.sh          # 全部：适配器、头显状态机与解算、冒烟（需要 Godot 4.6.2）
```

dry-run 加模拟头显，测整条链路：

```bash
./start.sh --dry-run                                            # 终端 1
uv run --frozen python tools/fake_vr.py --host 127.0.0.1        # 终端 2：开启遥操、右手画圆、握持开合、关闭
```

## 同步到主机

代码在 Mac 上管理（`~/Documents/Code/franka-vr-teleop`），同步到 lab 主机运行，不要在 lab 上直接改。
lab 不需要头显工程，也不要覆盖它的 `.venv`：

```bash
rsync -avP --exclude .git --exclude __pycache__ --exclude .venv --exclude vr-app --exclude build \
  --exclude keystore --exclude .github \
  ~/Documents/Code/franka-vr-teleop/ lab:Documents/Code/franka/franka_vr_teleop/
ssh lab 'cd ~/Documents/Code/franka/franka_vr_teleop && ~/.local/bin/uv sync --frozen'   # 依赖有变化时
```

推到 GitHub 后，主机上直接 `git clone` / `git pull` 也行。

## 构建 APK

### GitHub Actions

推到 GitHub 后，每次 push 到 `main`、提 PR 或手动触发（Actions → Build Quest APK → Run workflow）都会：
下载 Godot 4.6.2 与 Android 导出模板、OpenXR Vendors 5.1.0 插件 → 跑全部离线测试 → 构建 APK → 上传为 Artifact。
推 `v*` 标签（如 `git tag v0.2.0 && git push --tags`）时额外发布到 Release。

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

产物在 `build/FrankaTeleop.apk`。缺 OpenXR 插件时自动下载（`tools/fetch_addons.sh`，核对 sha256）；第一次导出会生成
`vr-app/android/` 构建模板（约 1 GB，不进 git）并下载 Gradle 依赖。

界面文字改动后要重新生成字体子集，否则新字显示成方框：`uv run --with fonttools python tools/make_font.py 'NotoSansSC[wght].ttf'`（字体下载地址见文件头）。
语音提示用 `tools/make_voice.sh` 生成。

## 协议

协议 v1，UDP 单端口（默认 47800）承载发现、控制、状态：头显广播发现 → 适配器回 HELLO（名字、工作空间）→
头显以约 90 Hz 发 CTRL（两只手的相对位姿、夹爪值、按键状态、会话 id），适配器以 30 Hz 回 STAT（RTT、反馈、限幅、dry-run 标记）。
包格式见 `franka_vr_bridge/protocol.py`，头显端 `vr-app/scripts/protocol.gd` 和它逐字节一致（`tools/proto_xcheck.py` 交叉校验）。
