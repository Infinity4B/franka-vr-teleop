# 使用

前提：主机和头显都按 [install.md](install.md) 装好。

## 每次上机

1. **FR3**：Control 柜开机，等 Desk 能打开（约 1 分钟）。主机**桌面终端**里运行 `host/net_up.sh`（切网口并 ping FR3）。
2. **Desk**（浏览器打开 `https://192.168.3.1`）：解锁关节，激活 FCI。
3. **主机**：
   ```bash
   cd <仓库目录>
   bash host/check_host.sh   # 可选，出问题时先跑这个
   ./start.sh --home         # 先慢速归位到 ready 姿态再取零点（会动，要输入 yes）
   ./start.sh                # 或者：以当前位置为零点
   ```
   其它参数：`--host IP` 换机器人 IP，`--csv out.csv` 记录每次发布的指令，`--dry-run` 不连机器人。
4. **头显**：**面向机器人正前方**打开 Franka Teleop。HUD 第三行链路显示“正常”、反馈显示“正常”后，长按 B 开启遥操，
   第一次开启时 FR3 会慢速回零点。然后按住扳机，手怎么动，夹爪就怎么动。
5. **结束**：长按 B 关闭遥操，主机上 Ctrl+C，FR3 停在当前位置。Desk 里锁上关节。

站在机器人旁边的人手放在急停（外部使能按钮）上。

## 手柄操作（只用右手柄）

| 右手柄 | 作用 |
|---|---|
| 长按 B 1 秒 | 遥操开 / 关（HUD 显示进度）。**第一次开启时 FR3 慢速回零点** |
| 按住食指扳机 | 跟随：手柄怎么动，末端就怎么动；松开冻结（离合，可以把手放回舒服的位置再按） |
| 握持键 | 夹爪：按一下（按过 70%）闭合抓取（30 N），再按一下张开。上一个动作没做完时按的那下会等它做完再执行。设置面板里关掉“夹爪：按一下合、再按一下开”后，改为按住抓、松开张开 |
| A | 慢速回零点 |
| 短按右摇杆 | 打开 / 关闭设置面板（打开时遥操暂停；右手射线指着、扣扳机点击） |
| 长按右摇杆 1 秒 | 以右手柄当前指向为“前方”重新对齐 |
| 戴着头显长按 Meta 键 | 重置视野，并按头显朝向重新对齐 |

- 启动时按头显朝向自动对齐“前方”，之后转头不影响手臂。站位变了就重新对齐。
- 以下情况遥操立即暂停（手臂、夹爪冻结）：打开设置面板、按 Meta 键、摘下头显、应用失去焦点。
  恢复后**要先松开扳机再按**才会跟随。开启遥操时扳机已经按着，也要先松开。
- 手柄上的小球：灰 = 保持，绿 = 跟随，黄 = 回零点。
- HUD（视野下方三行）：遥操状态 / 手臂与夹爪 / 链路（RTT、丢包、适配器有无 FR3 反馈、dry-run）。
- 语音：遥操开启 / 关闭、暂停 / 继续、已连接、连接断开、已对齐前方。
- 设置面板：连接（自动发现列表、手动 IP 数字键盘）、操控（位置 / 旋转缩放、发送频率、平滑、夹爪映射）、显示提示、对齐。
  设置存在头显的 `/sdcard/Android/data/com.infinity4b.frankateleop/files/settings.cfg`。

### 夹爪为什么慢

Franka Hand 每条指令有约 0.6 s 的固定延迟（2026-11-03 用 `tools/gripper_latency.py` 实测：原地 move 0.6–0.7 s，
开口在发指令 0.5–0.7 s 后才开始变，stop 后还会多走 2–3.5 cm），从按下握持键到合拢完约 1.4 s。这是 Franka Hand 本身的限制
（官方确认它目前不支持实时控制），所以默认用开 / 合两态（`gripper.mode: binary`）。开口跟握持键走的 `linear` 模式跟不上，
只留作对照。想快一点可以把 `gripper.open_width` 设小（比如 0.04），张开时只张到这么宽，下一次闭合的行程更短。

## 坐标与零点

- 头显发的是相对零点的末端位姿，x 前 / y 左 / z 上，和 FR3 基座系同向；姿态是基座系下的增量：
  `p = p0 + (x, y, z)`，`R = Rzyx(roll, pitch, yaw) · R0`。手柄在世界里怎么转，夹爪就怎么转。
- **零点 = 适配器启动时的末端位姿**（加 `--home` 时先归位到 ready 姿态再取）。按 A、第一次开启遥操时都慢速回到这里。

## 安全行为

| 情况 | FR3 |
|---|---|
| 遥操关闭 / 头显没连 | 停在最后的目标（启动时就是当前位置） |
| 0.12 s 没收到包、面板打开、摘下头显 | 停在当前指令 |
| 1 s 没收到包 | 停止发布，停在最后的目标 |
| 碰撞触发 reflex | 自动清错，从实际位置重新起步（不按旧指令冲回去）；一分钟超过 3 次就退出 |
| 目标离实际位置太远 | 阻抗误差被限在 5 cm / 0.3 rad 以内，弹簧力有上限 |
| Ctrl+C | 跟踪器平滑停止，FR3 停住 |

没测过 GPU 满载时的实时延迟，不要在训练跑着时遥操。

## 调参（`config.yaml`）

`config.yaml` 是机器人端的硬上限，头显的设置面板改不了。改完重启 `start.sh` 生效。常改的几项：

| 项 | 默认 | 说明 |
|---|---|---|
| `arm.workspace.right` | x −0.25~0.30，y ±0.40，z ±0.30 | 相对零点的末端范围（m / rad）。**z 下限 −0.30 约为基座上方 0.19 m，碰不到和基座同高的桌面**；要抓桌面上的东西，先量好桌面相对基座的高度再慢慢放宽 |
| `arm.max_lin_vel` / `max_ang_vel` | 0.5 m/s、1.5 rad/s | 跟随时末端指令速度上限 |
| `arm.reset_lin_vel` | 0.15 m/s | 回零点的速度 |
| `franka.stiffness` | 600 N/m、40 Nm/rad | 笛卡尔阻抗刚度。越大跟得越紧，碰到东西的力也越大 |
| `franka.error_clip` | 0.05 m、0.3 rad | 目标和实际偏差的上限，限住弹簧力 |
| `franka.collision` | libfranka 示例的保守值 | 碰撞阈值（关节力矩 / 末端力），太灵敏时调大 |
| `franka.gripper` | binary、30 N、全开 | `force` 抓取力，`open_width` 张开宽度（0 = 全开约 8 cm） |
| `franka.home_joints` | ready 姿态 | `--home` 归位的关节角 |
| `network.port` | 47800 | 改了要同步改 `host/host.env` 的 `VR_PORT` 并重跑防火墙脚本 |

## 第一次上真机

先用 `fake_vr.py` 代替头显，在主机本机连 127.0.0.1（不经过防火墙），动作幅度缩小：

```bash
./start.sh                                                              # 终端 1
uv run --frozen python tools/fake_vr.py --host 127.0.0.1 --scale 0.3    # 终端 2
```

FR3 应该先回零点（就在原地），再画一个直径 6 cm、上下起伏 ±1.5 cm 的小圈，同时末端俯仰 ±5°，夹爪开合几次，6 s 后停住。
`fake_vr.py --scenario dropout` 测断包（0.12 s 保持、1 s 停发），`--scenario static` 测静止不漂。

单独测夹爪（手臂不动，不要和 `start.sh` 同时跑，夹爪会开合十几次）：

```bash
uv run --frozen python tools/gripper_latency.py
```

## 排障

| 现象 | 原因和处理 |
|---|---|
| 头显发现列表里没有主机 | ufw 没放行 UDP 47800（`sudo bash host/setup_firewall_sudo.sh`，换了 Wi-Fi 要重跑）；不在同一局域网；路由器隔离广播时在设置面板手动填主机 IP（`host/check_host.sh` 会打印） |
| HUD 反馈显示“无” | 适配器没连上 FR3，看主机终端的报错。`--dry-run` 时本来就没有反馈 |
| franky 报 `UDP receive: Timeout` | ufw 挡了 FR3 发回的 UDP 包（TCP 能连上，握手正常），运行 `sudo bash host/setup_firewall_sudo.sh` |
| 连不上机器人 / TCP 1337 不通 | Desk 里没激活 FCI；网口没切（`host/net_up.sh`）；VPN 接管了路由（`ip route get 192.168.3.1` 看是不是走 FR3 的网口） |
| `Incompatible library version` | 机器人系统版本和 wheel 里的 libfranka 不对应，见 [install.md](install.md#机器人系统-59-及以上) |
| `start.sh` 报没有 `/dev/cpu_dma_latency` 的写权限 | 没运行 `sudo bash host/setup_rt_sudo.sh`，或运行后没重新登录 |
| 运动一卡一卡、报 `communication_constraints_violation` | 实时性不够：确认是 RT 内核、governor 是 performance（`host/check_host.sh`），主机不要同时跑重负载 |
| `net_up.sh` 报 `Not authorized to control networking` | 在 ssh 里运行了。要在主机桌面上的终端里运行 |
| 头显一直显示暂停 / 失去焦点 | 头显没设边界（恢复出厂后要重设），或设置面板开着、摘下了头显 |
| 恢复后手臂不跟 | 要先松开扳机再按 |
| 触发 reflex 后适配器退出 | 一分钟内超过 3 次自动清错。检查是不是撞到东西、`franka.collision` 是否太灵敏；Desk 里确认没有别的错误再重启 |
| 夹爪跟手慢 | Franka Hand 的固定延迟，见[上面](#夹爪为什么慢) |
