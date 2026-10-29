# 埃斯顿 S3-60 Pro 三阶段分段直线控制

程序按「固定 XYZ 调姿态 → 保持起点 Z 和目标姿态沿 XY 直线移动 → 固定目标 XY 和姿态移动 Z」执行。**只生成这三个大阶段的终点，每阶段一条 RobotMove、一个目标点，不再按 20 mm 或 5° 拆小步，也不批量下发小步点。**控制器负责整个阶段的直线/姿态插补；软件持续读取反馈，只在阶段末到位后进入下一阶段。默认仅预览，加 `--execute` 才执行。

已按用户确认进行一次真实动作：法兰局部 Y +45°、基座 XYZ 各 +100 mm。姿态和 Z 已完成，XY 在第 20 条指令未观察到 AutoRunning 后退出，完整目标未到达。详见 `TEST_RESULTS.md` 最后一节和 `logs/confirmed_move_result.json`。`run_confirmed_move.py` 是这一次请求的换算入口，默认仅预览；不要把重新运行当作续跑，它会从新的起点重新增加 100 mm，故障后需先排查，不得自动重试。

## 2026-10-29 通信恢复

真实反馈已恢复并实测约 19.84 Hz。控制器自身 IP 保持 `192.0.2.10`；控制器「设置 → ROS」的**目标服务端 IP/端口**为电脑 `192.0.2.20:9006`。本次验证只读，未发送运动。详细结果和当前进程记录见 [通信恢复记录](logs/communication_20261029/通信恢复记录.md)。

本轮已保留只读转发与桥接运行，请不要重复启动。以后重启电脑/WSL 后，可用下列两个终端恢复；新入口自动读取当前 WSL 地址和 Windows 网关，不依赖旧 NAT 地址，也不需要修改 PowerShell 执行策略。

Windows PowerShell 终端 A（保持打开）：

```powershell
cd D:\catch\estun
& "$env:USERPROFILE\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" -X utf8 -u .\start_feedback.py
```

WSL Ubuntu 终端 B（保持打开，强制只读，默认 ROS_DOMAIN_ID=42）：

```bash
cd /mnt/d/catch/estun
bash start_bridge_readonly.sh
```

WSL Ubuntu 终端 C 验证：

```bash
cd /mnt/d/catch/estun
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
python3 tests/observe_robot_info.py --seconds 15 --output logs/communication_health.json
ros2 topic echo /RobotInfo --once
```

`PowerOff` 是控制器状态，不是通信断开。时间戳是桥接本地接收时间，未验证控制器源时间。只读探测 `probe_info.sh` 会自己占用 UDP 9006 并使用独立 domain 189，不要在上述常驻桥接运行时再启动它。

## 文件与官方依据

- `ros2_ws/src/codroid_msgs`：从官方压缩包原样复制的消息包，在本机重新编译。
- `ros2_ws/src/estun_segmented/estun_segmented/core.py`：旋转换算、分段、到位状态机。
- `bridge.py`：沿用官方 `codroid_io.py` 的 UDP/JSON 转发逻辑，增加新鲜度、异常检查与参数。
- `runner.py`：读取起点、配置/命令行输入、预览、逐段执行、JSONL 日志。
- `official/codroid_bridge/codroid_node`：原始通信示例，仅供对照。不要同时运行官方 bridge 和本 bridge，也不要运行会自动运动的官方 `codroid_move_test.py`。
- `tests`：本地数学/状态机测试和真实 ROS2 + 模拟 UDP 控制器测试。

官方 `ROS2通信协议_测试_解密.docx` 中的关键表格是图片，已逐项核对 `inspection/image7.png`、`image8.png`、`image11.png`、`image12.png`。

姿态定义核对了[酷卓 S 系列官方用户手册](https://www.codroid.ai/download/shouce/v1107.pdf)第 9.2 节，PDF 第 125 页、印刷页码 107：右手系 X-Y-Z 固定角，对应 `Rz(rz) Ry(ry) Rx(rx)`。程序把目标旋转转换为邻近等价欧拉角，处理 ±π 跨越和万向节锁，只发送最终姿态；姿态插补、轨迹规划及逆解均交给官方控制器。不再自行生成 SLERP 小步目标。

官方 ROS2 协议没有提供坐标系/TCP 选择字段、运动指令编号、接收确认包、具体逆解错误码或停止/取消指令。因此：

- 所有输入必须与现场 RobotInfo 的参考坐标系和 TCP 定义相同；先对照示教器，不能把视觉相机坐标直接填入。
- 不设置 `header.frame_id` 来假装切换坐标系；它不在官方 UDP 指令中。
- 报错时只能报告 `fault_flag`、状态、位姿或超时。具体逆解/奇异/限位原因需要查示教器；不能仅凭超时断言逆解失败。
- 异常退出或 Ctrl+C 只停止后续发送。控制器已接收的当前段可能继续，现场需要用控制器停止按钮或硬件急停。
- 很短的运动可能未被反馈采样捕获 AutoRunning；程序会保守退出，不会凭 AutoReady 和目标接近就认定指令执行过，更不会重发。

## 通信协议

| 项目 | 官方定义 |
|---|---|
| 控制器 | 默认 `192.0.2.10`，UDP `9005` 接收运动 |
| 上位机 | UDP `9006` 接收状态；在控制器 Web 的「设置 → ROS」填写上位机可达 IP 和 9006 |
| `/RobotInfo` | `codroid_msgs/msg/RobotInfo` |
| `/RobotMove` | `trajectory_msgs/msg/JointTrajectory` |
| `joint_positions` | 六关节，rad |
| `end_positions` | `[x,y,z,rx,ry,rz]`，m、rad |
| `state` | `AutoReady` 可收指令，`AutoRunning` 运动中；`Idle`/`PowerOff` 不能执行 |
| `fault_flag` | 0 无故障，非零需人工排查和清除 |
| 笛卡尔 `joint_names` | `['x','y','z','rx','ry','rz']`，不能混入 Joint1 等关节名 |
| `positions` | 与名称一一对应，6 个数 |
| `velocities` | **2 个数**：平移速度 m/s、姿态角速度 rad/s |
| `accelerations` | **2 个数**：平移加速度 m/s²、姿态角加速度 rad/s² |

示例目标仅说明消息格式，不代表现场可达位置：

```json
{"joint_names":["x","y","z","rx","ry","rz"],"points":[{"positions":[0.4,0.1,0.302,3.141592653589793,0.0,0.0],"velocities":[0.005,0.03490658503988659],"accelerations":[0.02,0.08726646259971647]}]}
```

每次只发送一个 point；不使用 PTP、关节空间运动或一次性多目标队列。UDP 不自动重发。

## 1 在 WSL 中构建和本地测试

当前项目位于 Windows `D:\catch\estun`，WSL 路径 `/mnt/d/catch/estun`。下面命令在 **WSL Ubuntu 22.04 的 Bash** 中运行。

```bash
cd /mnt/d/catch/estun
bash build.sh
bash test_local.sh
```

如果现场是原生 Ubuntu，把工程复制到本地目录后，在该目录运行同样的两个脚本。脚本仅依赖已安装的 ROS2 Humble、colcon 和 Python 标准库，不需要 scipy/MoveIt。

测试会临时占用本机 UDP 9005、9006，仅连接 `127.0.0.1`，使用独立 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY。先关闭正在运行的 bridge。测试不连接真实机械臂。

每开一个现场终端先执行：

```bash
cd /mnt/d/catch/estun
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
```

## 2 先确认能读取真实 RobotInfo

网线连接后确认电脑和控制器的地址配置。控制器软件应支持官方 ROS 通信，协议要求 1.6.6 或更新；本程序不会刷写控制器固件，也不使用随附 Eco 模型替代 Pro 的实际参数。

本机已实际核对：Windows 以太网为 `192.0.2.20`，控制器 ROS 反馈目标为 `192.0.2.20:9006`；WSL 使用 NAT，地址为 `198.51.100.2`，Windows 的 WSL 网关为 `198.51.100.1`。这组配置已成功读取真实 RobotInfo。

在 **Windows PowerShell** 中启动仅反馈转发工具并保持运行：

```powershell
cd D:\catch\estun
& 'C:\Users\1\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' .\windows_feedback_relay.py --wsl-ip 198.51.100.2
```

该脚本只接收来自 `192.0.2.10` 的反馈并原样转发到 WSL 9006，没有发送运动指令的路径。Windows 安装了普通 Python 时也可用 `python windows_feedback_relay.py ...`。退出用 Ctrl+C。

WSL 重启后地址可能改变，运行 `ip -4 addr show eth0` 和 `ip route show default` 重新核对：relay 的 `--wsl-ip` 填 WSL eth0 地址；下面 bridge 的 `--feedback-source-ip` 填默认网关地址。原生 Ubuntu 或 WSL 已直接接收控制器 UDP 时，不运行转发工具，并去掉 `--feedback-source-ip`。

Windows TCP `portproxy` 不能转发 UDP。若脚本提示未收到 Windows 侧反馈，检查 ROS 开关、控制器目标 IP 和 Windows UDP 入站规则；不要关闭整个防火墙。本次已无需修改防火墙便收到数据。

终端 A（只读 bridge，拒绝运动转发）：

```bash
ros2 run estun_segmented bridge --receive-only --feedback-source-ip 198.51.100.1
```

控制器地址不是默认值时：

```bash
ros2 run estun_segmented bridge --controller-ip 192.0.2.10 --receive-only --feedback-source-ip 198.51.100.1
```

终端 B：

```bash
ros2 topic list -t
ros2 interface show codroid_msgs/msg/RobotInfo
ros2 topic echo /RobotInfo --once
ros2 topic hz /RobotInfo
# 上一个 hz 命令用 Ctrl+C 结束后，再运行：
ros2 run estun_segmented move --read --log logs/site_info.jsonl
```

必须看到真实六关节、六维末端位姿、state 和 fault_flag。先对照示教器确认 m/rad 换算、坐标系和 TCP。读取允许显示 Idle/PowerOff，但执行必须是无故障、稳定 AutoReady。

也可一键只读探测默认控制器，最长等待约 10 秒并保存日志（运行前关闭终端 A 的 bridge）：

```bash
bash probe_info.sh --feedback-source-ip 198.51.100.1
```

## 3 预览分段和官方 RobotMove 格式

终端 A 停止只读 bridge，改为正常 bridge。它本身不会主动产生运动目标：

```bash
ros2 run estun_segmented bridge --feedback-source-ip 198.51.100.1
```

终端 B，以下命令**只预览**，读取当前真实位姿后生成相对目标。`--delta` 长度用 m，姿态用 rad；单轴姿态增量只是方便初次测试，一般目标使用 `--target`。

```bash
# 姿态：只改变 rz 约 1°，XYZ 保持实际起点。
ros2 run estun_segmented move --stage orientation --delta 0 0 0 0 0 0.01745329252 --log logs/preview_orientation.jsonl
# Z：正向 2 mm，只发送一个 Z 阶段终点。
ros2 run estun_segmented move --stage z --delta 0 0 0.002 0 0 0 --log logs/preview_z.jsonl
# XY：X、Y 各正向 2 mm，只发送一个 XY 阶段终点。
ros2 run estun_segmented move --stage xy --delta 0.002 0.002 0 0 0 0 --log logs/preview_xy.jsonl
```

日志 `PREVIEW` 是将要发布的完整 JSON。预览不会向 `/RobotMove` 发布任何指令。每次运行重新读取起点，因此预览与后续执行之间不要手动移动机械臂；执行时也会重新生成并打印计划。

没有真实反馈时可离线检查，`--start` 不能与 `--execute` 同用：

```bash
ros2 run estun_segmented move --start 0.4 0.1 0.3 3.141592653589793 0 0 --delta 0.002 0.002 0.002 0 0 0.01745329252 --log logs/offline_preview.jsonl
```

## 4 人工执行低速短距离单阶段实机测试

先由现场操作者确认：位置单位 m、姿态单位 rad 和固定轴角定义与示教器一致；当前 TCP/坐标系正确；正向 Z 和 XY 路径有净空；调姿态时工具扫过区域无障碍；限位和奇异风险已检查；硬件急停可用且操作者能立即触及。正向 Z 不等于现场的“向上”，要以坐标轴实际方向为准。

**以下含 `--execute` 的命令会运动，必须由现场人员逐条明确运行。**默认低速 5 mm/s、2°/s；初次测试进一步降为 2 mm/s、1°/s。负方向请修改对应 delta 的符号。不要把测试命令放入自动批量脚本。

先测试姿态，确认 ARRIVED 和 COMPLETE 后结束本次测试：

```bash
ros2 run estun_segmented move --stage orientation --delta 0 0 0 0 0 0.01745329252 --angular-speed 0.01745329252 --execute --log logs/site_orientation.jsonl
```

再测试 Z，一条指令完成整个 Z 阶段，到位后返回：

```bash
ros2 run estun_segmented move --stage z --delta 0 0 0.002 0 0 0 --linear-speed 0.002 --execute --log logs/site_z.jsonl
```

再测试 XY，一条指令沿直线到终点：

```bash
ros2 run estun_segmented move --stage xy --delta 0.002 0.002 0 0 0 0 --linear-speed 0.002 --execute --log logs/site_xy.jsonl
```

每次只能运行一个 move 程序。现场不能同时用其他发布者向 `/RobotMove` 下指令。程序发现连接丢失或其他运动发布者会退出。任何异常先查现场状态和示教器，人工排除故障后重新运行；不要自动循环重试。

## 5 单阶段成功后测试完整三阶段

先预览：

```bash
ros2 run estun_segmented move --delta 0.002 0.002 0.002 0 0 0.01745329252 --linear-speed 0.002 --angular-speed 0.01745329252 --log logs/preview_all.jsonl
```

现场确认三个单阶段测试都成功且完整路径可用后，再手动运行：

```bash
ros2 run estun_segmented move --delta 0.002 0.002 0.002 0 0 0.01745329252 --linear-speed 0.002 --angular-speed 0.01745329252 --execute --log logs/site_all.jsonl
```

它只执行姿态、XY、Z 三个阶段，共三条单目标指令。XY 阶段保持起点 Z，最后才改变高度。阶段完成后才发送下一阶段，阶段内部不做小步停等。相对目标与起点使用同一帧反馈；无需移动的阶段自动跳过。

指定绝对目标（以下仅为格式示例，位置需换成你现场确认的可达目标）：

```bash
ros2 run estun_segmented move --target 0.4 0.1 0.3 3.141592653589793 0 0
```

配置文件方式：编辑 `ros2_ws/src/estun_segmented/config/target.json` 后先预览。示例文件的位置不是现场推荐位置，不要未修改就执行。

```bash
ros2 run estun_segmented move --config ros2_ws/src/estun_segmented/config/target.json
# 人工核对预览后，需要真实执行时再加 --execute。
```

配置支持 `position_tolerance`（默认 0.0005 m）、`angle_tolerance`（默认 0.5° 的弧度）、`feedback_timeout`（0.5 s）、`accept_timeout`（3 s）、`segment_timeout`（整个阶段默认 120 s）、`settle_seconds`（0.2 s）和 `settle_samples`（3）。命令行可用 `--phase-timeout` 调整阶段超时，兼容旧名 `--segment-timeout`。已移除 `--step-mm`、`--angle-step-deg` 和对应小步配置字段。

## 视觉接入

先把视觉结果换算到 RobotInfo 所用的参考系/TCP，单位转换为 m、rad。初始化 ROS 和同一个 MotionNode 后，调用 `node.move_to(target, stage='all', execute=True)`，其内部读取新鲜起点并完成整套反馈执行。`runner.make_node(journal, settings)` 创建该节点；不要在其外同时启动另一个 spin，也不要从订阅回调里阻塞调用，视觉结果可由上层主循环取出后调用。

只需要计算目标时，可直接 `core.plan(start, target, settings)`；它返回最多三个 Segment，依次是姿态、XY、Z 的终点。使用 `move_to` 完成阶段顺序和到位确认。

## 故障与日志

- `STATE`：状态或 fault_flag 改变。
- `START`：本次实际起点、关节角、目标及参数。
- `PLAN_STAGE` / `PREVIEW`：三阶段段数及逐条命令；无需移动的阶段为 0 段。
- `SEND`：仅发布当前段；bridge 中 `UDP_SEND` 记录实际转发。
- `ARRIVED`：AutoRunning 后回到 AutoReady，位置/旋转误差达标，并连续稳定至少 0.2 秒及 3 帧。
- `ABORT`：停止后续发送、不会重发。查 reason、bridge 日志及示教器。
- `COMPLETE`：所有需要的分段已经完成。

JSONL 日志追加写入。当前三大段版本的模拟测试在 `logs/ros_loopback_phase/`，完整流程为 3 条命令；旧的 `logs/ros_loopback/` 和此前实机日志保留作历史。现场测试请用 `logs/site_*.jsonl`，不能把模拟反馈当作实机运动结果。已完成的验证及版本区别见 `TEST_RESULTS.md`。
