# 无需助手：返回初始观察位姿

适用本机 Windows + WSL Ubuntu-22.04、现有 estun_segmented 程序。只控制机械臂返回，不启动视觉抓取、不操作夹爪。

初始观察位取自 `D:\catch\task_B\config.json` 的 observation_pose；公开版本不提供现场观察位，需在本地重新测量并填写。当前平移速度 50 毫米/秒、旋转速度 6 度/秒。命令直接读取配置，不需要手动换算或输入坐标。

## 1. 现场与程序准备

确认返回路径可用、控制器无故障并处于 AutoReady。Idle 或 PowerOff 状态下，需在示教器/控制器界面按现场流程启用自动运行；程序不会代为切换。退出正在运行的 task_B、其他运动程序和旧 bridge/反馈转发器，避免重复控制或端口占用。

本教程的现有 move 默认顺序为姿态 → XY → Z，到位容差内的阶段会跳过；它不会自动先抬升，也不提供碰撞规划。现场安排必须与这个运动顺序相容。

## 2. Windows PowerShell：读取网络并准备本次配置

打开 PowerShell，运行：

```powershell
wsl.exe -- bash -lc 'hostname -I; ip -4 route show default'
```

本机本次输出 WSL 地址为 `198.51.100.2`，默认网关为 `198.51.100.1`。WSL 重启后可能变化，后面的命令应填写当次输出。控制器地址为 `192.0.2.10`，Windows 接收反馈地址为 `192.0.2.20:9006`。

在 PowerShell 中生成配置快照，原 config.json 不变：

```powershell
$returnConfig = Get-Content D:\catch\task_B\config.json -Raw | ConvertFrom-Json
$returnMove = [ordered]@{target=$returnConfig.observation_pose}
$returnConfig.motion.psobject.Properties | ForEach-Object { $returnMove[$_.Name] = $_.Value }
$returnJson = $returnMove | ConvertTo-Json -Depth 5
[System.IO.File]::WriteAllText('D:\catch\task_B\return_observation_config.json', $returnJson, (New-Object System.Text.UTF8Encoding($false)))
Get-Content D:\catch\task_B\return_observation_config.json
```

检查 target 是所需初始观察位；速度字段使用米/秒、弧度/秒，当前应为 linear_speed=0.05、angular_speed=0.10471975511965977。

在这个 PowerShell 窗口启动现有只读反馈转发器，并保持窗口运行：

```powershell
& 'C:\Users\1\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -u D:\catch\estun\windows_feedback_relay.py --wsl-ip 198.51.100.2
```

应看到已转发反馈的输出。该脚本只转发反馈，不发送运动。返回观察位无需挂载相机或夹爪 USB。

## 3. WSL 终端 A：启动 bridge

另开终端进入 WSL（可在新 PowerShell 输入 `wsl`），执行：

```bash
cd /mnt/d/catch/task_B
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source ../camera/scripts/handeye_env.sh
ros2 run estun_segmented bridge --controller-ip 192.0.2.10 --feedback-source-ip 198.51.100.1 --bind-ip 0.0.0.0
```

保持这个终端运行。feedback-source-ip 使用第 2 步查到的默认网关。这里启动正常 bridge，供明确的 execute 指令使用；不要同时启动另一个 bridge。

## 4. WSL 终端 B：只读检查、预览、执行

再开一个 WSL 终端，先加载同一环境：

```bash
cd /mnt/d/catch/task_B
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source ../camera/scripts/handeye_env.sh
mkdir -p runs/manual_return
ros2 run estun_segmented move --read --log runs/manual_return/read.jsonl
```

检查 ROBOT_INFO 中 state=AutoReady、fault_flag=0。然后预览（不发送动作）：

```bash
ros2 run estun_segmented move --config return_observation_config.json --log runs/manual_return/preview.jsonl
```

明确执行返回：

```bash
ros2 run estun_segmented move --config return_observation_config.json --execute --log runs/manual_return/motion.jsonl
```

程序重新读取实时起点，按原控制器程序逐阶段执行。无需手动插补或调整坐标。只想返回观察位时，不要改用 `run_task_b.sh --execute`，后者会启动夹爪及抓取流程。

## 5. 判断完成与退出

以程序日志为准：最后出现 COMPLETE，且前面各实际运动阶段有 ARRIVED，表示现有程序确认到位。若已在容差内，COMPLETE 可能显示 phases=0、segments=0，表示无需再发送运动。

程序当前位置容差为 0.5 毫米、姿态角容差为 0.5 度，故返回的是该容差内的观察位姿，不要求反馈小数位完全相同。

若出现 ABORT，停止后续操作，保存日志并按 reason 排查，不连续重发。常见情况：

| 程序输出 | 处理 |
|---|---|
| 控制器状态不允许运动：Idle/PowerOff | 在示教器/控制器界面切换至 AutoReady 后再检查 |
| 没有收到 RobotInfo | 检查控制器 ROS 反馈开关、反馈目标 IP/9006、转发器和 WSL 当前地址 |
| 配套 bridge 状态缺失或过期 | 保留日志，检查 bridge 会话和实时反馈；不通过放宽超时强行执行 |
| 地址已被使用、发布者或 bridge 不唯一 | 结束旧会话，确保仅一个转发器、一个 bridge、一个运动程序 |
| 故障、超时或未确认到位 | 先检查控制器与示教器状态，不把退出当作已经完成 |

动作完成后，在 WSL 终端 A 按 Ctrl+C 结束 bridge，再在 PowerShell 转发器窗口按 Ctrl+C 结束转发。终端 B 的运动程序正常完成会自行退出。

Ctrl+C 只停止后续发送，已被控制器接收的当前段可能继续；需要停止机械臂时使用现场控制器停止按钮或硬件急停。

## 本次实际验证

2026-10-28 使用以上同一现有 move/bridge 和从 config.json 导出的参数。首次因 Idle 中止；用户切换 AutoReady 后再执行，14:39:47 程序记录 COMPLETE，完成 XY、Z 两阶段，姿态已在原设置容差内所以跳过；控制器 AutoReady、fault_flag=0。日志位于 `D:\catch\task_B\runs\return_observation_20261028_143827\motion.jsonl`。
