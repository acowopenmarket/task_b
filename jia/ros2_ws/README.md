# MS42DC 夹爪 ROS 2 控制程序

独立串口驱动，通过 USB 或 TTL 串口控制 WHEELTEC MS42DC 夹爪。
目标环境为 Ubuntu + ROS 2 Humble；采用标准 `ament_python` / `ament_cmake`
接口包，其他 ROS 2 版本需在目标环境编译验证。无需解压、编译厂家 `serial_ros2` 包。

## 1. 已实现的功能

- `open` / `close` 服务：按配置的行程发送一次张开 / 闭合命令。
- `move` 服务：指定方向、相对电机角度和速度，使用物理单位。
- `command` 话题：`false` 张开，`true` 闭合；每条消息发送一次命令。
- 定时查询、主动查询、9 字节反馈校验、分段和连续帧解析。
- 32 位角度反馈，避免厂家 `Motor.msg` 的 `uint16 angle` 截断。
- 串口重连；断线期间的动作失败返回，不缓存、不重发动作。
- 默认 `dry_run=true`，只打印动作帧，不打开串口、不生成虚假反馈。

启动仅连接串口和查询状态，不自动张开、闭合、归零或烧录固件。

## 2. 硬件与资料差异

1. 给电机提供手册要求的 10–25 V 外部电源；USB 用于通信。
2. 在目标电脑确认串口，如 `/dev/ttyACM0`、`/dev/ttyCH343USB0`，或 Windows 的 `COM3`。
   推荐使用 `/dev/serial/by-id/` 下的稳定路径。USB 固定 115200；TTL 波特率须匹配设备设置。
3. **确认 2 号拨码开关处于原始串口协议模式。** 2026 年夹爪手册第 10 页写 ON，
   2025 年 ROS 手册第 3 页写“往下拨”；资料互相矛盾，按实物板版本和有效反馈判断。
   不能仅凭另一份手册的开关示意推断。退出厂家上位机，避免串口被同时占用。
4. 默认设备 ID 为 1，细分 32。
5. `travel_turns` 是电机转动圈数，**不是夹爪开口宽度**。随附固件从 4.5 圈到 9.5 圈
   有多个版本，文件名不代表当前设备已经烧录了哪一版。当前默认按用户指定的 USB
   开合示例配置为 5.2 圈（1872°）、20 rad/s；运行真机前仍需核对实际型号 / 固件行程。

手册中 USB 闭合示例为 1872°（5.2 圈），不能套用到所有型号。
反馈示例 `01 01 00 00 00 00 8C A0 2C` 的 `0x8CA0 / 10 = 3600°`，
原文将其写成 360°；本程序按字节和十倍缩放解析。

## 3. 编译安装（Ubuntu + 已安装 ROS 2 Humble）

将本目录 `ros2_ws` 复制到 ROS 2 电脑，例如 `~/ros2_ws`：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
sudo apt update
sudo apt install python3-serial python3-colcon-common-extensions python3-rosdep python3-pytest
# 如果该系统此前未初始化 rosdep，先执行一次 sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select ms42_gripper
colcon test-result --verbose
```

若串口提示权限不足：

```bash
sudo usermod -aG dialout "$USER"
# 注销并重新登录后生效；用 ls -l /dev/ttyACM0 确认设备所属组。
```

不安装 ROS 2，也可以用 Python 3.10+ 验证协议：

```bash
cd ~/ros2_ws/src/ms42_gripper
python3 -m unittest discover -s test -v
python3 -m ms42_gripper.cli close --angle 1872 --speed 20
# 输出：7B 01 02 01 20 49 20 00 C8 F8 7D
python3 -m ms42_gripper.cli query
# 输出：7B 01 00 00 00 00 00 00 00 7A 7D
```

`protocol_cli` 永远只生成字节，不控制硬件，也不依赖 pyserial。

### 本电脑的 WSL 环境（已验证）

2026-10-09 已在本电脑 `Ubuntu-22.04`（WSL 2）中确认 ROS 2 Humble、colcon、
pyserial 和 pytest 可用。两个包已编译成功，30 项测试全部通过，0 跳过。
默认 launch 试运行已验证可以启动并正常退出。没有接入夹爪，不代表已完成实机验证。

当前保留的构建位于 `build_wsl` / `install_wsl` / `log_wsl`。
直接在 **WSL Ubuntu 终端**运行：

```bash
source /opt/ros/humble/setup.bash
cd /mnt/d/catch/jia/ros2_ws
source install_wsl/setup.bash
ros2 launch ms42_gripper gripper.launch.py
```

修改代码后重新编译和测试：

```bash
source /opt/ros/humble/setup.bash
cd /mnt/d/catch/jia/ros2_ws
colcon --log-base log_wsl build --build-base build_wsl --install-base install_wsl
source install_wsl/setup.bash
colcon --log-base log_wsl test --build-base build_wsl --install-base install_wsl \
  --packages-select ms42_gripper --event-handlers console_direct+
colcon test-result --test-result-base build_wsl --verbose
```

源码压缩包不包含构建产物；解压到其他电脑后需自行构建。

### WSL 2 接入夹爪 USB 串口

当前检查未发现 WSL 内的 `ttyACM*` / `ttyUSB*` / `ttyCH343USB*` 设备，
Windows 的 PATH 中也未找到 `usbipd`。WSL 程序使用 Linux 设备路径，不能直接填 Windows 的 COM 号。
依据 [Microsoft 官方 USB 接入说明](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)，
先安装 usbipd-win，再将指定 USB 设备共享 / 接入 WSL；以下是操作说明，本程序没有执行安装或 USB 转接。

Windows PowerShell 安装（交互式安装可在需要重启时提示）：

```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list
```

接好夹爪 Type-C 数据线，识别 CH9102 / 串口设备的真实 BUSID。
在管理员 PowerShell 中执行共享命令，将 `<BUSID>` 替换为实际值：

```powershell
usbipd bind --busid <BUSID>
```

保持 WSL 终端打开，在普通 PowerShell 中接入：

```powershell
usbipd attach --wsl --busid <BUSID>
```

然后在 WSL 中检查：

```bash
lsusb
ls /dev/ttyACM* /dev/ttyUSB* /dev/ttyCH343USB* 2>/dev/null
dmesg | tail -30
```

使用实际出现的 Linux 设备路径启动硬件模式，并按前文确认实际行程。
USB 接入 WSL 后，Windows 上位机不能同时使用该设备。USB 拔插或 WSL 重启后，可能需要重新 attach。
若 USB 已接入但没有串口设备，应先检查 `dmesg` 的驱动 / 设备枚举信息，不要猜测串口路径。

## 4. 启动与首次验证

先试运行：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ms42_gripper gripper.launch.py
```

另开终端，同样加载环境后调用下面的服务，即可看到打印的串口帧。
试运行中的动作服务 `success=true` 表示成功生成和打印指令；状态查询会明确返回失败，
因为没有真实设备反馈。

确认串口和实际行程后运行真机。以下采用用户指定的手册开合指令（5.2 圈、20 rad/s）：

```bash
ros2 launch ms42_gripper gripper.launch.py \
  port:=/dev/ttyACM0 dry_run:=false travel_turns:=5.2 speed_rad_s:=20.0
```

也可直接运行节点：

```bash
ros2 run ms42_gripper gripper_node --ros-args \
  -p port:=/dev/ttyACM0 -p dry_run:=false -p travel_turns:=5.2
```

连接后先查询，确认反馈，再用 `move` 小角度验证方向，最后调用完整开合。
`serial_connected=true` 只表示串口打开，不保证夹爪响应；`feedback_alive=true`
才表示近期收到校验有效的设备反馈。需要分别在终端中查看：

```bash
ros2 topic echo /ms42_gripper/serial_connected
ros2 topic echo /ms42_gripper/feedback_alive
ros2 topic echo /ms42_gripper/state
```

首次小角度验证（9° 是电机角度，不是夹指角度）：

```bash
ros2 service call /ms42_gripper/move ms42_gripper_interfaces/srv/Move \
  "{close: true, angle_deg: 9.0, speed_rad_s: 5.0}"
```

## 5. 控制接口

默认节点名 `ms42_gripper`，接口使用节点私有名称。

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/ms42_gripper/open` | `std_srvs/srv/Trigger` | 逆时针，按 `travel_turns` 张开 |
| `/ms42_gripper/close` | `std_srvs/srv/Trigger` | 顺时针，按 `travel_turns` 闭合 |
| `/ms42_gripper/move` | `ms42_gripper_interfaces/srv/Move` | 相对移动，角度单位 °，速度单位 rad/s |
| `/ms42_gripper/query` | `std_srvs/srv/Trigger` | 请求反馈，接收有效帧后发布 state |
| `/ms42_gripper/command` | `std_msgs/msg/Bool` | false 张开，true 闭合 |
| `/ms42_gripper/state` | `ms42_gripper_interfaces/msg/GripperState` | 实际反馈，接收时间戳、32 位原始角度 |
| `/ms42_gripper/serial_connected` | `std_msgs/msg/Bool` | 串口是否打开 |
| `/ms42_gripper/feedback_alive` | `std_msgs/msg/Bool` | 最近是否收到有效反馈 |

当前默认 `open` / `close` 和 `command` 的开合数据帧与用户提供的手册截图一致：

```text
闭合：7B 01 02 01 20 49 20 00 C8 F8 7D
张开：7B 01 02 00 20 49 20 00 C8 F9 7D
```

默认仍为 `dry_run=true`；需显式启用硬件模式才会发送这些数据。
`tools/close_once.py`、`tools/open_once.py`、`tools/run_one_cycle.py` 也采用 1872°、20 rad/s。
其中两个单次脚本仍绑定序列号 `EXAMPLE_DEVICE_SERIAL`；步进补闭合脚本 `finish_close_stepwise.py`
是另外的实验工具，不是完整开合入口。单次张开脚本不再使用历史测试的 20.9° 作为位置基准。

```bash
# 张开
ros2 service call /ms42_gripper/open std_srvs/srv/Trigger '{}'
# 闭合
ros2 service call /ms42_gripper/close std_srvs/srv/Trigger '{}'
# 指定顺时针转动 90°，速度 10 rad/s
ros2 service call /ms42_gripper/move ms42_gripper_interfaces/srv/Move \
  "{close: true, angle_deg: 90.0, speed_rad_s: 10.0}"
# 请求一次状态
ros2 service call /ms42_gripper/query std_srvs/srv/Trigger '{}'
# 话题控制：只发布一次，避免重复执行相对运动
ros2 topic pub --once /ms42_gripper/command std_msgs/msg/Bool '{data: false}'
```

动作服务成功表示串口完成写入，不是设备确认、夹紧成功或运动完成。
反馈 `target_reached` 只转发设备标志；由于堵转保护、请求延迟以及反馈没有命令序号，
不能将这个标志直接当作某次夹取任务成功。程序不推断夹持力或物体是否被抓住。

`move` 为相对电机角度；重复命令会产生新的相对动作。`open` / `close` 也不是带位置闭环的
“设置开口宽度”。闭合通常由固件堵转保护停止；不保证每次运动都达到下发角度。
手册未提供位置模式可靠 STOP 指令，且明确机械爪不可使用速度模式，因此本程序不伪造停止、
失能或回零命令。**退出节点或断开 USB 不保证正在进行的动作停止**；急停应由硬件实现。

## 6. 参数与多夹爪

修改 `src/ms42_gripper/config/gripper.yaml`，重新构建 / 加载环境，或指定参数文件：

```bash
ros2 launch ms42_gripper gripper.launch.py params_file:=/absolute/path/gripper.yaml
```

显式传入的 launch 参数优先于 YAML；未传入时使用 YAML。
所有硬件参数均在启动时读取，运行中只读；修改后重启节点。

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `port` | `/dev/ttyACM0` | Windows 原生 ROS 2 可使用 `COM3` 等，需另行验证 |
| `baudrate` | 115200 | USB 固定该值，TTL 按设备配置 |
| `motor_id` | 1 | 1–255，不支持广播 |
| `subdivision` | 32 | 支持 2、4、8、16、32，建议 32 |
| `travel_turns` | 5.2 | 开合命令行程（1872°）；也是单次 move 角度上限 / 360 |
| `speed_rad_s` | 20.0 | open/close 的电机速度 |
| `max_speed_rad_s` | 20.0 | 软件速度上限，可配置，最高 42.0 |
| `poll_hz` | 5.0 | 定时查询频率，最高 20 Hz |
| `feedback_timeout_s` | 0.1 | 单次查询等待，必须小于查询周期 |
| `feedback_stale_s` | 1.0 | feedback_alive 的过期时间 |
| `reconnect_interval_s` | 2.0 | 串口打不开 / 断线后重新连接间隔 |
| `dry_run` | true | 不连接、不控制硬件 |

角度 / 速度线格式为十倍整数，分辨率 0.1；对非有限数、负数和超限输入返回失败。
单次角度协议上限 6553.5°，方向单独编码。反馈依厂家例程解析为无符号速度 / 无符号角度，
没有定义负方向的反馈符号；保留 `raw_speed`、`raw_angle` 便于按实机固件核对。
它们不代表标定后的绝对夹爪位置。

使用两个独立 USB 串口时，可以分别添加 `namespace:=left` 和 `namespace:=right`：

```bash
ros2 launch ms42_gripper gripper.launch.py namespace:=left port:=/dev/ttyACM0
ros2 launch ms42_gripper gripper.launch.py namespace:=right port:=/dev/ttyACM1
```

接口变为 `/left/ms42_gripper/open` 等。两条命令默认仍为试运行。
不要启动两个节点抢同一个串口；TTL 多机总线需要额外的集中调度，不在此独立单设备驱动范围内。
本版实现 USB/TTL 串口，不实现 CAN、NPN、`ros2_control` 或 MoveIt 夹爪 action；
资料缺少开口宽度换算、力反馈和可靠停止协议，不能承诺这些接口的物理语义。

## 7. 协议依据与验证范围

- `../MS42DC步进电机机械爪用户手册-20260324.pdf`：第 10–12 页位置控制，第 16–17 页反馈。
- `../5.ROS例程与教程/驱控一体步进电机ROS控制使用手册.pdf`：第 9 页夹爪相对运动模式。
- `../5.ROS例程与教程/源码/ROS2.zip` 内的 `step_motor/src/motor_node.cpp`：
  字段排列、异或校验、请求及反馈解码。
- [ROS 2 Humble 启动文件说明](https://docs.ros.org/en/ros2_documentation/humble/How-To-Guides/Launch-file-different-formats.html)
  和 [rclpy Humble 官方源码](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py)。

已提供不依赖 ROS 的协议 / 模拟串口单元测试，包括厂家示例向量、32 位角度、噪声 / 分包、
校验错误、超时、串口断线、短写及动作不重放，以及真实 ROS 服务的试运行集成测试。
Windows 原生环境：27 项测试通过，3 项 ROS 测试跳过。
本机 WSL / ROS 2 Humble：两个包构建成功，30 项测试全部通过，0 错误、0 失败、0 跳过；
launch 试运行可正常启动和退出。所有 Python 文件通过语法解析，两个 package.xml 通过 XML 解析。
首次软件验证时未连接真机；后续实机记录见下节。

## 8. 本机一次开合记录（2026-10-10）

CH9102（序列号 `EXAMPLE_DEVICE_SERIAL`）由 Windows COM3 / BUSID `4-3` 转接至 WSL `/dev/ttyACM0`。
WSL 用户 `estun_1` 已加入 `dialout` 组。通过 ROS 2 服务各发送一次动作：

| 阶段 | 指令 / 反馈 |
| --- | --- |
| 初始 | 角度 21.0°，速度 0，target_reached=true |
| 闭合指令 | `7B 01 02 01 20 46 50 01 2C 62 7D`（1800°，30 rad/s） |
| 闭合后 | 角度 1623.5°，连续收到速度 0 的反馈，target_reached=false |
| 张开指令 | `7B 01 02 00 20 46 50 01 2C 63 7D`（1800°，30 rad/s） |
| 张开后 | 角度 20.9°，连续收到速度 0 的反馈，target_reached=true |

张开后角度回到起始附近。闭合阶段有实际运动及停止反馈，但没有置位到位标志；
这些数据不能单独证明闭合限位、夹持力或堵转原因，不将 1800° 视为所有型号的标定行程。
本次实际 ROS 控制节点已退出，USB 仍接入 WSL。

为复现保留 `tools/run_one_cycle.py`。该脚本依次调用 ROS 服务，启动后先要求有效反馈，
每次动作仅发送一次，并等待角度变化 / 运动后连续静止反馈。
必须显式传入 `--execute` 才会控制真机；执行失败不会自动重发动作。

```bash
source /opt/ros/humble/setup.bash
source /mnt/d/catch/jia/ros2_ws/install_wsl/setup.bash
python3 /mnt/d/catch/jia/ros2_ws/tools/run_one_cycle.py \
  --execute --port /dev/ttyACM0 --turns 5.2 --speed 20.0
```

上面的命令会再次产生真实开合，仅在需要重复试验且运动范围已清空时运行。
该命令现采用用户指定的手册参数；上表的 1800°、30 rad/s 保留为历史测试记录。
