# task_B 本机操作页面

双击 `D:\catch\task_B\启动操作面板.cmd`，保持启动窗口运行，浏览器自动打开 `http://127.0.0.1:8765`。也可以把这个地址加入书签；每次使用前先启动服务。

## 两个按钮

- **回观察位**：从 config.json 读取保存的 observation_pose 和当前运动参数，使用现有 estun_segmented 控制器返回，到位后打开夹爪，恢复初始状态。当前速度为平移50毫米/秒、旋转6度/秒。先挂载并查询夹爪串口，串口不可用时不移动机械臂；不启动相机。机械臂顺序沿用原程序：姿态→XY→Z，容差内的阶段跳过；不会自动先抬升。到位复核成功后只发送一次开爪指令，等待原驱动的电机完成反馈；开爪失败时页面报告失败，不重发。
- **开始 task_B**：挂载已共享的相机与夹爪 USB，启动原 Windows 反馈转发器，并运行现有 `bash run_task_b.sh --execute`。重新采集，不使用历史坐标。一次点击只启动一轮现有任务；任务内部仍按原配置最多额外重试2次。请先到观察位再开始。

启动页面、刷新页面都不会启动设备或发送动作。两个按钮互斥，运行期间不可重复点击。失败后不会自动返回或重新执行；看清失败原因后，可单独点击“回观察位”，到位后再按“开始 task_B”。页面关闭再打开不会重新执行任务。

程序结果保存在 `task_B\runs\panel_日期时间_编号\program\result.json`；同目录 events.jsonl、bridge.log、camera.log 由现有程序生成。上一级 console.log 和 relay.log 记录完整控制台与转发器输出。页面实时显示最近200条日志，原始输出仍完整写入文件。

## 出错时

- `Idle` / `PowerOff`：用示教器或控制器界面按现场流程切换到 AutoReady，再点击操作。
- 旧会话或端口占用：退出手动开启的转发器、bridge、task_B、相机预览程序，再操作；页面不会强杀其他会话。
- USB 未共享：按原设备安装流程使用 usbipd 共享相机与夹爪。页面只挂载已共享的设备，不修改驱动或系统权限。
- WSL 网关与配置不一致：核对 config.json 的 hardware.feedback_source_ip。转发目标会按当前 WSL 地址自动获取，不使用旧地址。
- 任意程序中止：页面显示原 reason；不会忽略错误、降低阈值或自动重发。

故障后软件退出不保证控制器已经停止；需要停止机械臂时使用现场控制器停止按钮或硬件急停。运行中不要关闭服务窗口。页面不提供完整碰撞规划，使用前按原现场约定核实运动路径。

## 实现与测试边界

页面仅监听本机127.0.0.1，不开放局域网或互联网。动作接口只接受本页面会话和两个固定操作，不接受任意命令。每轮独占操作槽，并复用 task_B 的 WSL 设备锁及原控制器的状态、通信唯一性和到位校验。

页面服务本身不改变 config.json、视觉算法、重试次数或控制器参数。

2026-10-29 更新：回观察位后打开夹爪，结果中 observation_reached 与 gripper_open_completed 分别记录两步是否完成；二者均完成才报告成功。Windows接口/调度测试9项、WSL观察位及原硬件/反馈测试15项通过，测试日志为 runs/control_panel_tests_20261029.txt、runs/observation_gripper_tests_20261029.txt。均使用模拟设备，本次未执行实机动作。

仅测试交互可在 Windows PowerShell 运行：

```powershell
& 'C:\Users\1\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' D:\catch\task_B\control_panel_server.py --dry-run --port 8766
```

此模式顶部显示“页面测试模式”，点击不会连接WSL或设备；不得用测试模式的成功提示当作实机到位或抓取成功。
