# taskB 简洁操作指导

公开版本的 confirmed 项默认关闭；完成现场核对后由操作者逐项确认。运行时检查仍由原程序执行。

识别范围就是相机整幅画面，使用深度区域与彩色分割一致性定位，复用原坐标转换。无需识别箱子、划分四角/中心、测固定箱位、指定颜色或拍空箱照片。

1. 放好单个物体。用示教器到已设置的观察/释放位，确保完整看见有效源区域、返回路径有净空，下方能接收释放物体。核对真实标定、夹爪原参数、网络串口，退出重复相机/bridge/夹爪程序。
2. 启动 Windows 只读反馈转发器及现有 WSL 配套环境后，在 task_B 目录运行 `bash run_task_b.sh --capture`。以 result.json 为准，应为 PRESENT、localizable=true、top_valid=true，且成功生成 target。图像仅用于查看，运行放行仍由程序判断，不人工指定目标或修改阈值。
3. 准备就绪且检查通过后运行 `bash run_task_b.sh --execute`。它重新采集，再按默认姿态接近、下降到位、闭合并等电机完成、先竖直抬升再返回观察位，重新拍照复查。默认最多额外2次抓取，重试重新定位。

- ABSENT：在观察位松爪结束。
- PRESENT 且目标距相机深度小于 171 mm：一次有效的多帧识别确认后，即判成功，在观察位松爪结束。距离使用识别物体区域有效深度中位数 `rgbd_candidate.depth_m`（相机光轴深度），严格小于 0.171 米；等于 171 mm 不满足。
- PRESENT 且不满足上述近距离条件：在观察位松爪，重新定位后再抓。
- UNKNOWN：只采集或超时退出，不作为成功，不盲目重抓。

当前算法不拟合桌面，不因候选区域触及画面边缘一律拒绝。NO_MATCHING_RGBD_OBJECT 表示未找到彩色与深度匹配、且与周围深度分离的目标；不能当作抓取成功。复查只有原目标位置得到有效、更远的深度支持才确认移除。

2026-10-29 增加近距离成功条件，与原 ABSENT 条件为“或”关系，仅用于完成抓取并返回观察位后的新鲜复查。ABSENT 保留原有连续确认次数，UNKNOWN 会中断该连续确认；近距离条件只需一次有效的多帧识别结果，内部多帧稳定性及同步校验保持不变。结果 `success_reason` 区分 `SOURCE_TARGET_ABSENT` 与 `TARGET_CAMERA_DEPTH_BELOW_171MM`，并保留实际深度和阈值；`object_held_verified` 仍为 false。新条件尚未实机验证。

四角和中心只作为人工五次验收点，同一套算法处理，不切换程序区域。每次保留runs目录日志。

2026-10-28 实机流程已成功完成一次：runs/20261028_134952_1341/result.json 返回 success=true、attempts=1，两次复查均为 ABSENT / PREVIOUS_TARGET_DEPTH_CLEARED，松爪完成。成功含义为原目标源区域已清空；object_held_verified=false，不代表传感器已验证持物或全程未滑落。

## 本机复现步骤

当前配置 min_flange_z_m=0.214（2026-10-29 增加 1 毫米）、feedback_timeout=0.5，使用 config.json 已保存的观察位。不得降低坐标模块下限或延长超时来绕过错误。修复后的 runner 持有固定执行器并处理一批已就绪回调，hardware 在图像分析期间继续处理反馈。

1. Windows PowerShell 查看设备和 WSL 地址：

```powershell
usbipd list
wsl.exe -- hostname -I
wsl.exe -- ip -4 route show default
```

2. 将相机和夹爪串口挂载到 WSL。本次相机 BUSID 为 3-4，夹爪为 5-3；若编号变化，以 list 的实际对应设备为准：

```powershell
usbipd attach --wsl --busid 3-4
usbipd attach --wsl --busid 5-3
```

3. 在 Windows PowerShell 运行现有只读反馈转发器并保持运行。下面 IP 是本次 WSL 地址；重启后如变化，使用第 1 步的地址。config.json 的 feedback_source_ip 对应 WSL 默认网关，本次为 198.51.100.1。

```powershell
& 'C:\Users\1\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' D:\catch\estun\windows_feedback_relay.py --wsl-ip 198.51.100.2
```

4. 关闭独立相机预览、旧 bridge 和夹爪节点，避免占用设备。WSL Bash 中运行：

```bash
cd /mnt/d/catch/task_B
bash run_task_b.sh --capture
```

5. 程序只读结果满足第 2 条后，执行一次：

```bash
bash run_task_b.sh --execute
```

程序会重新采集，不使用上次坐标。等待最终日志目录中的 result.json；只有 success=true 才按程序标准记为成功。正常结束后相机和 bridge 自动清理，在 Windows 转发器终端按 Ctrl+C 结束转发。失败时保留 events.jsonl、result.json、camera.log、bridge.log 和配置快照，不自动重发。

需要调整盒子时，任务结束后可在两个 WSL 终端分别运行现有只读预览：

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 bash /mnt/d/catch/camera/scripts/run_camera.sh
```

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 bash /mnt/d/catch/camera/scripts/run_center_test.sh --ros-args -p show_gui:=true -p duration_sec:=0.0
```

调整后先退出这两个程序，再运行 task_B，避免重复相机驱动。

## 相机初始化超时排查

若 camera.log 出现 `uvc_open failed`，先检查是否有独立相机预览或旧驱动占用 USB。不同 ROS_DOMAIN_ID 的驱动不会被 task_B 的主题唯一性检查发现，但仍会争用同一物理设备。不要在预览驱动保持运行时启动 --capture 或 --execute；先在原驱动终端按 Ctrl+C，等待退出，再重新采集。仅关闭图像显示窗口不一定会结束相机驱动。

2026-10-28 14:15 的故障已确认是域 87 的旧相机驱动占用设备；结束该驱动后，14:18:50 的只读采集正常，结果目录 runs/20261028_141850_21635。

