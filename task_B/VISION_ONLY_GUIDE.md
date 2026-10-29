# 只识别一次并查看 IoU

机械臂可以保持关闭。本模式仅采集相机 RGB-D，不启动机器人 bridge、反馈转发器或夹爪，不产生机器人基座坐标或运动目标。

## 启动

先打开一个 WSL 终端并保持打开。Windows PowerShell 运行：

```powershell
usbipd list
usbipd attach --wsl --busid 3-4
```

BUSID 3-4 为本机当前相机；若列表变化，使用列表中 Orbbec 相机的 BUSID。仅当尚未 Attached 时挂载。不要同时运行另一个相机驱动或预览。

在 WSL 终端中运行一次：

```bash
cd /mnt/d/catch/task_B
bash run_task_b.sh --vision-only
```

程序按配置采集三帧 RGB-D，完成识别后退出并清理自己的相机进程。每次换好物体后重复这条命令即可。

## 结果

控制台显示 RESULT 和日志目录；目录内 result.json 中：

- mode=camera_only：纯相机模式。
- presence=PRESENT、localizable=true：程序得到一个符合当前条件的候选。
- rgb_depth_iou：本次彩色区域与深度区域 IoU。
- point_camera_m：相机光学坐标，米；不是机械臂基座坐标。
- robot_feedback_required=false、motion_sent=false。

UNKNOWN 或中止时按 reason 报告失败。未产生 rgb_depth_iou 时，没有本次有效 IoU，不能填 0 或复用上一次值。

IoU 是同次采集中彩色与深度区域的一致性指标，不是物体类别识别准确率。现有识别器输出一个最高评分候选。

## 本次验证

2026-10-28，机器人反馈为 PowerOff 后停止使用依赖机器人的 --capture；不要求用户启用机器人。完成独立模式后，以 --vision-only 实际采集成功，IoU=0.7864693446088795。日志目录：runs/20261028_151055_675。
