# S3-60 Pro + Gemini 335 相机工具

ROS2 Humble / Ubuntu 22.04 下的 Eye-in-Hand 标定、RGB-D 三维点计算和法兰目标位姿计算。当前阶段只读：不会发送 RobotMove 或夹爪控制命令。

| 模块 | 作用 |
| --- | --- |
| `vision_center/` | 对齐 RGB-D 中心点深度、相机内参反投影；对接已有医疗项目的三维中心输出 |
| `handeye_calibration/` | 棋盘格采集、保存反馈、手眼求解、独立验证样本评估 |
| `coordinate_transform/` | 相机点到基座变换、夹爪偏移补偿、RobotMove 六分量生成 |
| `configs/`、`tests/`、`docs/` | 配置示例、离线测试、使用与验证说明 |

新增[桌面单物体候选点](docs/grasp_candidate.md)：XY 使用物体轮廓的轴对齐外接矩形中心像素，Z 使用该轮廓圈住的整个区域（包含内部）有效深度的中位数，再反投影并转换到基座。已删除轮廓重心、局部小区域深度和内部点回退；中心不移动。保存轮廓、外接矩形、中心及全部有效采样区域，不发送运动指令。

## 使用

```bash
python3 -m pip install -e .
python3 -m unittest discover -s tests -v
python3 -m handeye_calibration.cli --help
```

ROS2 取流需另行安装官方 [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) 的 `v2-main` 驱动和现有 `estun_segmented` 只读反馈桥。详见 [使用说明](docs/usage.md)。本仓库不包含相机 SDK、机械臂主程序、夹爪程序或识别模型。

## 当前验证范围

2026-10-23 的真实棋盘格采集使用 15 个训练姿态和 5 个独立验证姿态；验证角点位置 RMS 约 0.660 mm，深度固定点散布 RMS 约 0.562 mm。详见 [验证记录](docs/validation_2026-10-23.md)。这些是样本一致性指标，不代表实际抓取精度或外部量具测量精度。

向下抓取姿态按 `[-π, 0, 0] rad`，夹爪偏移暂按法兰局部 `[0, 0, 0.15384] m`；实际夹持中心尚待核对。法兰目标低于 Z 下限时，按 `z_output=max(z_calculated,0.213)` 输出 RobotMove 六分量，X、Y 和姿态不变。结果保留原始位姿、限高调整量及夹爪对原目标的偏差；被抬高后不能声称仍到达原目标。所有输出保持 `executable=false` 和 `motion_sent=false`，因为当前程序只计算、不发送运动。

默认以实测物体顶面中心作为夹爪参考点，不再以物体半高或体积中心作为默认目标。输入观测必须标记 `point_kind=top_surface_center`，且坐标确实来自该表面；旧体积中心数据不能直接改标签替代测量。

现场图像、原始采集记录、日志、模型和原医疗项目留在本地并由 `.gitignore` 排除。仓库中的标定示例仅适用于原安装条件，不能直接作为其他安装的已验证标定结果。
