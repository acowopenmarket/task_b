# Task B：RGB-D 定位与夹取

当前 `D:\catch\task_B` 及其必要依赖的源代码快照，整理日期：2026-10-29。
主程序、测试、配置模板和操作文档位于 [task_B](task_B/README.md)。
保留原有同级目录结构和相对路径。现场配置及真实标定未上传；代码与文档中的现场 IP、设备序列号和观察位已替换或移除。各文件的发布校验值及处理状态见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。
`192.0.2.*` 与 `198.51.100.*` 均为文档示例地址，文档中的历史“本机”描述不能作为部署参数。

## 目录

- `task_B/`：识别、任务流程、操作面板、配置、离线测试和操作文档。
- `camera/`：RGB-D 候选与坐标转换、手眼标定模块、相机启动脚本及相关测试。
- `estun/`：ROS 2 消息、机器人通信与分段运动模块、反馈转发及离线测试。
- `jia/`：MS42DC 夹爪协议、客户端和 ROS 2 包。
- `try/`：原完整回归所需的联动模块及其离线测试。

不包含运行日志、历史采集图像、离线预览、Python 缓存、安装依赖目录、ROS 构建产物和厂家安装包。
原文档引用的 `runs/` 证据留在原电脑，并未随本仓库上传。依赖模块文档中的历史路径与历史验证描述保留原文。

## 离线验证

建议 Ubuntu 22.04 / WSL、Python 3.10；不需要连接设备。

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
cd task_B
cp config.example.json config.json
python3 verify_offline.py
python3 -m unittest -v test_control_panel_action
```

操作面板服务运行在 Windows，使用 Windows Python 在 `task_B/` 内执行 `python -m unittest -v test_control_panel`。

`verify_offline.py` 运行 Task B、相机、机器人数学/协议、联动、夹爪协议与客户端的离线测试。
这些测试的通过不代表执行了实机抓取。配置模板不包含可用观察位、偏移、安全高度或真实标定，配置相关测试需另行准备本地测试配置；历史数据测试还需要未上传的采集记录。直接执行完整回归不能全部通过，见 [EXPORT_VERIFICATION.md](EXPORT_VERIFICATION.md)。

## ROS 2 环境

实机使用 Ubuntu 22.04 / WSL 与 ROS 2 Humble、Orbbec ROS 2 相机工作空间、cv_bridge、pyserial。
外部 Orbbec 驱动不在本仓库内；沿用现场已验证的安装，并设置 `ORBBEC_WS`。

在已安装 ROS 2 Humble、colcon 和依赖的终端，从仓库根目录构建：

```bash
bash estun/build.sh
(
  source /opt/ros/humble/setup.bash
  cd jia/ros2_ws
  colcon build --symlink-install --build-base build_wsl --install-base install_wsl
)
```

构建不启动设备。`task_B/run_task_b.sh` 使用上述 `install_wsl` 目录。
现场还需依据包声明准备 ROS 2 依赖；详见各模块文档。

## 配置与操作

复制 `task_B/config.example.json` 为本地 `task_B/config.json`，再填写网络地址、串口、观察姿态、夹爪偏移和安全限值。所有现场确认项默认 `false`。
`camera/configs/calibration.example.json` 仅描述结构，状态为 `unvalidated_template`，不含真实矩阵、内参或验证记录，不能用于运动。应采集并验证真实标定，将本地结果保存在 `camera/local/calibration.json`，或修改本地配置的 `hardware.calibration_path`。
上述本地配置、标定和采集目录已被 `.gitignore` 排除。本次发布未重新标定，也未验证实机动作。

- [配置说明](task_B/CONFIGURATION.md)
- [只识别与 IoU](task_B/VISION_ONLY_GUIDE.md)
- [操作面板](task_B/CONTROL_PANEL_GUIDE.md)
- [操作指导](task_B/OPERATION_GUIDE.md)
- [现场检查](task_B/FIELD_CHECKLIST.md)
- [第三方说明](estun/estun_ros2_control/THIRD_PARTY.md)

后续修改本目录内的源文件后，可直接使用 Git 提交和推送。原 `D:\catch\task_B` 的修改不会自动同步到此快照。
