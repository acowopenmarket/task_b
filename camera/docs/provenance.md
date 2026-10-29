# 依赖与来源

- 相机驱动使用官方 OrbbecSDK_ROS2，外部安装于独立 `~/orbbec_ws`，本仓库不复制其源码或 SDK。
- 机器人姿态与末端字段复用原 `estun_segmented/core.py` 的纯数学实现。快照只包含 `quaternion`、`euler_near`、`finite`、`Settings`、`Segment`、`command`，无发送函数。
- 原文件 SHA256 为 `95bc099e458ad8013bc96f2c13918feedc0cb6cd0bed4a51425b27b0c1e9a6d6`。`math_provenance.json` 记录源文件和所选符号的指纹；AST 指纹允许不同系统换行符而不改变数学约定。
- 姿态依据为 [Codroid S 系列手册](https://www.codroid.ai/download/shouce/v1107.pdf) 第 9.2 节：固定轴 XYZ，对应 `Rz(rz) Ry(ry) Rx(rx)`。该约定也通过真实固定棋盘格的不同姿态观测检查。
- 识别模型由已有医疗项目提供，未上传；这里只提供其坐标输出适配器。
