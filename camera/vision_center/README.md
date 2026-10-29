# 三维中心点

`grasp_candidate.py` 按“轮廓外接矩形中心像素 + 整个轮廓内部有效深度中位数”计算单物体候选点；`candidate_pipeline.py` 复用已有手眼标定和采集时位姿转换到基座；`candidate_cli.py` 离线验证并输出全部采样像素证据。[操作说明](../docs/grasp_candidate.md)。轮廓重心、中心局部深度和内部点回退算法已删除。

`ros2/camera_center_test` 是独立 ROS2 包，显示彩色图像中心、邻域有效深度中位数和相机坐标（米），检查无效深度、图像几何和同步问题。

它不识别物体类别，也不自动选择可夹持部位。已有医疗项目负责物体识别；`adapters.py` 对接其 `position_3d` / `position_xyz` 毫米坐标。CLI 的 `medical-observation` 要求显式确认识别源图匹配、深度几何已核对，并补充同步位姿与相机元数据。

完整的物体识别模型到实际定位精度尚未端到端验证。
