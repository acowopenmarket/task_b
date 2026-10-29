# 配置说明

vision 新定位参数：

- depth_layer_m=0.012：相邻有效深度像素可连接的最大距离差，也是备用深度分层步长（相邻窗口重叠），米。曲面总深度跨度可大于该值；无效像素不建立连接。
- rgb_depth_min_iou=0.6：彩色区域与深度区域交并比下限（2026-10-28 用户由0.75调整为0.6）。
- min_depth_separation_m=0.004：候选区域与周边的最小可测距离差，米；不拟合桌面。
- min_object_area_ratio=0.001：小噪声连通区域过滤比例。
- min_depth_points=30：目标有效测量数下限，不是全画面有效比例。
- target_temporal_span_m=0.006：多帧目标深度中位数变化上限，米。
- source_clearance_depth_m=0.015：复查原目标区域的深度变远量，米。

原 frames、confirm_bursts、timeout、同步/标定和顶部连续性参数继续使用。已删除 min_valid_ratio、min_floor_ratio_absent、floor_tolerance_m，定位不要求桌面或全图完整深度。

观察位、真实标定、机器人/串口、夹爪偏移、固定姿态和速度参数沿用现场配置。max_insert_depth_m=0.044；min_flange_z_m=0.214（2026-10-29 用户要求在 0.213 米基础上增加 1 毫米）。该值高于现有 coordinate_transform.geometry 模块的 0.213 米强制下限。

box.rim_z_m 仅保留可选人工运动高度信息，不参与识别；全画面深度推算的障碍高度阻断已删除。workspace_bounds 为可选运动范围。现有程序不提供完整碰撞检测。
