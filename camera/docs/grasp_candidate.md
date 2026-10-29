# 外接矩形中心与完整物体区域深度中位数

按2026-10-24操作者确认的规则计算。已删除轮廓重心、中心附近小区域中位数和向内部可靠点移动的选点算法。现有机械臂与夹爪控制代码、手眼标定及法兰补偿不变，不发送运动指令。

## 唯一选点规则

1. 圈出物体外部轮廓。继续使用深度种子引导的 RGB GrabCut 区域分割，归并同一物体的深度碎片，不使用颜色标签或物品类别模型。不完整边缘区域会编号并记录排除原因；多个完整物体则失败。
2. 计算轴对齐外接矩形，取中心像素 `(u,v)`。本实现以像素中心的包围范围 `[xmin,ymin,xmax,ymax]` 为边界，`u=(xmin+xmax)/2`，`v=(ymin+ymax)/2`，保留半像素精度。即使中心位于孔洞、背景或缺失深度处，也绝不移动该中心。
3. 将外轮廓圈住的**全部区域（包含内部）**作为深度采样范围，不只取轮廓线、不取矩形中的背景、不取中心局部窗口。有效样本为正数、有限数、位于已配置离桌高度范围内的深度点；零值、NaN/Inf、桌面/范围外数据不参加统计。当前配置范围为离桌8 mm至0.4 m。
4. `Zc=median(全部有效区域深度)`；以 `(u,v,Zc)` 和深度 CameraInfo 反投影得到相机三维坐标。内参无畸变时 `Xc=(u-cx)*Zc/fx`，`Yc=(v-cy)*Zc/fy`。XY并非直接把像素值当作机器人坐标。
5. 复用采集时法兰位姿、现有手眼矩阵转换到基座。该点是“外接矩形中心射线＋区域深度中位数”构造的坐标，不代表中心像素的直接测距或物体内部真实几何中心。

不对区域深度做平均、不进行 MAD 裁剪来改变中位数、不填充缺失深度。每帧有效点不足20个则失败。使用当前帧的区域中位数作为输出 Z；至少3帧的区域中位数范围需不超过6 mm，其他帧只检查稳定性，不再对多帧中位数做第二次中位数。失败不输出默认坐标。

## 接口与运行

```python
from vision_center.grasp_candidate import compute_candidate
from vision_center.candidate_pipeline import add_base_coordinates

candidate, masks = compute_candidate(raw_frames, depth_encoding, depth_info, rgb=bgr_image)
result = add_base_coordinates(candidate, capture_context, depth_info,
                              calibration, robot_convention, config)
```

`raw_frames` 是静止连拍的 `[N,H,W]` 深度。`bgr_image`、相机内参和目标位姿应对应最后一帧。现有同步、故障、连拍位姿变化量和标定匹配检查保持。

```bash
python3 -m vision_center.candidate_cli \
  --snapshot results/current_candidate_20261024/snapshot.json \
  --depth-series results/current_candidate_20261024/depth_series.npy \
  --image results/current_candidate_20261024/color.png \
  --depth-info results/current_candidate_20261024/depth_info.json \
  --calibration results/handeye_real_20261023/calibration.json \
  --output results/current_candidate_20261024/bbox_region_median
```

`--show` 显示预览，按任意键关闭。CLI 不启动相机、机械臂反馈桥或运动进程。旧纸盒数据也可按相同参数格式读取；旧批次未逐帧保存 CameraInfo 的证据限制仍需保留。

## 输出证据

- `measurement.json`：矩形像素范围、固定中心像素、区域深度中位数、所有参加统计的深度值、相机/基座坐标、有效点数、每帧统计及稳定性。`point_kind=bbox_center_region_depth_candidate`，`center_relocated=false`，`center_pixel_depth_used=false`。
- `overview.png`：绿色物体轮廓和外接矩形、红色中心十字、青色参与中位数的全部有效区域像素。
- `object_mask.png` / `object_edge_mask.png`：轮廓填充区域和仅供显示的轮廓线。深度统计使用前者，绝不使用后者。
- `region_valid_depth_mask.png`：当前帧实际参加深度中位数的像素。
- `region_depth_support.png`：目标区域采样放大图。

保存的瓶子数据得到外接矩形范围 `[43,70,542,234]`、中心 `(292.5,152)`、有效深度点13488、中位深度0.257 m。中心固定在瓶身图像中部，不回退到瓶盖。这个样本有效深度只覆盖物体区域的一部分；显示采样掩码以供核对，不能将中位深度代表性直接当作实机抓取精度验证。

运行 `python3 -m unittest discover -s tests -v` 检查完整区域参与中位数、矩形中心固定、缺失中心深度不回退、单物体限制、同步及坐标链。当前52项离线测试通过。
