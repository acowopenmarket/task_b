# 发布副本验证记录

日期：2026-10-29。下表是在首次整理的本地快照上运行的结果，使用了本地现场配置；随后按用户要求准备公开版本，移除了现场配置和真实标定，替换了现场地址、序列号和模板确认项。当前发布文件校验值见 `SOURCE_MANIFEST.json`，其中标记原样文件与公开化处理文件。
未修改识别或运动算法。所有测试均离线执行，没有启动相机、机器人或夹爪。

| 测试 | 环境 | 结果 |
| --- | --- | --- |
| Task B 原回归套件 | WSL Ubuntu 22.04 / Python 3.10 / numpy 1.21.5 / OpenCV 4.5.4 | 67 项，2 failures、4 errors |
| camera_full | 同上 | 52 项通过 |
| robot | 同上 | 24 项通过 |
| pick | 同上 | 17 项通过 |
| gripper_protocol | 同上 | 16 项通过 |
| gripper_client | 同上 | 11 项通过 |
| test_control_panel | Windows / Python 3.14.5 | 9 项通过 |
| test_control_panel_action | WSL / Python 3.10 | 6 项通过（与面板测试一起执行） |

## Task B 原套件的未通过项

- `test_real_captures_select_object_not_box_or_background`：两个子场景缺少历史 `runs/` 采集目录，报 `StopIteration`，共计两个 errors。
- `test_recorded_small_low_contrast_target_stays_rejected_with_diagnostics`：缺少历史 `records.json`，报 `FileNotFoundError`。
- `test_recorded_review_returns_at_146mm_before_218mm`：缺少历史 `events.jsonl`，报 `FileNotFoundError`。
- `test_recorded_curved_object_and_consecutive_source_context`：未携带历史采集，实际 0 组，断言要求至少 12 组。
- `test_both_limits_and_reference_error`：原测试断言法兰 Z 下限为 `0.213` m，当前 `config.json` 是 `0.214` m，报 `AssertionError: 0.214 != 0.213`。保留当前配置，不为通过测试而更改安全限值。

另外，在 WSL 一起运行操作面板与动作适配器测试时，面板的两个 Windows 专用派发测试因 `subprocess.CREATE_NO_WINDOW` 不存在而失败；同样的 9 个面板测试已在 Windows 重新执行并全部通过。动作适配器的 6 项测试在 WSL 全部通过。

## 本地原始输出

这些日志在发布副本内生成，按 `.gitignore` 留在本地：

- `task_B/runs/verification_20261029_190213/summary.json` 与各套件 `.txt`
- `task_B/runs/control_panel_export_test.txt`
- `task_B/runs/control_panel_windows_export_test.txt`

完整回归目前不能标记为全部通过；本次 Git 发布不代表实机抓取成功。

## 公开模板与离线夹具

公开版本仅提供 `task_B/config.example.json`，真实 `config.json`、标定及 `camera/results/` 不纳入 Git。模板中的观察位、夹爪偏移和安全高度必须填写，所有现场确认项均为 false。`calibration.example.json` 无真实内外参且状态为 unvalidated_template。
相机坐标转换正例测试改为在测试中显式声明合成验证状态，不再依赖模板携带“真实已验证”状态。
公开模板不能直接替代部署配置，配置相关完整回归需要操作者在本地另行提供测试配置。

隐私处理后重新验证：相机合成回归 52 项通过；Windows 操作面板 9 项通过。原始输出分别为本地 `task_B/runs/public_camera_tests.txt` 和 `task_B/runs/public_panel_tests.txt`，不上传采集或运行日志。
