# 图像分析后 bridge 状态过期：原因与修复

2026-10-28。

## 失败表现与代码依据

两次实机均完成初始化开爪，并在首次图像分析返回 PRESENT 后立即报告“配套 bridge 状态缺失或过期”。第二次证据为 runs/20261028_133930_2874/events.jsonl：捕获起始 monotonic=473.348096997，OBSERVATION=476.177825365，随后 ABORT=476.178559615。

estun_segmented.runner 的状态接收回调 on_status 更新本地 status_time；current 检查本地状态时间是否超过 feedback_timeout=0.5 秒。bridge 每 0.1 秒发布状态。原 task_B.hardware 在同一调用线程同步执行 analyze，期间不调用 ROS spin，因此耗时图像处理会暂停处理反馈回调。返回后只 spin 一次，不保证已处理状态回调，即可能触发上述过期检查。这是明确存在的回调调度缺陷，与两次日志相符；现有日志不能据此排除其他通信故障。

## 已实施修复

hardware.py 新增 analyze_with_feedback：仅将 analyze 计算提交到单个工作线程，调用线程在等待时持续执行原 healthy(camera=False)，保留 ROS 回调、反馈新鲜度、唯一发布者及通信故障检查。计算完成后再检查一次健康状态，随后取回结果；故障和分析异常照常抛出。

未延长 0.5 秒通信超时，未修改识别阈值、成功条件、运动顺序或反馈故障判断。config.json 的法兰 Z 下限仍为用户指定的 0.213 米。未通过重新标记状态时间来伪造新鲜反馈。

## 验证

在现有 WSL 环境运行：

```bash
cd /mnt/d/catch/task_B
source ../camera/scripts/handeye_env.sh
python3 -m unittest -v test_analysis_feedback test_rgbd_candidate
```

8 项通过（Ran 8 tests in 3.014s，OK）。新增 3 项验证耗时 0.65 秒时主线程持续检查、通信故障仍中止、分析异常仍传播；原 RGB-D 5 项测试通过，包括已存真实图像回放。

新增测试已纳入 verify_offline.py。此次未运行完整回归，也未重新连接设备或执行机械臂动作；不能将离线验证记为实机修复成功。下一次应先只读验证分析完成后仍可连续接收实时反馈，再进入实机流程。

## 后续修复：检查前处理已就绪的反馈通道

后续实机在初始化阶段仍出现过期，说明仅将 analyze 放入工作线程不足以覆盖全部路径。

新增真实 ROS 本地回归 test_feedback_dispatch.py，在独立 ROS 域 179 内持续发布 RobotInfo 和 bridge 状态，不建立 UDP 连接。旧实现中暂停回调 0.7 秒后，只调用一次 spin 再检查，复现 MotionError。该问题说明本地缓存过期不等于消息发布端已断流。

runner.py 已改用节点持有的 SingleThreadedExecutor，并在每次 spin 中有限次处理已就绪回调，覆盖多个订阅及执行器任务，随后继续原 current 检查。每次处理后检查锁存故障；不伪造接收时间，不延长 0.5 秒阈值。

修复后本地 ROS 调度 4 项、分析与视觉 8 项、运动单元 24 项通过，现有 UDP 本机回环 14 个场景均 PASS（含断流、错误消息、控制器故障、阶段故障和只读拒发）。本次运行期间更新了 test_local.sh，运行中的脚本读取偏移导致结束时额外出现 `ts.txt: command not found`；各测试结果文件已经完整写出，不能将脚本退出码记为通过。更新后的脚本增加了调度回归入口。

真实只读验证 runs/20261028_134926_1062 返回 live_readonly、PRESENT、localizable=true、top_valid=true，并完成目标计算；未出现 bridge 过期。此后按用户授权启动一次实机流程，证据目录 runs/20261028_134952_1341，最终结果以该目录 result.json 为准。
