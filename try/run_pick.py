#!/usr/bin/env python3
"""名称 → 预设末端位姿 → 机械臂反馈到位 → 一次夹爪闭合。"""

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'estun/ros2_ws/src/estun_segmented'))

from estun_segmented.core import MotionError, Settings, errors, finite, plan
from estun_segmented.runner import Journal, make_node, preview


def load_target(path, name):
    with open(path, encoding='utf-8-sig') as stream:
        config = json.load(stream)
    if not isinstance(config, dict) or set(config) - {'targets', 'motion'}:
        raise ValueError('配置只支持 targets 和 motion')
    targets = config.get('targets')
    if not isinstance(targets, dict) or name not in targets:
        raise ValueError(f'未配置名称：{name}')
    target = targets[name]
    values = target
    if isinstance(target, dict):
        if set(target) != {'delta'}:
            raise ValueError('相对目标只支持 delta 字段')
        values = target['delta']
    if not isinstance(values, list) or not finite(values, 6):
        raise ValueError(f'{name} 必须对应 [x,y,z,rx,ry,rz] 六个有限数值（m、rad）')
    values = config.get('motion', {})
    if not isinstance(values, dict):
        raise ValueError('motion 必须为配置对象')
    return target, Settings(**values)


def target_pose(target, start):
    if isinstance(target, dict):
        return tuple(a + b for a, b in zip(start, target['delta']))
    return tuple(target)


class Gripper:
    """复用已运行的 jia 节点；动作不重试，不自行打开串口。"""

    def __init__(self, motion):
        from std_srvs.srv import Trigger
        self.motion = motion
        self.request_type = Trigger.Request
        self.query = motion.create_client(Trigger, '/ms42_gripper/query')
        self.close = motion.create_client(Trigger, '/ms42_gripper/close')

    def _call(self, client, label):
        if not client.service_is_ready():
            raise MotionError(f'夹爪 {label} 服务不可用')
        future = client.call_async(self.request_type())  # 只发一次。
        deadline = time.monotonic() + 3.0
        while not future.done():
            if time.monotonic() >= deadline:
                raise MotionError(f'夹爪 {label} 响应超时，发送结果未知；不重发')
            self.motion.spin()
        response = future.result()
        if response is None or not response.success:
            reason = response.message if response is not None else '无响应'
            raise MotionError(f'夹爪 {label} 失败：{reason}')
        return response.message

    def check_ready(self):
        for client in (self.query, self.close):
            if not client.wait_for_service(timeout_sec=3.0):
                raise MotionError('夹爪服务不可用，请先启动 jia 的 gripper_node')
        # 只查询反馈；dry_run 或没有真实反馈时，在机械臂运动前退出。
        self._call(self.query, 'query')

    def close_once(self):
        return self._call(self.close, 'close')


def pick(motion, gripper, target, settings, journal, execute=False):
    if execute:
        gripper.check_ready()
    if isinstance(target, dict):
        # 本次任务只从当前稳定反馈计算一次绝对目标，后续阶段不再累加偏移。
        target = target_pose(target, motion.read_start().pose)
    # move_to 内部逐阶段等待 AutoRunning → AutoReady、误差达标且连续稳定。
    motion.move_to(target, stage='all', execute=execute)
    if not execute:
        journal.write('PICK_PREVIEW', target=target, next_action='/ms42_gripper/close')
        return
    # 包含无需移动的情形；夹取前重新确认新鲜、稳定反馈及完整六维目标。
    feedback = motion.read_start()
    position_error, angle_error = errors(feedback.pose, target)
    if (position_error > settings.position_tolerance
            or angle_error > settings.angle_tolerance):
        raise MotionError('末端未到达设定目标，不发送夹爪闭合指令')
    journal.write('PICK_ARRIVED', actual=feedback.pose,
                  position_error_m=position_error, angle_error_rad=angle_error)
    message = gripper.close_once()
    journal.write('GRIPPER_CLOSE_SENT', message=message,
                  note='闭合指令已写入；现有夹爪接口不确认物体已夹住')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', help='pick_targets.json 中预设的名称')
    parser.add_argument('--config', type=Path, default=ROOT / 'pick_targets.json')
    parser.add_argument('--execute', action='store_true', help='执行机械臂运动和一次夹爪闭合')
    parser.add_argument('--start', nargs=6, type=float, help='仅离线预览：起点，m、rad')
    options = parser.parse_args(args)
    if options.execute and options.start is not None:
        parser.error('--start 仅用于预览，不能与 --execute 同用')
    journal = Journal(None)
    motion = ros = None
    try:
        target, settings = load_target(options.config, options.name)
        journal.write('PICK_TARGET', name=options.name, target=target)
        if options.start is not None:
            target = target_pose(target, options.start)
            preview(journal, plan(options.start, target, settings), settings)
            journal.write('PICK_PREVIEW', target=target, next_action='/ms42_gripper/close')
        else:
            import rclpy as ros
            ros.init(args=[])
            motion = make_node(journal, settings)
            gripper = Gripper(motion) if options.execute else None
            pick(motion, gripper, target, settings, journal, options.execute)
    except (MotionError, ValueError, TypeError, OSError, ImportError, RuntimeError) as exc:
        journal.write('PICK_ABORT', reason=str(exc))
        return 1
    except KeyboardInterrupt:
        journal.write('PICK_ABORT', reason='人工中断，停止后续指令')
        return 130
    finally:
        if motion is not None:
            motion.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
        journal.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
