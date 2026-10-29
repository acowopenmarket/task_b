"""Verbatim pure helpers from the existing robot project; no networking or motion runner."""
from dataclasses import dataclass
import math


def finite(values, length):
    return len(values) == length and all(
        isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
        for v in values)


def quaternion(rpy):
    # 官方 X-Y-Z 固定角：R = Rz(rz) Ry(ry) Rx(rx)，四元数顺序为 w,x,y,z。
    cr, cp, cy = [math.cos(v / 2) for v in rpy]
    sr, sp, sy = [math.sin(v / 2) for v in rpy]
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)


def euler_near(q, previous):
    """选择与上一目标最接近的等价欧拉角，避免跨 ±π 时数值跳变。"""
    w, x, y, z = q
    sin_pitch = max(-1, min(1, 2*(w*y-z*x)))
    if abs(sin_pitch) > 1 - 1e-12:
        # 万向节锁处固定上一个 roll，解出 yaw；不把三轴分别线性插值。
        pitch = math.copysign(math.pi/2, sin_pitch)
        roll = previous[0]
        delta = math.atan2(-2*(x*y-w*z), 1-2*(x*x+z*z))
        yaw = delta + math.copysign(1, sin_pitch)*roll
    else:
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    candidates = [(roll, pitch, yaw), (roll+math.pi, math.pi-pitch, yaw+math.pi)]
    def near(values):
        return tuple(v + 2*math.pi*round((p-v)/(2*math.pi))
                     for v, p in zip(values, previous))
    return min(map(near, candidates),
               key=lambda r: sum((a-b)**2 for a, b in zip(r, previous)))


@dataclass(frozen=True)
class Settings:
    linear_speed: float = 0.005
    angular_speed: float = math.radians(2)
    linear_accel: float = 0.02
    angular_accel: float = math.radians(5)
    position_tolerance: float = 0.0005
    angle_tolerance: float = math.radians(0.5)
    feedback_timeout: float = 0.5
    accept_timeout: float = 3.0
    segment_timeout: float = 120.0  # 当前整个阶段的超时；保留旧配置字段名。
    settle_seconds: float = 0.2
    settle_samples: int = 3

    def __post_init__(self):
        for name, value in vars(self).items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} 必须是有限正数')
        if not isinstance(self.settle_samples, int) or self.settle_samples < 2:
            raise ValueError('settle_samples 必须是至少 2 的整数')
        if self.accept_timeout >= self.segment_timeout:
            raise ValueError('accept_timeout 必须小于 segment_timeout')


@dataclass(frozen=True)
class Segment:
    stage: str
    pose: tuple


def command(segment, settings):
    # 官方协议：笛卡尔速度/加速度各为两个数，不是六个数。
    return {'joint_names': ['x', 'y', 'z', 'rx', 'ry', 'rz'], 'points': [{
        'positions': list(segment.pose),
        'velocities': [settings.linear_speed, settings.angular_speed],
        'accelerations': [settings.linear_accel, settings.angular_accel]}]}
