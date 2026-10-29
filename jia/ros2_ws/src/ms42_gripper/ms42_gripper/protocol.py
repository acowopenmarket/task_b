"""Wire format from MS42DC manual, sections 3.1 and 3.6.

Command: 7b ID MODE DIR SUB ANGLE(2 BE) SPEED(2 BE) XOR 7d.
Feedback: ID REACHED SPEED(2 BE) ANGLE(4 BE) XOR; no delimiters.
"""

from dataclasses import dataclass
from functools import reduce
import math
from operator import xor


def bcc(data: bytes) -> int:
    return reduce(xor, data, 0)


def validate_id(motor_id: int) -> None:
    if isinstance(motor_id, bool) or not isinstance(motor_id, int):
        raise ValueError('motor_id must be an integer')
    if not 1 <= motor_id <= 255:
        raise ValueError('motor_id must be in [1, 255]; broadcast is unsupported')


def scaled_u16(value: float, name: str, minimum: float, maximum: float) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be numeric')
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be finite and in [{minimum}, {maximum}]')
    # The protocol resolution is 0.1; round half up, avoiding Python banker's rounding.
    raw = int(math.floor(value * 10.0 + 0.5))
    if not 0 <= raw <= 65535:
        raise ValueError(f'{name} exceeds the 16-bit wire range')
    return raw


def move_frame(motor_id: int, close: bool, angle_deg: float,
               speed_rad_s: float, subdivision: int = 32) -> bytes:
    validate_id(motor_id)
    if not isinstance(close, bool):
        raise ValueError('close must be bool')
    if (isinstance(subdivision, bool) or not isinstance(subdivision, int)
            or subdivision not in (2, 4, 8, 16, 32)):
        raise ValueError('subdivision must be 2, 4, 8, 16 or 32')
    angle = scaled_u16(angle_deg, 'angle_deg', 0.1, 6553.5)
    speed = scaled_u16(speed_rad_s, 'speed_rad_s', 0.1, 42.0)
    payload = (bytes([0x7B, motor_id, 0x02, int(close), subdivision])
               + angle.to_bytes(2, 'big') + speed.to_bytes(2, 'big'))
    return payload + bytes([bcc(payload), 0x7D])


def query_frame(motor_id: int) -> bytes:
    validate_id(motor_id)
    payload = bytes([0x7B, motor_id, 0, 0, 0, 0, 0, 0, 0])
    return payload + bytes([bcc(payload), 0x7D])


@dataclass(frozen=True)
class Feedback:
    motor_id: int
    target_reached: bool
    raw_speed: int
    raw_angle: int

    @property
    def speed_rad_s(self) -> float:
        return self.raw_speed / 10.0

    @property
    def angle_deg(self) -> float:
        return self.raw_angle / 10.0


class FeedbackParser:
    """Handle partial/concatenated frames, noise and checksum errors.

    There is no header in this protocol. ID, flag and XOR are the only
    resynchronization checks; they cannot guarantee rejection of all noise.
    """

    def __init__(self, motor_id: int):
        validate_id(motor_id)
        self.motor_id = motor_id
        self.buffer = bytearray()

    def clear(self) -> None:
        self.buffer.clear()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 9:
            frame = bytes(self.buffer[:9])
            if (frame[0] == self.motor_id and frame[1] in (0, 1)
                    and bcc(frame[:8]) == frame[8]):
                frames.append(Feedback(
                    frame[0], bool(frame[1]),
                    int.from_bytes(frame[2:4], 'big'),
                    int.from_bytes(frame[4:8], 'big'),
                ))
                del self.buffer[:9]
            else:
                del self.buffer[0]
        return frames
