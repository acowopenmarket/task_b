"""Serial operations independent of ROS; testable with an injected serial port."""

import time

from .protocol import FeedbackParser, move_frame, query_frame


class CommunicationError(OSError):
    pass


class FeedbackTimeout(TimeoutError):
    pass


class GripperClient:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, motor_id=1,
                 subdivision=32, max_angle_deg=1872.0, max_speed_rad_s=20.0,
                 feedback_timeout=0.1, dry_run=True, serial_factory=None):
        # Validate configuration before opening a port or writing any bytes.
        move_frame(motor_id, True, max_angle_deg, max_speed_rad_s, subdivision)
        if not 0.01 <= feedback_timeout <= 2.0:
            raise ValueError('feedback_timeout must be in [0.01, 2.0] seconds')
        if not isinstance(baudrate, int) or isinstance(baudrate, bool) or baudrate <= 0:
            raise ValueError('baudrate must be a positive integer')
        if not isinstance(port, str) or not port:
            raise ValueError('port must be a non-empty string')
        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id
        self.subdivision = subdivision
        self.max_angle_deg = max_angle_deg
        self.max_speed_rad_s = max_speed_rad_s
        self.feedback_timeout = feedback_timeout
        self.dry_run = dry_run
        self.serial_factory = serial_factory
        self.serial_port = None
        self.parser = FeedbackParser(motor_id)

    @property
    def connected(self):
        return self.serial_port is not None and self.serial_port.is_open

    def connect(self):
        if self.dry_run or self.connected:
            return
        factory = self.serial_factory
        if factory is None:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError('Install python3-serial / pyserial for hardware mode') from exc
            factory = serial.Serial
        self.serial_port = factory(
            port=self.port, baudrate=self.baudrate,
            timeout=0, write_timeout=0.2,
        )
        self.parser.clear()

    def disconnect(self):
        port, self.serial_port = self.serial_port, None
        self.parser.clear()
        if port is not None:
            port.close()

    def _write(self, frame):
        if self.dry_run:
            return
        if not self.connected:
            raise CommunicationError('Serial port is disconnected; command was not sent')
        try:
            if self.serial_port.write(frame) != len(frame):
                raise CommunicationError('Incomplete serial write; motion outcome is unknown')
        except OSError:
            self.disconnect()
            raise

    def move(self, close, angle_deg, speed_rad_s):
        frame = move_frame(self.motor_id, close, angle_deg, speed_rad_s, self.subdivision)
        if angle_deg > self.max_angle_deg:
            raise ValueError(f'angle_deg exceeds configured travel limit {self.max_angle_deg}')
        if speed_rad_s > self.max_speed_rad_s:
            raise ValueError(f'speed_rad_s exceeds configured limit {self.max_speed_rad_s}')
        self._write(frame)
        return frame

    def query(self):
        if self.dry_run:
            raise FeedbackTimeout('Dry-run does not fabricate hardware feedback')
        if not self.connected:
            raise CommunicationError('Serial port is disconnected')
        try:
            # Discard stale/partial input before a new request. The wire format
            # has no transaction ID, so a very late reply can still be ambiguous.
            self.serial_port.reset_input_buffer()
            self.parser.clear()
            self._write(query_frame(self.motor_id))
            deadline = time.monotonic() + self.feedback_timeout
            while time.monotonic() < deadline:
                chunk = self.serial_port.read(min(4096, max(1, self.serial_port.in_waiting)))
                frames = self.parser.feed(chunk)
                if frames:
                    return frames[-1]
                if not chunk:
                    time.sleep(min(0.002, max(0.0, deadline - time.monotonic())))
        except OSError:
            self.disconnect()
            raise
        raise FeedbackTimeout('No valid 9-byte feedback before timeout')
