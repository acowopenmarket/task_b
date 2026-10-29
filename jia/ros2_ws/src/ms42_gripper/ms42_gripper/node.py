"""ROS 2 service/topic facade; all serial callbacks share one callback group."""

import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from ms42_gripper_interfaces.msg import GripperState
from ms42_gripper_interfaces.srv import Move

from .client import FeedbackTimeout, GripperClient


class GripperNode(Node):
    def __init__(self):
        super().__init__('ms42_gripper')
        defaults = {
            'port': '/dev/ttyACM0', 'baudrate': 115200, 'motor_id': 1,
            'subdivision': 32, 'travel_turns': 5.2, 'speed_rad_s': 20.0,
            'max_speed_rad_s': 20.0, 'poll_hz': 5.0,
            'feedback_timeout_s': 0.1, 'feedback_stale_s': 1.0,
            'reconnect_interval_s': 2.0, 'dry_run': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        p = {name: self.get_parameter(name).value for name in defaults}
        numeric = ('travel_turns', 'speed_rad_s', 'max_speed_rad_s', 'poll_hz',
                   'feedback_timeout_s', 'feedback_stale_s', 'reconnect_interval_s')
        if any(not math.isfinite(p[name]) or p[name] <= 0 for name in numeric):
            raise ValueError('Timing, travel and speed parameters must be finite and positive')
        if p['poll_hz'] > 20.0 or p['feedback_timeout_s'] >= 1.0 / p['poll_hz']:
            raise ValueError('poll_hz must be <=20 and feedback_timeout_s < 1/poll_hz')
        self.travel_deg = p['travel_turns'] * 360.0
        self.speed = p['speed_rad_s']
        if self.speed > p['max_speed_rad_s'] or self.speed < 0.1:
            raise ValueError('speed_rad_s must be >=0.1 and <=max_speed_rad_s')
        self.stale_s = p['feedback_stale_s']
        self.reconnect_s = p['reconnect_interval_s']
        self.client = GripperClient(
            port=p['port'], baudrate=p['baudrate'], motor_id=p['motor_id'],
            subdivision=p['subdivision'], max_angle_deg=self.travel_deg,
            max_speed_rad_s=p['max_speed_rad_s'],
            feedback_timeout=p['feedback_timeout_s'], dry_run=p['dry_run'],
        )
        self.last_feedback = None
        self.last_connect_attempt = -math.inf
        self.last_warning = -math.inf
        self.state_pub = self.create_publisher(GripperState, '~/state', 10)
        self.connected_pub = self.create_publisher(Bool, '~/serial_connected', 10)
        self.alive_pub = self.create_publisher(Bool, '~/feedback_alive', 10)
        self.create_service(Trigger, '~/open', self._open)
        self.create_service(Trigger, '~/close', self._close)
        self.create_service(Trigger, '~/query', self._query)
        self.create_service(Move, '~/move', self._move)
        self.create_subscription(Bool, '~/command', self._command, 1)
        self.timer = self.create_timer(
            1.0 / p['poll_hz'], self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        if self.client.dry_run:
            self.get_logger().info('DRY RUN: frames are logged; no serial port or motor is used')
        else:
            self.get_logger().info(f'Hardware mode: {p["port"]}; no automatic startup motion')
        self._connect()

    def _warn(self, message):
        now = time.monotonic()
        if now - self.last_warning >= 5.0:
            self.get_logger().warning(message)
            self.last_warning = now

    def _connect(self):
        if self.client.dry_run or self.client.connected:
            return
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_s:
            return
        self.last_connect_attempt = now
        self.last_feedback = None
        try:
            self.client.connect()
            self.get_logger().info('Serial port opened; waiting for valid device feedback')
        except (OSError, RuntimeError) as exc:
            self._warn(f'Cannot open serial port: {exc}')

    def _send(self, close, angle, speed):
        # Do not reconnect/retry/replay movement commands: delivery is one-shot.
        frame = self.client.move(close, angle, speed)
        mode = 'DRY RUN' if self.client.dry_run else 'TX'
        self.get_logger().info(f'{mode}: {frame.hex(" ").upper()}')
        return ('Dry-run frame logged; hardware was not controlled' if self.client.dry_run
                else 'Frame written; this does not confirm motion completion')

    def _motion_response(self, close, angle, speed, response):
        try:
            response.message = self._send(close, angle, speed)
            response.success = True
        except (ValueError, OSError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _open(self, request, response):
        return self._motion_response(False, self.travel_deg, self.speed, response)

    def _close(self, request, response):
        return self._motion_response(True, self.travel_deg, self.speed, response)

    def _move(self, request, response):
        return self._motion_response(request.close, request.angle_deg,
                                     request.speed_rad_s, response)

    def _command(self, msg):
        try:
            self._send(msg.data, self.travel_deg, self.speed)
        except (ValueError, OSError) as exc:
            self.get_logger().error(f'Movement command rejected: {exc}')

    def _publish_feedback(self, feedback):
        msg = GripperState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.motor_id = feedback.motor_id
        msg.target_reached = feedback.target_reached
        msg.raw_speed = feedback.raw_speed
        msg.raw_angle = feedback.raw_angle
        msg.motor_speed_rad_s = feedback.speed_rad_s
        msg.motor_angle_deg = feedback.angle_deg
        self.state_pub.publish(msg)
        self.last_feedback = time.monotonic()

    def _query(self, request, response):
        try:
            self._publish_feedback(self.client.query())
            response.success = True
            response.message = 'Valid feedback received and published on ~/state'
        except OSError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _tick(self):
        self._connect()
        if self.client.connected:
            try:
                self._publish_feedback(self.client.query())
            except FeedbackTimeout as exc:
                self._warn(f'Feedback timeout: {exc}')
            except OSError as exc:
                self.last_feedback = None
                self._warn(f'Serial error; movement commands will not be replayed: {exc}')
        self.connected_pub.publish(Bool(data=self.client.connected))
        alive = (self.client.connected and self.last_feedback is not None
                 and time.monotonic() - self.last_feedback < self.stale_s)
        self.alive_pub.publish(Bool(data=alive))

    def destroy_node(self):
        # The manuals do not define a reliable position-mode STOP command.
        # Closing a port is NOT a stop command; do not invent a speed-mode stop.
        try:
            self.client.disconnect()
        except OSError as exc:
            self.get_logger().warning(f'Error closing serial port: {exc}')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GripperNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
