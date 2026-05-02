#!/usr/bin/env python3
"""
stop_scan_node — Stop-Scan FSM skeleton (Phase 1, Week 1)

State machine: HOVER → STOP → SCAN → MOVE → HOVER → ...

Week 1 scope: state transitions only, logged to console.
Week 2+: actual sensor collection, scan storage, SLAM feed.
"""

import rclpy
from rclpy.node import Node
from enum import Enum, auto

from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped


class DroneState(Enum):
    INIT   = auto()
    HOVER  = auto()
    STOP   = auto()
    SCAN   = auto()
    MOVE   = auto()


# Durations in seconds — configurable via ROS2 params
HOVER_DURATION = 5.0
STOP_DURATION  = 2.0
SCAN_DURATION  = 8.0
MOVE_DURATION  = 3.0


class StopScanNode(Node):

    def __init__(self):
        super().__init__('stop_scan_node')

        # Params — override at launch with _param:=value
        self.declare_parameter('hover_duration', HOVER_DURATION)
        self.declare_parameter('stop_duration',  STOP_DURATION)
        self.declare_parameter('scan_duration',  SCAN_DURATION)
        self.declare_parameter('move_duration',  MOVE_DURATION)

        self._state = DroneState.INIT
        self._armed = False
        self._mode  = ''
        self._state_timer = 0.0

        self.sub_mavros_state = self.create_subscription(
            State,
            '/mavros/state',
            self._mavros_state_cb,
            10,
        )

        self.pub_setpoint = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10,
        )

        # 10 Hz control loop
        self.create_timer(0.1, self._tick)

        self.get_logger().info('stop_scan_node started — waiting for GUIDED + ARMED')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _mavros_state_cb(self, msg: State):
        self._armed = msg.armed
        self._mode  = msg.mode
        if self._state == DroneState.INIT and msg.armed and msg.mode == 'GUIDED':
            self._transition(DroneState.HOVER)

    # ── FSM tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        dt = 0.1
        self._state_timer += dt

        if self._state == DroneState.HOVER:
            duration = self.get_parameter('hover_duration').value
            if self._state_timer >= duration:
                self._transition(DroneState.STOP)

        elif self._state == DroneState.STOP:
            duration = self.get_parameter('stop_duration').value
            if self._state_timer >= duration:
                self._transition(DroneState.SCAN)

        elif self._state == DroneState.SCAN:
            duration = self.get_parameter('scan_duration').value
            if self._state_timer >= duration:
                self._transition(DroneState.MOVE)

        elif self._state == DroneState.MOVE:
            duration = self.get_parameter('move_duration').value
            self._publish_hold_setpoint()
            if self._state_timer >= duration:
                self._transition(DroneState.HOVER)

    def _transition(self, new_state: DroneState):
        self.get_logger().info(
            f'FSM: {self._state.name} → {new_state.name}  '
            f'(after {self._state_timer:.1f}s)'
        )
        self._state = new_state
        self._state_timer = 0.0

    def _publish_hold_setpoint(self):
        """Publish a hold-position setpoint (current position) during MOVE."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # TODO Week 2+: fill with actual next waypoint from path planner
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 1.0
        msg.pose.orientation.w = 1.0
        self.pub_setpoint.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StopScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
