"""Failure modes — sprint plan v2 sim Задача 5 + ack orch 21:00.

Default behavior — `hover_and_wait` (НЕ land), потому что landing в Gazebo на
препятствие может уронить дрон. Только mavros_disconnect → land_immediately
(SITL EKF исчез — других вариантов нет).

Coverage stall: Δcoverage < ε за окно N секунд → hover_and_wait.
ε / window — параметры (orch 21:00: `ε = 0.005`, window = 30s).
"""
from __future__ import annotations

import collections
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist


class FailureHandler:
    """Coverage-stall детектор + hover_and_wait emitter.

    Bridge main loop вызывает `check(coverage)` per step. Если возвращает True
    → switch в hover_mode (publish zero velocity continuously, без вызовов predict).
    """

    def __init__(
        self,
        node: Node,
        *,
        stall_epsilon: float = 0.005,
        stall_window_s: float = 30.0,
        cmd_vel_topic: str = "/mavros/setpoint_velocity/cmd_vel_unstamped",
    ) -> None:
        self.node = node
        self.stall_epsilon = stall_epsilon
        self.stall_window_s = stall_window_s
        self._history: collections.deque[tuple[float, float]] = collections.deque()
        self._hovering = False
        self.cmd_vel_pub = node.create_publisher(Twist, cmd_vel_topic, 10)

    def reset(self) -> None:
        self._history.clear()
        self._hovering = False

    @property
    def hovering(self) -> bool:
        return self._hovering

    def check(self, coverage: float) -> bool:
        """Return True если stall detected. Caller сам решает hover_and_wait."""
        now = time.monotonic()
        self._history.append((now, coverage))
        while self._history and (now - self._history[0][0] > self.stall_window_s):
            self._history.popleft()
        if len(self._history) < 2 or (now - self._history[0][0]) < self.stall_window_s:
            return False
        delta = coverage - self._history[0][1]
        if abs(delta) < self.stall_epsilon:
            self._hovering = True
            self.node.get_logger().warn(
                f"coverage stall: Δ={delta:.4f} < ε={self.stall_epsilon} "
                f"за {self.stall_window_s}s → hover_and_wait"
            )
            return True
        return False

    def hover_step(self) -> None:
        """Publish zero velocity (hover_and_wait one tick)."""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def trigger_invalid_action(self, action: int) -> None:
        self._hovering = True
        self.node.get_logger().error(
            f"invalid action {action} from policy → hover_and_wait"
        )

    def trigger_bridge_crash(self, exc: BaseException) -> None:
        self._hovering = True
        self.node.get_logger().error(
            f"bridge_crash: {type(exc).__name__}: {exc} → hover_and_wait"
        )

    def trigger_odom_stale(self, age_s: float) -> None:
        """
        Odom не приходит >threshold s (TASK-059 attempt #1 RCA): VisitedGridBuilder
        зависает в (0,0), policy получает stale obs → flyaway. Hover + skip predict.
        Bridge продолжит после первого свежего odom callback.
        """
        if not self._hovering:
            self.node.get_logger().warn(
                f"odom stale: age={age_s:.2f}s > threshold → hover_and_wait (skip predict)"
            )
        self._hovering = True

    def clear_hover_if_recovered(self) -> None:
        """Bridge вызывает после получения свежего odom — выходим из hover."""
        if self._hovering:
            self.node.get_logger().info("odom recovered → exiting hover")
        self._hovering = False

    def trigger_pose_out_of_box(self, x: float, y: float, half_extent: float) -> None:
        """Drone vышел за room safe box → hover_and_wait (не autoland, чтобы не упасть
        на препятствие за стеной; orch решит recovery decisionом)."""
        self._hovering = True
        self.node.get_logger().error(
            f"pose out of safe box: ({x:.2f}, {y:.2f}) outside ±{half_extent:.2f} → hover_and_wait"
        )
