"""ActionExecutor (D-refactor 2026-05-20) — POSITION setpoint control.

Old approach: velocity Twist publish с cell_size/linear_speed timing → PID overshoot,
yaw control lost (cmd_vel_unstamped angular.z ignored by MAVROS plugin), body→world
transform complexity. Attempts #1-11 failed на этом подходе.

New approach (Aleks @09:58): ArduPilot accepts target pose via `/mavros/setpoint_position/local`
(PoseStamped). ArduPilot internally handles velocity/accel ramp/PID к target.
We just set target pose and wait for arrival.

ActionExecutor maintains current target pose + publisher 10Hz (mandatory для ArduPilot
GUIDED mode). Each action updates target_pose. wait_for_arrival polls real pose
(from obs_builder via callback) с per-action-type tolerance:
- Translations (actions 0-3): distance_to_target < 0.08m
- Rotations (actions 4-5): |yaw diff| < 2°
- Action 7 (forward to wall): distance < 0.1m

Action semantics:
    0 forward       : target = current + (cell_size, 0)_body_frame
    1 backward      : target = current - (cell_size, 0)_body_frame
    2 strafe_left   : target = current + (0, cell_size)_body_frame
    3 strafe_right  : target = current - (0, cell_size)_body_frame
    4 rotate_plus   : target_yaw += 15° (position unchanged)
    5 rotate_minus  : target_yaw -= 15° (position unchanged)
    6 scan          : servo command (no pose change)
    7 forward_until_collision: target = current + (front_dist - 0.4m)_body_forward
"""
from __future__ import annotations

import math
import time
from collections.abc import Callable

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


CMD_POSE_TOPIC = "/mavros/setpoint_position/local"
SG90_CMD_TOPIC = "/drone/sg90/cmd"
MAINTAIN_RATE_HZ = 10.0
DEFAULT_SCAN_HOVER_S = 0.5
SERVO_STEP_DEG = 30.0
SERVO_MAX_DEG = 180.0
GRID_SIZE_DEFAULT = 64
TARGET_ALTITUDE_M = 3.0  # was 2.0 — 2026-05-20 attempt #19: buffer для z drift во время XY movement (-0.4m peak observed)

# Arrival tolerances (Aleks @09:58 spec + smoke test @10:18 tuning)
ARRIVAL_TOL_TRANSLATION_M = 0.08
ARRIVAL_TOL_ACTION7_M = 0.10
ARRIVAL_TOL_YAW_DEG = 3.0     # was 2.0 — smoke showed drone достигает 2.9° accuracy
ARRIVAL_TIMEOUT_S = 8.0       # max wait per action (safety cap)
ARRIVAL_TIMEOUT_ACTION7_S = 15.0  # action 7 может далеко лететь
ARRIVAL_POLL_S = 0.05

# Action 7 stop margin
ACTION7_WALL_MARGIN_M = 0.4


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angle (a - b) wrapped to [-π, π]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _make_pose(x: float, y: float, z: float, yaw: float, frame: str = "map") -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    # Quaternion from yaw (z-axis rotation)
    half = yaw / 2.0
    ps.pose.orientation.x = 0.0
    ps.pose.orientation.y = 0.0
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


class InvalidActionError(ValueError):
    pass


class ActionExecutor:
    """Position-setpoint action executor (D-refactor).

    Maintains target_pose + publishes к /mavros/setpoint_position/local @10Hz
    (mandatory непрерывный stream для ArduPilot GUIDED). Each action updates
    target_pose; execute() blocks until arrival или timeout.
    """

    def __init__(
        self,
        node: Node,
        *,
        cell_size_m: float,
        wall_threshold: float,  # legacy compat — теперь действует как min approach к стене
        get_front_distance_m: Callable[[], float],
        get_pose: Callable[[], "Pose2D"],  # obs_builder.pose accessor
        target_altitude_m: float = TARGET_ALTITUDE_M,
        grid_size: int = GRID_SIZE_DEFAULT,
        cmd_pose_topic: str = CMD_POSE_TOPIC,
        sg90_cmd_topic: str = SG90_CMD_TOPIC,
        # Legacy params (kept для backwards compat в bridge invocation, не использу-
        # ются в position control):
        linear_speed: float = 0.3,
        angular_speed: float = 0.26,
        get_yaw_rad: Callable[[], float] | None = None,
        scan_hover_s: float = DEFAULT_SCAN_HOVER_S,
        settle_hover_s: float = 0.1,
    ) -> None:
        self.node = node
        self.cell_size_m = cell_size_m
        self.wall_threshold = wall_threshold
        self.target_altitude = target_altitude_m
        self.grid_size = grid_size
        self.scan_hover_s = scan_hover_s
        self._get_front_m = get_front_distance_m
        self._get_pose = get_pose

        self.pose_pub = node.create_publisher(PoseStamped, cmd_pose_topic, 10)
        self.sg90_cmd_pub = node.create_publisher(Float64, sg90_cmd_topic, 10)
        self._servo_deg = 0.0

        # Target pose — initialized после takeoff release (см. initialize_target())
        self._target_pose: PoseStamped | None = None
        # 10 Hz maintenance timer (необходим для ArduPilot GUIDED setpoint stream)
        self._maint_timer = node.create_timer(
            1.0 / MAINTAIN_RATE_HZ, self._publish_maintenance
        )

    # ---- target management ----

    def initialize_target(
        self, x: float, y: float, z: float | None = None, yaw: float = 0.0
    ) -> None:
        """Set initial target pose. Called bridge'ом после /takeoff/ready signal."""
        if z is None:
            z = self.target_altitude
        self._target_pose = _make_pose(x, y, z, yaw)
        self.node.get_logger().info(
            f"action_executor target initialized: ({x:.2f}, {y:.2f}, {z:.2f}, "
            f"yaw={math.degrees(yaw):.1f}°)"
        )

    def _publish_maintenance(self) -> None:
        """10 Hz publish current target pose (mandatory ArduPilot GUIDED)."""
        if self._target_pose is None:
            return
        self._target_pose.header.stamp = self.node.get_clock().now().to_msg()
        self.pose_pub.publish(self._target_pose)

    def _set_target(self, x: float, y: float, yaw: float) -> None:
        """Update target pose (altitude held constant)."""
        z = self.target_altitude if self._target_pose is None else self._target_pose.pose.position.z
        self._target_pose = _make_pose(x, y, z, yaw)

    @property
    def target_pose(self) -> PoseStamped | None:
        return self._target_pose

    @property
    def servo_deg(self) -> float:
        return self._servo_deg

    # ---- public API ----

    def execute(
        self,
        action: int,
        override_speed: float | None = None,         # legacy compat, ignored
        override_wall_threshold: float | None = None, # legacy compat
    ) -> dict[str, float]:
        wt = override_wall_threshold if override_wall_threshold is not None else self.wall_threshold

        pose = self._get_pose()
        cur_x = pose.x_m
        cur_y = pose.y_m
        cur_yaw = pose.heading_rad

        if action == 0:
            return self._translation(cur_x, cur_y, cur_yaw, 1.0, 0.0)
        if action == 1:
            return self._translation(cur_x, cur_y, cur_yaw, -1.0, 0.0)
        if action == 2:
            return self._translation(cur_x, cur_y, cur_yaw, 0.0, 1.0)
        if action == 3:
            return self._translation(cur_x, cur_y, cur_yaw, 0.0, -1.0)
        if action == 4:
            return self._rotation(cur_x, cur_y, cur_yaw, +math.radians(15.0))
        if action == 5:
            return self._rotation(cur_x, cur_y, cur_yaw, -math.radians(15.0))
        if action == 6:
            return self._scan()
        if action == 7:
            return self._forward_until_collision(cur_x, cur_y, cur_yaw, wt)
        raise InvalidActionError(f"invalid action {action} (Discrete(8): 0..7)")

    def stop(self) -> None:
        """Freeze drone at current target (no update). Used в failure modes."""
        # Target unchanged → drone holds. Nothing к do.
        pass

    # ---- internals ----

    def _translation(
        self,
        cur_x: float,
        cur_y: float,
        cur_yaw: float,
        body_dx_cells: float,
        body_dy_cells: float,
    ) -> dict[str, float]:
        """Move 1 cell в body-frame direction (translate, keep yaw)."""
        # Body→world transform via current yaw
        c = math.cos(cur_yaw)
        s = math.sin(cur_yaw)
        dx_world = (body_dx_cells * c - body_dy_cells * s) * self.cell_size_m
        dy_world = (body_dx_cells * s + body_dy_cells * c) * self.cell_size_m

        target_x = cur_x + dx_world
        target_y = cur_y + dy_world
        self._set_target(target_x, target_y, cur_yaw)
        arrived = self._wait_arrival_position(target_x, target_y, ARRIVAL_TOL_TRANSLATION_M,
                                              ARRIVAL_TIMEOUT_S)
        return {"kind": 0.0, "arrived": float(arrived)}

    def _rotation(
        self, cur_x: float, cur_y: float, cur_yaw: float, dyaw_rad: float
    ) -> dict[str, float]:
        target_yaw = cur_yaw + dyaw_rad
        # Position unchanged, only yaw target updated
        self._set_target(cur_x, cur_y, target_yaw)
        arrived = self._wait_arrival_yaw(target_yaw, math.radians(ARRIVAL_TOL_YAW_DEG),
                                         ARRIVAL_TIMEOUT_S)
        return {"kind": 1.0, "arrived": float(arrived)}

    def _scan(self) -> dict[str, float]:
        self._servo_deg = (self._servo_deg + SERVO_STEP_DEG) % SERVO_MAX_DEG
        cmd = Float64()
        cmd.data = math.radians(self._servo_deg)
        self.sg90_cmd_pub.publish(cmd)
        # Target pose unchanged — drone holds
        time.sleep(self.scan_hover_s)
        return {"kind": 2.0, "duration_s": self.scan_hover_s, "servo_deg": self._servo_deg}

    def _forward_until_collision(
        self, cur_x: float, cur_y: float, cur_yaw: float, wall_margin: float
    ) -> dict[str, float]:
        """Action 7: target = current + (front_dist - margin) forward.

        ArduPilot navigates autonomously. We poll arrival.
        """
        front = max(0.0, self._get_front_m())
        margin = max(ACTION7_WALL_MARGIN_M, wall_margin)
        travel = max(0.0, front - margin)
        c = math.cos(cur_yaw)
        s = math.sin(cur_yaw)
        target_x = cur_x + c * travel
        target_y = cur_y + s * travel
        self._set_target(target_x, target_y, cur_yaw)

        if travel <= 0.01:
            self.node.get_logger().warn(
                f"action 7: front={front:.2f}m ≤ margin={margin:.2f}m, no travel"
            )
            return {"kind": 3.0, "travel": 0.0, "arrived": 1.0}

        arrived = self._wait_arrival_position(
            target_x, target_y, ARRIVAL_TOL_ACTION7_M, ARRIVAL_TIMEOUT_ACTION7_S
        )
        return {"kind": 3.0, "travel": travel, "arrived": float(arrived)}

    def _wait_arrival_position(
        self, target_x: float, target_y: float, tolerance: float, timeout_s: float
    ) -> bool:
        """Poll pose until distance < tolerance OR timeout."""
        t_start = time.monotonic()
        while time.monotonic() - t_start < timeout_s:
            pose = self._get_pose()
            dx = pose.x_m - target_x
            dy = pose.y_m - target_y
            dist = math.hypot(dx, dy)
            if dist < tolerance:
                return True
            time.sleep(ARRIVAL_POLL_S)
        self.node.get_logger().warn(
            f"arrival timeout {timeout_s:.1f}s: target=({target_x:.2f}, {target_y:.2f}) "
            f"final_pose=({pose.x_m:.2f}, {pose.y_m:.2f}) dist={dist:.2f} > tol={tolerance:.2f}"
        )
        return False

    def _wait_arrival_yaw(
        self, target_yaw: float, tolerance_rad: float, timeout_s: float
    ) -> bool:
        """Poll pose until |yaw_diff| < tolerance_rad OR timeout."""
        t_start = time.monotonic()
        while time.monotonic() - t_start < timeout_s:
            pose = self._get_pose()
            diff = abs(_angle_diff(pose.heading_rad, target_yaw))
            if diff < tolerance_rad:
                return True
            time.sleep(ARRIVAL_POLL_S)
        self.node.get_logger().warn(
            f"yaw arrival timeout {timeout_s:.1f}s: target={math.degrees(target_yaw):.1f}° "
            f"final={math.degrees(pose.heading_rad):.1f}° diff={math.degrees(diff):.1f}°"
        )
        return False
