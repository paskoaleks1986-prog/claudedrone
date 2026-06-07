#!/usr/bin/env python3
"""policy_bridge_node — SWEEP-02 RL bridge между Gazebo сенсорами и MAVROS velocity.

Architecture:
    [ROS2 topics] ──► ObsBuilder ──┐
                                    ├──► PPO.predict() ──► ActionExecutor ──► /mavros/cmd_vel
              VisitedGridBuilder ──┘                              │
                                                                  └─► /rl_policy/{action,visited_grid,coverage}

Rate: 10 Hz (calibration в TASK-061 / fallback 3.6 Hz если mavros odom slow).
Action 7 — continuous loop inside ActionExecutor (single predict call → multi-step Gazebo).

Failure modes:
    bridge_crash      → hover_and_wait (default — НЕ land, чтобы не упасть на препятствие)
    invalid_action    → hover_and_wait
    mavros_disconnect → land_immediately (only this one lands)  TODO Phase 2
    coverage_stall    → hover_and_wait

Sprint: model-to-sim-bridge Phase 1 (TASK-059).
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from policy_bridge.action_gate import gate_blocks, movement_clearance_m
from policy_bridge.obs_builder import ObsBuilder
from policy_bridge.visited_grid import VisitedGridBuilder
from policy_bridge.action_executor import ActionExecutor, InvalidActionError
from policy_bridge.coverage import Coverage
from policy_bridge.failure_modes import FailureHandler
from policy_bridge.adaptive_speed import AdaptiveSpeedController
from policy_bridge.stuck_detector import StuckDetector
from policy_bridge.wall_follower import WallFollower
from policy_bridge.wall_map_builder import WallMapBuilder
from policy_bridge.phase_controller import PhaseController, Phase


PARAM_DEFAULTS: dict[str, object] = {
    "model_path": "",
    "room_size": 6.4,
    "cell_size": 0.1,
    # TASK-059 attempt #1 RCA (2026-05-19): 0.15 m оказался слишком тесный
    # для real Gazebo (drone 0.3 m/s, VL53L0X max 2 m → no warning до впритык).
    # 0.50 m = ~1.7 cell stop distance, безопаснее.
    "wall_threshold": 0.50,
    "linear_speed": 0.3,
    "angular_speed": 0.26,
    "rate_hz": 10.0,
    "free_mask_path": "auto",
    "rosbag_dir": "",
    "max_steps": 3000,
    "perimeter_topic": "/drone/perimeter",
    "sweep_topic": "/scan/sweep",
    "joint_state_topic": "/joint_state",
    "odom_topic": "/mavros/local_position/odom",
    # TASK-059 attempt #1 RCA: bridge получал stale obs (QoS odom mismatch),
    # publish'ил cmd_vel на based-on-stale pose, drone flew away.
    # Если odom callback не приходит дольше этого порога — hover+skip predict.
    "odom_stale_threshold_s": 1.0,
    # Safe box: drone должен оставаться в ±room_size/2+margin. Outside → hover.
    "safe_box_margin_m": 0.5,
    # TASK-062 Path B HYBRID (2026-05-20)
    "mode": "hybrid",          # hybrid | wall_follow_only | rl_only
    "wall_distance": 0.6,
    "perimeter_laps": 1,
    # v2 Block 3 (2026-06-06): ActionGate — шаг 0-3, который закончится ближе
    # этого к препятствию, отклоняется до исполнения (training parity: env не
    # двигает дрона в стену). Правило: gate_margin = safety floor + cell.
    # v2 run F (Aleks 08:26): 0.5 (= floor 0.4 + cell). SIM-ONLY клиренс.
    # run F checklist #1 (2026-06-07): floor 0.4 → 0.45 ⇒ gate 0.55.
    "gate_margin_m": 0.55,
}


class PolicyBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("policy_bridge_node")
        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.model_path = str(self.get_parameter("model_path").value)
        self.room_size = float(self.get_parameter("room_size").value)
        self.cell_size = float(self.get_parameter("cell_size").value)
        self.wall_threshold = float(self.get_parameter("wall_threshold").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.free_mask_path = str(self.get_parameter("free_mask_path").value)
        self.max_steps = int(self.get_parameter("max_steps").value)
        self.odom_stale_threshold_s = float(
            self.get_parameter("odom_stale_threshold_s").value
        )
        self.safe_box_margin_m = float(self.get_parameter("safe_box_margin_m").value)
        # TASK-062
        self.mode = str(self.get_parameter("mode").value)
        self.wall_distance = float(self.get_parameter("wall_distance").value)
        self.perimeter_laps = int(self.get_parameter("perimeter_laps").value)
        # v2 Block 3
        self.gate_margin_m = float(self.get_parameter("gate_margin_m").value)
        self._gate_block_count = 0

        self.grid_size = int(round(self.room_size / self.cell_size))
        if self.grid_size != 64:
            self.get_logger().warn(
                f"grid_size={self.grid_size} ≠ 64 — SWEEP-02 was trained на MAP_SIZE=64. "
                "Bridge будет работать но policy expects 64×64 visited grid."
            )

        # ----- step counter / timer (declare cb groups FIRST so ObsBuilder gets sub group) -----
        self.step_count = 0
        # TASK-059 attempt #2 RCA (2026-05-20): timer ДОЛЖЕН быть MutuallyExclusive
        # чтобы _tick не reenter'ил во время action 7's 21s blocking loop.
        # Subscriptions — Reentrant (separate group), чтобы odom/perimeter обновлялись
        # параллельно с long-running action execute (иначе VGB и safety net застывают).
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()

        # ----- core components -----
        self.obs_builder = ObsBuilder(
            self,
            perimeter_topic=str(self.get_parameter("perimeter_topic").value),
            sweep_topic=str(self.get_parameter("sweep_topic").value),
            joint_state_topic=str(self.get_parameter("joint_state_topic").value),
            odom_topic=str(self.get_parameter("odom_topic").value),
            callback_group=self.sub_cb_group,
        )
        self.visited = VisitedGridBuilder(
            room_size_m=self.room_size,
            cell_size_m=self.cell_size,
            grid_size=self.grid_size,
        )
        self.coverage = self._build_coverage()
        self.executor_act = ActionExecutor(
            self,
            cell_size_m=self.cell_size,
            wall_threshold=self.wall_threshold,
            get_front_distance_m=lambda: self.obs_builder.front_distance_m,
            # D-refactor (2026-05-20): position setpoint control. Pass pose accessor
            # для arrival check + initial target.
            get_pose=lambda: self.obs_builder.pose,
            grid_size=self.grid_size,
            # Legacy (ignored в position control but kept в signature)
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
            get_yaw_rad=lambda: self.obs_builder.pose.heading_rad,
            # v2 Block 2: training parity — env помечает visited все клетки
            # пройденные за action (включая промежуточные у action 7).
            visited_update_fn=self.visited.update,
            # v2 run F: velocity-gated arrival (Aleks 08:26)
            get_speed_m_s=lambda: self.obs_builder.speed_m_s,
        )
        # v2 Block 2: servo_angle obs = commanded angle executor'а (training
        # parity: в env servo-динамики нет). Убирает 1 kHz JointState churn.
        self.obs_builder.set_servo_angle_source(lambda: self.executor_act.servo_deg)

        # v2 night watch: координация с safety_guard (latched Bool).
        # True → executor молчит (maintenance пауза), predict пропускается;
        # False → target переинициализируется на текущую позу.
        safety_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Bool, "/safety/active",
            lambda m: self.executor_act.set_safety_hold(bool(m.data)),
            safety_qos, callback_group=self.sub_cb_group,
        )
        self.failure = FailureHandler(self)

        # ----- adaptive speed controller (TASK-059 attempt #5, Aleks/Web 4-mode design) -----
        self.adaptive_speed = AdaptiveSpeedController()
        self._adaptive_log_counter = 0
        # v2 Block 2: freshness gate counter (см. _predict_and_execute_one_step)
        self._obs_fresh_skip_count = 0

        # ----- stuck detector (TASK-059 attempt #7, Aleks/Web R2 escape pattern) -----
        # Detects coverage-stuck loops (policy/obs feedback cycle) и injects escape:
        # 8x rotate (≈120°) + 4x backward → breaks observation determinism.
        self.stuck_detector = StuckDetector()

        # ----- Phase Controller (TASK-062 Path B HYBRID 2026-05-20) -----
        # mode=hybrid: WallFollower phase → RL phase по perimeter_complete.
        # mode=wall_follow_only: только wall_follow, никогда не switches к RL.
        # mode=rl_only: пропускает wall phase, сразу RL (legacy behavior).
        self.wall_follower = WallFollower(
            wall_distance=self.wall_distance,
        )
        self.wall_map_builder = WallMapBuilder(
            room_size_m=self.room_size,
            grid_size=self.grid_size,
        )
        self.phase_controller = PhaseController(
            wall_follower=self.wall_follower,
            wall_map_builder=self.wall_map_builder,
            visited_update_fn=self.visited.update,
            policy_predict_fn=None,  # bridge handles RL inline в _tick
            node_logger=self.get_logger(),
        )
        # Force initial phase per mode
        if self.mode == "rl_only":
            from policy_bridge.phase_controller import Phase as _Phase
            self.phase_controller.phase = _Phase.RL_EXPLORE
            self.get_logger().info(
                "mode=rl_only — пропускаем wall phase, начинаем сразу RL"
            )

        # Wall-follow arrival gating (Aleks @11:10 Risk 2 fix).
        # _wall_follow_tick шлёт новый target только когда drone достиг
        # предыдущего (pos tol 0.15m + yaw tol 5°). Maintenance timer 10Hz
        # action_executor продолжает publish существующий target пока ждём.
        # Timeout 3s страхует от deadlock (если drone застрял — force next step).
        self._wf_last_target_x: float | None = None
        self._wf_last_target_y: float | None = None
        self._wf_last_target_yaw: float | None = None
        self._wf_target_set_monotonic: float | None = None
        self._wf_arrival_skip_count = 0
        self._wf_last_was_rotation: bool = False  # attempt #12 RCA Cause #2:
        # rotation commands waitгают yaw_arrival с longer timeout

        # ----- takeoff ready signal (D-refactor 2026-05-20) -----
        # Bridge waits for /takeoff/ready True перед началом predict loop. takeoff_node
        # publishes ready после стабилизации hover, потом releases setpoint control.
        # Bridge takes over publishing к /mavros/setpoint_position/local через action_executor.
        self._takeoff_ready = False
        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Bool, "/takeoff/ready", self._takeoff_ready_cb, ready_qos,
            callback_group=self.sub_cb_group,
        )

        # ----- model -----
        self.model = self._load_model()

        # ----- publishers -----
        self.action_pub = self.create_publisher(Int32, "/rl_policy/action", 10)
        self.coverage_pub = self.create_publisher(Float32, "/rl_policy/coverage", 10)
        self.visited_grid_pub = self.create_publisher(
            OccupancyGrid, "/rl_policy/visited_grid", 10
        )

        # ----- timer (cb groups уже declared выше до ObsBuilder construction) -----
        period_s = 1.0 / self.rate_hz
        self.timer = self.create_timer(period_s, self._tick, callback_group=self.timer_cb_group)

        self.get_logger().info(
            f"policy_bridge_node ready · rate={self.rate_hz} Hz · room {self.room_size}×{self.room_size} m · "
            f"defaults linear={self.linear_speed} m/s angular={self.angular_speed} rad/s "
            f"wall_threshold={self.wall_threshold} m"
        )
        if self.coverage.free_mask is None:
            self.get_logger().warn(
                "free_mask not loaded → coverage = visited.sum()/grid_total (не Option A!). "
                "Передай free_mask_path для accurate coverage."
            )
        else:
            self.get_logger().info(f"coverage Option A · free_count={self.coverage.free_count}")

    # ---- model loading -----------------------------------------------------

    def _load_model(self):
        if not self.model_path:
            raise RuntimeError("model_path param empty — set it via launch arg.")
        # Late import чтобы node мог быть импортирован без SB3 (для unit тестов).
        from stable_baselines3 import PPO
        path = Path(self.model_path)
        if not path.exists():
            raise FileNotFoundError(f"model.zip not found: {path}")
        self.get_logger().info(f"loading PPO from {path} (device=cpu)")
        return PPO.load(str(path), device="cpu")

    def _build_coverage(self) -> Coverage:
        path = self.free_mask_path
        if path in ("", "none"):
            return Coverage(free_mask_path=None, grid_size=self.grid_size)
        if path == "auto":
            # v2 Block 2 (2026-06-06): auto-resolve реализован. Ищем
            # free_mask.png рядом с миром: DEFAULT_WORLD (env, тот же механизм,
            # что drone.launch.py) → src/drone_sim/worlds/rl_rooms/<world>/.
            # Без маски coverage = visited/4096 занижает прогресс ~втрое
            # (free cells ≈ 1/3 грида) — и порог COVERAGE_TARGET недостижим.
            world = os.environ.get("DEFAULT_WORLD", "")
            sim_root = os.environ.get(
                "AEROSEARCH_ROOT", "/data/git/aerosearch"
            ) + "/claudedrone-git/simulation"
            candidate = (
                Path(sim_root)
                / "src/drone_sim/worlds/rl_rooms" / world / "free_mask.png"
            )
            if world and candidate.exists():
                self.get_logger().info(f"free_mask_path=auto → {candidate}")
                return Coverage(
                    free_mask_path=str(candidate), grid_size=self.grid_size
                )
            self.get_logger().warn(
                f"free_mask_path=auto: не нашёл {candidate} "
                f"(DEFAULT_WORLD={world!r}) — coverage без маски"
            )
            return Coverage(free_mask_path=None, grid_size=self.grid_size)
        return Coverage(free_mask_path=path, grid_size=self.grid_size)

    # ---- main tick ---------------------------------------------------------

    def _takeoff_ready_cb(self, msg: Bool) -> None:
        """Signal от takeoff_node: hover stable, релиз setpoint control bridge'у."""
        if msg.data and not self._takeoff_ready:
            self._takeoff_ready = True
            pose = self.obs_builder.pose
            # run F fix (в) 2026-06-07: z-capture at release. Константа 3.0
            # давала хронический Z-лаг (~0.18м), Position Controller делил
            # authority между Z и XY → median XY 0.063 м/с при WPNAV cap 0.2
            # → action7 arrival timeouts (RCA step 100, вердикт @338 cov ✗).
            # Берём ФАКТИЧЕСКИЙ hover z; sanity < 0.5м (odom ещё пуст) →
            # fallback на константу с warn.
            if pose.z_m > 0.5:
                z_capture = pose.z_m
                self.get_logger().info(
                    f"z-capture at release: {z_capture:.2f}m (фактический hover; "
                    f"константа TARGET_ALTITUDE_M=3.0 не используется)"
                )
            else:
                z_capture = None
                self.get_logger().warn(
                    f"z-capture failed (odom z={pose.z_m:.2f} < 0.5m sanity) — "
                    f"fallback на TARGET_ALTITUDE_M"
                )
            # Initial target = current pose (drone holds in place)
            self.executor_act.initialize_target(
                pose.x_m, pose.y_m, z=z_capture, yaw=pose.heading_rad
            )
            self.get_logger().info(
                f"/takeoff/ready received — bridge taking over setpoint control "
                f"at ({pose.x_m:.2f}, {pose.y_m:.2f}, yaw={pose.heading_rad:.2f}rad)"
            )

    def _tick(self) -> None:
        if self.step_count >= self.max_steps:
            return

        # D-refactor: wait для takeoff_node release control signal
        if not self._takeoff_ready:
            return

        # v2 night watch: safety_guard владеет дроном — не predict'им и не
        # двигаем (он отведёт от препятствия и отпустит /safety/active=False).
        if self.executor_act.safety_hold:
            return

        # Pre-flight checks (TASK-059 attempt #1 RCA — safety net):
        # 1. odom staleness — без свежей позы VGB зависает, policy получает stale
        #    obs → flyaway. Если odom > threshold s → hover + skip predict.
        # 2. pose out of safe-box — даже если odom свежая, поза за room bounds
        #    (drone vышел через стену/потолок) → permanent hover (recovery —
        #    orch decision'ом, не auto).
        now_s = self.get_clock().now().nanoseconds * 1e-9
        staleness = self.obs_builder.staleness_seconds(now_s)
        pose = self.obs_builder.pose
        half_extent = self.room_size / 2.0 + self.safe_box_margin_m
        out_of_box = (
            self.obs_builder.has_received_odom
            and (abs(pose.x_m) > half_extent or abs(pose.y_m) > half_extent)
        )
        odom_stale = (
            self.obs_builder.has_received_odom
            and staleness["odom"] > self.odom_stale_threshold_s
        )

        # Recovery: если hovering исключительно из-за odom_stale и odom вернулся —
        # выходим из hover. Out-of-box hover — persistent (требует ручного restart).
        if self.failure.hovering and not odom_stale and not out_of_box:
            self.failure.clear_hover_if_recovered()

        if out_of_box:
            self.failure.trigger_pose_out_of_box(pose.x_m, pose.y_m, half_extent)
        elif odom_stale:
            self.failure.trigger_odom_stale(staleness["odom"])

        if self.failure.hovering:
            self.failure.hover_step()
            return  # НЕ counting toward max_steps

        try:
            self._predict_and_execute_one_step()
        except InvalidActionError as e:
            self.failure.trigger_invalid_action(getattr(e, "args", [-1])[0] if e.args else -1)
            self.executor_act.stop()
        except Exception as e:  # noqa: BLE001 — bridge_crash blanket
            self.failure.trigger_bridge_crash(e)
            self.executor_act.stop()

    def _wall_follow_tick(self, pose) -> None:
        """One tick для wall_follow phase (TASK-062 Path B).

        attempt #12 RCA fixes (Aleks @12:00 path В):
          - Cause #1: min_emit_interval 1.0s gate перед PhaseController.step()
            (раньше 10Hz spamил setpoints, drone не успевал → AngErr crash)
          - Cause #2: rotation cmd (turn_left/turn_right) issued как yaw-only
            setpoint и ждёт yaw_arrival 5s timeout (раньше pos+yaw together →
            attitude tilt → AP crash detector AngErr 70°)
          - Cause #3: handled в wall_follower.py (start_pos defer + min_flight_s 60)
        """
        WF_MIN_EMIT_INTERVAL_S = 1.0       # Cause #1: throttle
        ARRIVAL_TOL_POS_M = 0.25
        ARRIVAL_TOL_YAW_RAD = math.radians(5.0)
        ARRIVAL_TIMEOUT_TRANS_S = 8.0      # 2026-05-20 attempt #18: bumped from 3.0 — drone не успевал доходить за 3s при 0.30 m/s + физ. дампирование
        ARRIVAL_TIMEOUT_YAW_S = 12.0       # 2026-05-20 attempt #18: bumped from 5.0 — 90° at 15°/s = 6s + margin
        now_mono = time.monotonic()

        # ---- Cause #1: min interval throttle ----
        if self._wf_target_set_monotonic is not None:
            since_last_emit = now_mono - self._wf_target_set_monotonic
            if since_last_emit < WF_MIN_EMIT_INTERVAL_S:
                # Maintenance timer (action_executor) продолжает publishing старый target
                return

        # ---- arrival check (previous target) ----
        if self._wf_last_target_x is not None:
            dx = pose.x_m - self._wf_last_target_x
            dy = pose.y_m - self._wf_last_target_y
            pos_err = math.hypot(dx, dy)
            yaw_err = abs(self._angle_diff(pose.heading_rad, self._wf_last_target_yaw))
            arrived = pos_err < ARRIVAL_TOL_POS_M and yaw_err < ARRIVAL_TOL_YAW_RAD
            timeout_s = (
                ARRIVAL_TIMEOUT_YAW_S if self._wf_last_was_rotation
                else ARRIVAL_TIMEOUT_TRANS_S
            )
            elapsed = now_mono - (self._wf_target_set_monotonic or now_mono)
            if not arrived and elapsed < timeout_s:
                self._wf_arrival_skip_count += 1
                if self._wf_arrival_skip_count % 20 == 0:
                    self.get_logger().info(
                        f"WF arrival wait ({('YAW' if self._wf_last_was_rotation else 'TRANS')}): "
                        f"pos_err={pos_err:.2f}m yaw_err={math.degrees(yaw_err):.1f}° "
                        f"elapsed={elapsed:.1f}s skips={self._wf_arrival_skip_count}"
                    )
                return
            if not arrived:
                self.get_logger().warn(
                    f"WF arrival TIMEOUT ({elapsed:.1f}s, {('YAW' if self._wf_last_was_rotation else 'TRANS')}) "
                    f"pos_err={pos_err:.2f}m yaw_err={math.degrees(yaw_err):.1f}° — forcing next step"
                )

        distances_m = self.obs_builder.perimeter_distances_m
        cmd = self.phase_controller.step(
            distances=distances_m,
            pose_x=pose.x_m,
            pose_y=pose.y_m,
            pose_yaw=pose.heading_rad,
            step_count=self.step_count,
        )

        # ---- Cause #2: decouple yaw turn from translation ----
        # Rotation cmd (turn_left/turn_right) → yaw-only setpoint:
        # keep position = current pose, change только yaw.
        # Other commands (forward/adjust_*/follow_wall/corner_resume/find_wall_forward)
        # → normal pose+yaw setpoint.
        is_rotation = cmd.type in ("turn_left", "turn_right")
        if cmd.target_x is not None and cmd.target_y is not None:
            if is_rotation:
                # Hold position at current; only yaw changes
                target_x = pose.x_m
                target_y = pose.y_m
            else:
                target_x = cmd.target_x
                target_y = cmd.target_y
            self.executor_act.initialize_target(
                target_x, target_y, z=None, yaw=cmd.target_yaw
            )
            self._wf_last_target_x = target_x
            self._wf_last_target_y = target_y
            self._wf_last_target_yaw = cmd.target_yaw
            self._wf_last_was_rotation = is_rotation
            self._wf_target_set_monotonic = now_mono
            self._wf_arrival_skip_count = 0  # reset для нового target

        # Coverage update — visited_grid updates handled inside phase_controller
        cov = self.coverage.compute(self.visited.grid)
        self.coverage_pub.publish(Float32(data=float(cov)))
        self._publish_visited_grid(self.visited.grid)

        self.step_count += 1
        if self.step_count % 50 == 0:
            self.get_logger().info(
                f"step {self.step_count} · WF {cmd.debug_tag} · "
                f"target=({cmd.target_x:.2f}, {cmd.target_y:.2f}, yaw={math.degrees(cmd.target_yaw):.1f}°) · "
                f"wall_cells={self.wall_map_builder.wall_cells_total} · coverage {cov:.3f}"
            )

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def _predict_and_execute_one_step(self) -> None:
        pose = self.obs_builder.pose

        # TASK-062 Path B HYBRID: PhaseController decides phase
        if self.mode != "rl_only" and self.phase_controller.current_phase == Phase.WALL_FOLLOW:
            self._wall_follow_tick(pose)
            return
        # RL phase below (legacy path)

        # v2 Block 2: freshness gate. Politika тренирована turn-based — obs
        # обязан описывать состояние ПОСЛЕ предыдущего действия. Если сенсорные
        # кэши старше порога (odom ~8.8Hz → 3 цикла, perimeter 10Hz), snapshot
        # отражает позу/дистанции ДО остановки → off-distribution. Пропускаем
        # тик (без счёта шага), следующий через 0.1с перепроверит.
        now_s = self.get_clock().now().nanoseconds * 1e-9
        stale = self.obs_builder.staleness_seconds(now_s)
        if stale["odom"] > 0.35 or stale["perimeter"] > 0.35 or stale["sweep"] > 1.0:
            self._obs_fresh_skip_count += 1
            if self._obs_fresh_skip_count % 50 == 1:
                self.get_logger().warn(
                    f"obs not fresh, skip predict: odom={stale['odom']:.2f}s "
                    f"perimeter={stale['perimeter']:.2f}s sweep={stale['sweep']:.2f}s "
                    f"(skips={self._obs_fresh_skip_count})"
                )
            return
        self._obs_fresh_skip_count = 0

        self.visited.update(pose.x_m, pose.y_m)

        obs = self.obs_builder.build_obs(self.visited.grid)
        action_arr, _ = self.model.predict(obs, deterministic=False)
        raw_action_from_policy = int(action_arr)

        # TASK-059 attempt #8 escape v2 (rl-lab @03:18): StuckDetector v2 со
        # smart escape — scoring direction via (free_dist + unvisited_in_cone),
        # rotate→sweep OR fallback backward. Возвращает (action, escape_active)
        # для B2 bypass в adaptive_speed.
        cov_for_stuck = self.coverage.compute(self.visited.grid)
        pose = self.obs_builder.pose
        # Convert pose to cell ix/iy (matches VisitedGridBuilder formula)
        pos_ix = int((pose.x_m + self.room_size / 2.0) / self.cell_size)
        pos_iy = int((pose.y_m + self.room_size / 2.0) / self.cell_size)
        # Clip к grid bounds (drone может быть outside в edge cases)
        pos_ix = max(0, min(pos_ix, self.grid_size - 1))
        pos_iy = max(0, min(pos_iy, self.grid_size - 1))

        perimeter_distances = self.obs_builder.perimeter_distances_m
        action, escape_active = self.stuck_detector.check_v2(
            coverage=cov_for_stuck,
            raw_action=raw_action_from_policy,
            distances_m=perimeter_distances,
            visited_grid=self.visited.grid,
            pos_ix=pos_ix,
            pos_iy=pos_iy,
            pose_x_m=pose.x_m,
            pose_y_m=pose.y_m,
            heading_rad=pose.heading_rad,
            node_logger=self.get_logger(),
        )

        # TASK-059 attempt #5: AdaptiveSpeedController + B2 (rl-lab @03:18):
        # escape_bypass пропускает action 7 degradation чтобы StuckDetector v2
        # forced sweep исполнялся как full multi-cell action 7 (без этого
        # coverage capped ~0.45 в mock).
        distances_m = perimeter_distances + [self.obs_builder.sweep_distance_m]
        action_mod, cfg, mode = self.adaptive_speed.evaluate(
            action, distances_m, escape_bypass=escape_active
        )

        # v2 Block 3: ActionGate — слой изоляции №1. Шаг 0-3 в сторону стены
        # отклоняем мгновенно (training parity: в env такой шаг не двигает
        # дрона). Дрон держит позицию, шаг засчитывается, модель получает
        # свежий obs и выбирает дальше — вместо 8s tug-of-war с safety_guard.
        if gate_blocks(action_mod, perimeter_distances, self.gate_margin_m):
            clearance = movement_clearance_m(action_mod, perimeter_distances)
            self._gate_block_count += 1
            self.get_logger().info(
                f"gate: action {action_mod} отклонён — clearance "
                f"{clearance:.2f}m < margin {self.gate_margin_m:.2f}m "
                f"(blocks={self._gate_block_count})"
            )
            self.action_pub.publish(Int32(data=action_mod))
            self.step_count += 1
            return

        # Publish EXECUTED action (post-stuck/adaptive) для policy logging
        self.action_pub.publish(Int32(data=action_mod))

        self.executor_act.execute(
            action_mod,
            override_speed=cfg.linear_speed,
            override_wall_threshold=cfg.wall_threshold,
        )

        cov = self.coverage.compute(self.visited.grid)
        self.coverage_pub.publish(Float32(data=float(cov)))
        self._publish_visited_grid(self.visited.grid)

        if self.failure.check(cov):
            # hovering=True; следующий tick поедет в hover branch
            return

        self.step_count += 1
        # Log every 50 steps + immediately log mode changes + escape events
        self._adaptive_log_counter += 1
        escape_tag = " [ESCAPE]" if self.stuck_detector.escape_active else ""
        action_chain = []
        if raw_action_from_policy != action:
            action_chain.append(f"{raw_action_from_policy}→{action}")
        else:
            action_chain.append(str(action))
        if action != action_mod:
            action_chain.append(f"→{action_mod}")
        action_str = "".join(action_chain)

        if self.step_count % 50 == 0 or action != action_mod or raw_action_from_policy != action:
            self.get_logger().info(
                f"step {self.step_count} · action {action_str}{escape_tag} · "
                f"mode {mode.value} (v={cfg.linear_speed:.2f}m/s wt={cfg.wall_threshold:.2f}m) · "
                f"coverage {cov:.3f} · escape_total={self.stuck_detector.escape_count_total}"
            )

    def _publish_visited_grid(self, grid: np.ndarray) -> None:
        """Отправляет grid как OccupancyGrid для Foxglove визуализации."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.cell_size
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        # Origin = SW corner мира.
        msg.info.origin.position.x = -self.room_size / 2.0
        msg.info.origin.position.y = -self.room_size / 2.0
        msg.info.origin.orientation.w = 1.0
        # OccupancyGrid expects int8 -1/0/100. Visited=100, unvisited=0.
        data = (grid * 100).astype(np.int8).flatten().tolist()
        msg.data = data
        self.visited_grid_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PolicyBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.executor_act.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
