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

from policy_bridge.action_gate import (
    action7_free_run_blocks,
    action7_sensor_blocks,
    gate_blocks,
    movement_clearance_m,
)
from policy_bridge.am_adapter import ActiveMappingAdapter
from policy_bridge.display_map import build_display_map, display_coverage
from policy_bridge.obs_builder import ObsBuilder
from policy_bridge.world_config import WorldGeometry, load_world_geometry
from policy_bridge.visited_grid import VisitedGridBuilder
from policy_bridge.action_executor import (
    ACTION7_WALL_MARGIN_M,
    ActionExecutor,
    InvalidActionError,
)
from policy_bridge.coverage import Coverage
from policy_bridge.failure_modes import FailureHandler
from policy_bridge.adaptive_speed import AdaptiveSpeedController, classify_mode
from policy_bridge.stuck_detector import StuckDetector
from policy_bridge.wall_follower import WallFollower
from policy_bridge.wall_map_builder import WallMapBuilder
from policy_bridge.phase_controller import PhaseController, Phase


PARAM_DEFAULTS: dict[str, object] = {
    "model_path": "",
    # Блок Б (2026-06-07): per-world геометрия из config/worlds.yaml.
    # world_name "" → env DEFAULT_WORLD; мир без записи в yaml = fail-fast.
    # room_size/cell_size стали ИНФОРМАЦИОННЫМИ (yaml выигрывает, расхождение
    # = warn) — оставлены для совместимости старых launch-вызовов.
    "world_name": "",
    "worlds_config": "auto",
    "room_size": 6.4,
    "cell_size": 0.1,
    # v1.5c: терминация эпизода по mapped_ratio (env: coverage>95% → done).
    # Пост-терминальное поведение модели вырождено (frontiers исчерпаны,
    # ран 3: stall-шторм после mapped 0.975) — миссия завершена, hover.
    "mapped_success_threshold": 0.95,
    # Блок 2 (Aleks 2026-06-07): mission-done по DISPLAY-карте (дорисованной).
    # Ниже parity-0.95 — display заполнена плотнее (углы/дыры<проёма закрыты).
    "display_success_threshold": 0.92,
    # Alignment §3.1: фильтр мелких frontier-кластеров (parity-карта, в obs).
    # 1 = bit-exact с историческим env (v1.5c, AM-4 фикстуры). 3 = aligned-
    # build ПОСЛЕ retrain (env + bridge оба на 3). Меняет obs → НЕ ставить 3
    # пока модель не дообучена с фильтром (иначе off-distribution).
    "min_frontier_cluster_cells": 1,
    # v2-stub (Aleks 2026-06-08): occupancy-free_run sensor-gate для action7.
    # default FALSE = текущее RAW-сенсорное поведение (action7_sensor_blocks).
    # При v2-экспорте → True: маска по free_cells(ch0) > N (§3.2 v2, точное
    # зеркало train, без timing-jitter). wall_stop_cells = N (граница).
    "v2_sensor_mask": False,
    "wall_stop_cells": 6,
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
    # v1.5c deploy (2026-06-07): модельная семья.
    #   sweep02       — legacy PPO Dict obs (distances+servo+visited)
    #   activemapping — MaskablePPO Box(21,) + occupancy/frontier (AM-v1);
    #                   predict ОБЯЗАН получать action_masks (F1), mode
    #                   принудительно rl_only (wall-phase не в тренировке).
    "model_family": "activemapping",
    # AM eval-эталоны rl-lab сняты deterministic=True; sweep02 летал False.
    "deterministic": True,
    # StuckDetector escape-инъекции: auto = только sweep02 (для AM ломает
    # распределение — у модели есть frontier obs, меряем ЕЁ поведение).
    "stuck_escape": "auto",
}

# v1.5c livelock breaker (ран 2026-06-07 15:0x RCA): deterministic policy +
# физический отказ исполнения (front ≤ margin → action 7 «no travel») =
# замороженный obs → модель вечно повторяет одно действие (2800 шагов у
# стены, 1558 no-travel warn'ов). В env такого состояния НЕТ — env двигает
# дрона до соседней со стеной клетки, а наш safety-слой держит margin 0.7 м.
# Решение: движенческое действие, не давшее смены клетки NOOP_MASK_LIMIT раз
# подряд, временно убирается из action_mask (динамическая маска — штатный
# режим MaskablePPO) до смены клетки ИЛИ накопленного поворота ≥ 45°.
# Release по ОДНОЙ ротации (первая версия) дал corner-dance: 15° мало,
# gate блокирует снова → цикл 3 блока + 1 ротация навечно (2953 gate-block
# в ране 15:2x). 45° = 3 ротации — направление действия реально сменилось.
NOOP_MASK_LIMIT = 3
RELEASE_HEADING_DEG = 45.0
MOVEMENT_ACTIONS = (0, 1, 2, 3, 7)
# Backstop: rot-осцилляция (+15/−15) не копит Δheading и не меняет клетку —
# детерминированный argmax может зациклиться и на ротациях. После
# STALL_STEPS шагов без смены клетки predict один раз сэмплирует из
# распределения модели (deterministic=False) — выход из цикла действиями
# самой модели, не хардкодом.
STALL_STEPS = 30


class PolicyBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("policy_bridge_node")
        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.model_path = str(self.get_parameter("model_path").value)
        self.room_size = float(self.get_parameter("room_size").value)
        self.cell_size = float(self.get_parameter("cell_size").value)
        self.mapped_success_threshold = float(
            self.get_parameter("mapped_success_threshold").value
        )
        self.display_success_threshold = float(
            self.get_parameter("display_success_threshold").value
        )
        self._mission_complete = False
        self._last_display = None       # Блок 2: дорисованная display-карта
        self._last_disp_cov = 0.0
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
        self.v2_sensor_mask = bool(self.get_parameter("v2_sensor_mask").value)
        self.wall_stop_cells = int(self.get_parameter("wall_stop_cells").value)
        self._gate_block_count = 0
        self._last_mapped = 0.0
        # v1.5c livelock breaker (см. NOOP_MASK_LIMIT)
        self._noop_streak: dict[int, int] = {}
        # action -> heading на момент маскировки (release: cell change ИЛИ
        # |Δheading| ≥ RELEASE_HEADING_DEG — направление реально сменилось)
        self._infeasible_actions: dict[int, float] = {}
        self._cell_stall_count = 0
        self._stall_kick_count = 0
        self._a7_sensor_mask_count = 0   # v2 sensor gate action7 (Aleks 2026-06-08)
        # v1.5c deploy
        self.model_family = str(self.get_parameter("model_family").value)
        if self.model_family not in ("sweep02", "activemapping"):
            raise ValueError(f"model_family={self.model_family!r} — "
                             "ожидаю sweep02 | activemapping")
        self.deterministic = bool(self.get_parameter("deterministic").value)
        stuck_escape = str(self.get_parameter("stuck_escape").value)
        if stuck_escape not in ("auto", "on", "off"):
            raise ValueError(f"stuck_escape={stuck_escape!r} — auto|on|off")
        self.stuck_enabled = stuck_escape == "on" or (
            stuck_escape == "auto" and self.model_family == "sweep02"
        )
        if self.model_family == "activemapping" and self.mode != "rl_only":
            self.get_logger().warn(
                f"model_family=activemapping несовместим с mode={self.mode!r} "
                "(wall-phase не в тренировке AM-v1) — форсирую rl_only"
            )
            self.mode = "rl_only"

        # ----- Блок Б: per-world геометрия (worlds.yaml, fail-fast) -----
        self.geom = self._load_geometry()
        # yaml выигрывает над legacy-скалярами; расхождение = warn
        if abs(self.geom.room_x_m - self.room_size) > 1e-6 and \
                self.room_size != float(PARAM_DEFAULTS["room_size"]):
            self.get_logger().warn(
                f"param room_size={self.room_size} игнорируется — worlds.yaml "
                f"{self.geom.world_name}: {self.geom.room_x_m}×{self.geom.room_y_m} м"
            )
        self.room_size = self.geom.room_x_m       # legacy-поля (квадратные вызовы)
        self.cell_size = self.geom.resolution_m
        self.room_x = self.geom.room_x_m
        self.room_y = self.geom.room_y_m
        self.nx = self.geom.nx
        self.ny = self.geom.ny

        self.grid_size = self.nx
        if not self.geom.is_model_canon:
            self.get_logger().warn(
                f"мир {self.geom.world_name}: грид {self.nx}×{self.ny} @ "
                f"{self.cell_size} м ≠ модельный канон 64×64 @ 0.1 — модель "
                f"({self.model_family}) работает OOD: гео-слои bridge "
                "(safe-box/visited/occupancy) по миру, obs-нормализации — "
                "по контракту модели."
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
            room_x_m=self.room_x, room_y_m=self.room_y,
            nx=self.nx, ny=self.ny,
        )
        self.coverage = self._build_coverage()

        # ----- ActiveMapping adapter (v1.5c deploy 2026-06-07) -----
        # Box(21,) obs + occupancy/frontier + action_masks (протокол v1.0).
        # free_mask ОБЯЗАТЕЛЕН (§5.1 mapped_ratio) — без него fail-fast,
        # никаких тихих fallback'ов. servo_deg — late-bound c executor'а
        # (создаётся ниже), вызовы идут только в runtime.
        self.am_adapter: ActiveMappingAdapter | None = None
        if self.model_family == "activemapping":
            self.am_adapter = ActiveMappingAdapter(
                room_size_m=self.room_x,
                room_y_m=self.room_y,
                cell_size_m=self.cell_size,
                free_mask=self.coverage.free_mask,
                get_pose=lambda: self.obs_builder.pose,
                get_vl_raw_m=lambda: self.obs_builder.perimeter_distances_m,
                get_tf_raw_m=lambda: self.obs_builder.sweep_distance_m,
                get_servo_deg=lambda: self.executor_act.servo_deg,
                min_frontier_cluster_cells=int(
                    self.get_parameter("min_frontier_cluster_cells").value
                ),
            )

        self.executor_act = ActionExecutor(
            self,
            cell_size_m=self.cell_size,
            wall_threshold=self.wall_threshold,
            get_front_distance_m=lambda: self.obs_builder.front_distance_m,
            # D-refactor (2026-05-20): position setpoint control. Pass pose accessor
            # для arrival check + initial target.
            get_pose=lambda: self.obs_builder.pose,
            # Блок Б: action7 safety cap = grid_size*cell — большая ось мира
            grid_size=max(self.nx, self.ny),
            # Legacy (ignored в position control but kept в signature)
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
            get_yaw_rad=lambda: self.obs_builder.pose.heading_rad,
            # v2 Block 2: training parity — env помечает visited все клетки
            # пройденные за action (включая промежуточные у action 7).
            # v1.5c: для AM тот же хук дополнительно интегрирует occupancy
            # при каждой НОВОЙ клетке (§2.5 п.2 — action 7 / translations).
            visited_update_fn=self._on_pose_update,
            # v2 run F: velocity-gated arrival (Aleks 08:26)
            get_speed_m_s=lambda: self.obs_builder.speed_m_s,
            # v2-stub (Aleks 2026-06-08): action7 travel по occupancy free_run.
            # default off (v2_sensor_mask=False) → текущее raw-поведение.
            v2_sensor_mask=self.v2_sensor_mask,
            wall_stop_cells=self.wall_stop_cells,
            get_free_run_cells=lambda: self.am_adapter.free_run_ch0(),
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
            room_y_m=self.room_y,
            ny=self.ny,
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
        # v1.5c: сырое действие политики ДО stuck/adaptive/gate — для чистого
        # профиля действий (analyze: частоты 0-7, strafe vs rotate).
        self.action_raw_pub = self.create_publisher(
            Int32, "/rl_policy/action_raw", 10
        )
        self.coverage_pub = self.create_publisher(Float32, "/rl_policy/coverage", 10)
        # v1.5c: mapped_ratio (§5.1) — собственная метрика модели AM.
        self.mapped_ratio_pub = self.create_publisher(
            Float32, "/rl_policy/mapped_ratio", 10
        )
        # GIF-карты (запрос Aleks 2026-06-07 ~18:10): occupancy AM-эпизода
        # для offline-сборки 2D-анимаций (make_map_gif.py). Конвенция
        # OccupancyGrid: UNKNOWN→-1, FREE→0, OCCUPIED→100.
        self.occupancy_pub = self.create_publisher(
            OccupancyGrid, "/rl_policy/occupancy_grid", 10
        )
        self.visited_grid_pub = self.create_publisher(
            OccupancyGrid, "/rl_policy/visited_grid", 10
        )

        # ----- timer (cb groups уже declared выше до ObsBuilder construction) -----
        period_s = 1.0 / self.rate_hz
        self.timer = self.create_timer(period_s, self._tick, callback_group=self.timer_cb_group)

        self.get_logger().info(
            f"policy_bridge_node ready · family={self.model_family} · "
            f"mode={self.mode} · deterministic={self.deterministic} · "
            f"stuck_escape={'on' if self.stuck_enabled else 'off'} · "
            f"rate={self.rate_hz} Hz · room {self.room_x}×{self.room_y} m · "
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

    # ---- world geometry (Блок Б) --------------------------------------------

    def _load_geometry(self) -> WorldGeometry:
        world = str(self.get_parameter("world_name").value) or os.environ.get(
            "DEFAULT_WORLD", ""
        )
        cfg = str(self.get_parameter("worlds_config").value)
        if cfg in ("", "auto"):
            sim_root = os.environ.get(
                "AEROSEARCH_ROOT", "/data/git/aerosearch"
            ) + "/claudedrone-git/simulation"
            cfg = f"{sim_root}/src/policy_bridge/config/worlds.yaml"
        geom = load_world_geometry(cfg, world)
        self.get_logger().info(
            f"world geometry: {geom.world_name} · {geom.room_x_m}×{geom.room_y_m} м "
            f"@ {geom.resolution_m} → грид {geom.nx}×{geom.ny}"
            f"{' · model canon' if geom.is_model_canon else ' · ⚠ НЕ канон 64×64'}"
        )
        return geom

    # ---- model loading -----------------------------------------------------

    def _load_model(self):
        if not self.model_path:
            raise RuntimeError("model_path param empty — set it via launch arg.")
        path = Path(self.model_path)
        if not path.exists():
            raise FileNotFoundError(f"model.zip not found: {path}")
        # Late imports чтобы node мог быть импортирован без SB3 (unit тесты).
        if self.model_family == "activemapping":
            from sb3_contrib import MaskablePPO
            self.get_logger().info(
                f"loading MaskablePPO (AM-v1) from {path} (device=cpu, "
                f"deterministic={self.deterministic})"
            )
            model = MaskablePPO.load(str(path), device="cpu")
            obs_shape = tuple(model.observation_space.shape)
            if obs_shape != (21,):
                raise ValueError(
                    f"model obs space {obs_shape} ≠ (21,) — это не "
                    "ActiveMapping-v1 модель? Проверь model_path/model_family."
                )
            return model
        from stable_baselines3 import PPO
        self.get_logger().info(f"loading PPO from {path} (device=cpu)")
        return PPO.load(str(path), device="cpu")

    def _on_pose_update(self, x_m: float, y_m: float) -> None:
        """Hook ActionExecutor'а на каждом poll'е arrival-ожидания (20 Hz):
        visited (training parity, v2 Block 2) + occupancy при смене клетки
        (AM, §2.5 п.2)."""
        self.visited.update(x_m, y_m)
        if self.am_adapter is not None:
            self.am_adapter.on_pose_update(x_m, y_m)

    # ---- v1.5c livelock breaker ---------------------------------------------

    def _cell_of(self, pose) -> tuple[int, int]:
        return (
            int((pose.x_m + self.room_x / 2.0) / self.cell_size),
            int((pose.y_m + self.room_y / 2.0) / self.cell_size),
        )

    def _feasibility_update(self, action: int, moved: bool) -> None:
        """Учёт no-op'ов исполнения. moved = клетка дрона сменилась за шаг."""
        if moved:
            self._noop_streak.clear()
            self._cell_stall_count = 0
            if self._infeasible_actions:
                self.get_logger().info(
                    "feasibility mask released (cell change): "
                    f"{sorted(self._infeasible_actions)}"
                )
                self._infeasible_actions.clear()
            return
        self._cell_stall_count += 1
        # ротации меняют body→world направления — release только после
        # НАКОПЛЕННОГО поворота ≥ RELEASE_HEADING_DEG (см. RCA corner-dance)
        heading = self.obs_builder.pose.heading_rad
        released = [
            a for a, h0 in self._infeasible_actions.items()
            if abs(math.degrees(self._angle_diff(heading, h0)))
            >= RELEASE_HEADING_DEG
        ]
        for a in released:
            del self._infeasible_actions[a]
            self._noop_streak.pop(a, None)
        if released:
            self.get_logger().info(
                f"feasibility mask released (Δheading ≥ {RELEASE_HEADING_DEG}°): "
                f"{sorted(released)}"
            )
        if action not in MOVEMENT_ACTIONS:
            return
        n = self._noop_streak.get(action, 0) + 1
        self._noop_streak[action] = n
        if n >= NOOP_MASK_LIMIT and action not in self._infeasible_actions:
            self._infeasible_actions[action] = heading
            self.get_logger().warn(
                f"feasibility mask: action {action} no-op ×{n} подряд — "
                f"маскирую до смены клетки / Δheading {RELEASE_HEADING_DEG}°"
            )

    def _build_coverage(self) -> Coverage:
        path = self.free_mask_path
        kw = {"grid_size": self.grid_size, "nx": self.nx, "ny": self.ny}
        if path in ("", "none"):
            return Coverage(free_mask_path=None, **kw)
        if path == "auto":
            # v2 Block 2 (2026-06-06): auto-resolve. Блок Б: имя мира берём
            # из world geometry (worlds.yaml), не напрямую из env.
            # Без маски coverage = visited/total занижает прогресс ~втрое
            # (free cells ≈ 1/3 грида) — и порог COVERAGE_TARGET недостижим.
            world = self.geom.world_name
            sim_root = os.environ.get(
                "AEROSEARCH_ROOT", "/data/git/aerosearch"
            ) + "/claudedrone-git/simulation"
            candidate = (
                Path(sim_root)
                / "src/drone_sim/worlds/rl_rooms" / world / "free_mask.png"
            )
            if world and candidate.exists():
                self.get_logger().info(f"free_mask_path=auto → {candidate}")
                return Coverage(free_mask_path=str(candidate), **kw)
            self.get_logger().warn(
                f"free_mask_path=auto: не нашёл {candidate} "
                f"(world={world!r}) — coverage без маски"
            )
            return Coverage(free_mask_path=None, **kw)
        return Coverage(free_mask_path=path, **kw)

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
            # v1.5c: §2.5 п.3 — начало эпизода: клетка спавна FREE +
            # первичный взгляд (7 лучей) из hover-позы.
            if self.am_adapter is not None:
                self.am_adapter.reset_episode()
                self.get_logger().info(
                    "AM adapter: episode reset + первичный взгляд "
                    f"(integrations={self.am_adapter.integrations})"
                )

    def _tick(self) -> None:
        if self.step_count >= self.max_steps:
            return

        # v1.5c: миссия завершена (mapped ≥ threshold) — hover, не predict'им.
        if self._mission_complete:
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
        # Блок Б: per-axis извлечения из worlds.yaml (раньше хардкод 6.4 убивал
        # полёт в indoor_room 16×10 — 7800 ERROR-строк hover на x>3.7)
        half_x = self.room_x / 2.0 + self.safe_box_margin_m
        half_y = self.room_y / 2.0 + self.safe_box_margin_m
        out_of_box = (
            self.obs_builder.has_received_odom
            and (abs(pose.x_m) > half_x or abs(pose.y_m) > half_y)
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
            self.failure.trigger_pose_out_of_box(
                pose.x_m, pose.y_m, max(half_x, half_y)
            )
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

    def _free_run_cells_ch0(self) -> int:
        """v2-stub (Aleks 2026-06-08): free_run ch0 целых FREE-клеток для
        action7-маски §3.2 v2 (occupancy, не raw). Зовётся только при
        v2_sensor_mask=True (default off). ⚠ финализировать геометрию против
        v2 parity-фикстур при v2-экспорте."""
        return self.am_adapter.free_run_ch0()

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

        if self.am_adapter is not None:
            # v1.5c: §2.5 п.1 — интеграция на step boundary (снапшот obs
            # ПОСЛЕ завершения предыдущего действия), затем obs+mask одним
            # BFS-расчётом. predict БЕЗ action_masks запрещён (F1).
            self.am_adapter.integrate_now()
            obs, action_mask, mapped = self.am_adapter.snapshot()
            # Блок 2: mission-done по DISPLAY-карте (дорисованной), не по
            # parity. Display заполнена плотнее (углы достроены, дыры < проёма
            # закрыты) → порог 0.92 ниже parity-0.95; дрон перестаёт гонять
            # за угловыми пикселями. Модель продолжает obs из parity (выше).
            disp = build_display_map(
                self.am_adapter.builder.occ, self.cell_size
            )
            disp_cov = display_coverage(disp, self.coverage.free_mask)
            self._last_display = disp
            self._last_disp_cov = disp_cov
            if disp_cov >= self.display_success_threshold:
                self._mission_complete = True
                self.mapped_ratio_pub.publish(Float32(data=mapped))
                self._publish_occupancy()  # финальная display-карта в GIF
                self.get_logger().info(
                    f"🏁 MISSION COMPLETE: display_cov {disp_cov:.3f} ≥ "
                    f"{self.display_success_threshold} (parity mapped "
                    f"{mapped:.3f}) на шаге {self.step_count} — hover"
                )
                return
            # Sensor gate action7 (Aleks v2 2026-06-08): маскируем action7 ДО
            # predict, если front-сенсор внутри executor's ЭФФЕКТИВНОЙ маржи
            # (= max(ACTION7_WALL_MARGIN_M, текущий mode wall_threshold) —
            # читаем из режима, НЕ хардкод). Рассинхрон порогов давал no-travel
            # в зоне [0.60, mode_wt] (3 события @0.67-0.68 в N=6 acceptance).
            # См. action_gate.action7_sensor_blocks.
            # v2-stub (Aleks 2026-06-08): при v2_sensor_mask=True маска action7
            # по occupancy free_run (§3.2 v2, точное зеркало train, без
            # timing-jitter). Default False = RAW-сенсорный путь ниже (b3a1c6d).
            perim = self.obs_builder.perimeter_distances_m
            a7_sensor_masked = False
            a7_margin = 0.0
            if bool(action_mask[7]):
                if self.v2_sensor_mask:
                    free_cells = self._free_run_cells_ch0()
                    a7_sensor_masked = action7_free_run_blocks(
                        free_cells, self.wall_stop_cells
                    )
                elif perim and len(perim) >= 6:
                    eff_wt = self.adaptive_speed.mode_table[
                        classify_mode(min(perim))
                    ].wall_threshold
                    a7_margin = max(ACTION7_WALL_MARGIN_M, eff_wt)
                    a7_sensor_masked = action7_sensor_blocks(perim[0], a7_margin)
            if self._infeasible_actions or a7_sensor_masked:
                action_mask = action_mask.copy()
                for a in self._infeasible_actions:
                    action_mask[a] = False
                # маскируем action7 только если останется ≥1 валидное действие
                # (rotate 4/5 сенсором не гейтятся → почти всегда True).
                if a7_sensor_masked and int(action_mask.sum()) > 1:
                    action_mask[7] = False
                    self._a7_sensor_mask_count += 1
                    if self._a7_sensor_mask_count % 10 == 1:
                        why = ("free_run" if self.v2_sensor_mask
                               else f"front={perim[0]:.2f}m<{a7_margin:.2f}m")
                        self.get_logger().info(
                            f"sensor gate: action7 masked ({why}) "
                            f"×{self._a7_sensor_mask_count}"
                        )
            deterministic = self.deterministic
            if deterministic and self._cell_stall_count >= STALL_STEPS:
                deterministic = False  # stall backstop: сэмпл из распределения
                self._stall_kick_count += 1
                self.get_logger().warn(
                    f"stall backstop: {self._cell_stall_count} шагов без смены "
                    f"клетки — stochastic predict (kick #{self._stall_kick_count})"
                )
            action_arr, _ = self.model.predict(
                obs, action_masks=action_mask, deterministic=deterministic
            )
            self.mapped_ratio_pub.publish(Float32(data=mapped))
            self._last_mapped = mapped
            self._publish_occupancy()
        else:
            obs = self.obs_builder.build_obs(self.visited.grid)
            action_arr, _ = self.model.predict(obs, deterministic=False)
        raw_action_from_policy = int(action_arr)
        self.action_raw_pub.publish(Int32(data=raw_action_from_policy))

        # TASK-059 attempt #8 escape v2 (rl-lab @03:18): StuckDetector v2 со
        # smart escape — scoring direction via (free_dist + unvisited_in_cone),
        # rotate→sweep OR fallback backward. Возвращает (action, escape_active)
        # для B2 bypass в adaptive_speed.
        cov_for_stuck = self.coverage.compute(self.visited.grid)
        pose = self.obs_builder.pose
        # Convert pose to cell ix/iy (matches VisitedGridBuilder formula)
        pos_ix = int((pose.x_m + self.room_x / 2.0) / self.cell_size)
        pos_iy = int((pose.y_m + self.room_y / 2.0) / self.cell_size)
        # Clip к grid bounds (drone может быть outside в edge cases)
        pos_ix = max(0, min(pos_ix, self.nx - 1))
        pos_iy = max(0, min(pos_iy, self.ny - 1))

        perimeter_distances = self.obs_builder.perimeter_distances_m
        if self.stuck_enabled:
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
        else:
            # v1.5c AM: escape-инъекции выключены (stuck_escape=auto) —
            # измеряем поведение модели, не харнесса.
            action, escape_active = raw_action_from_policy, False

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
            # gate-отказ = нет смещения — учитываем в livelock breaker
            if self.am_adapter is not None:
                self._feasibility_update(action_mod, moved=False)
            return

        # Publish EXECUTED action (post-stuck/adaptive) для policy logging
        self.action_pub.publish(Int32(data=action_mod))

        # Перед action 7 (длинный move) — snap heading на ОСЬ (90°), не на
        # 15°-решётку (Aleks Блок 2, обосновано раном B): осевые заходы дают
        # чистые прямые вдоль стен вместо косых «ёлочкой». Ротации (4/5)
        # остаются на 15° — модель смотрит в 24 направлениях, но длинный
        # move летит строго по оси.
        if action_mod == 7:
            step = math.radians(90.0)
            snapped = round(pose.heading_rad / step) * step
            self.executor_act.snap_to_yaw(snapped)

        cell_before = self._cell_of(pose)
        self.executor_act.execute(
            action_mod,
            override_speed=cfg.linear_speed,
            override_wall_threshold=cfg.wall_threshold,
        )
        if self.am_adapter is not None:
            moved = self._cell_of(self.obs_builder.pose) != cell_before
            self._feasibility_update(action_mod, moved)

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
            mapped_tag = (
                f" · mapped {self._last_mapped:.3f}"
                if self.am_adapter is not None else ""
            )
            self.get_logger().info(
                f"step {self.step_count} · action {action_str}{escape_tag} · "
                f"mode {mode.value} (v={cfg.linear_speed:.2f}m/s wt={cfg.wall_threshold:.2f}m) · "
                f"coverage {cov:.3f}{mapped_tag} · escape_total={self.stuck_detector.escape_count_total}"
            )

    def _publish_occupancy(self) -> None:
        """Occupancy для GIF/трека — Блок 2: DISPLAY-карта (дорисованная),
        не parity. Модель parity не теряет (она в am_adapter, отдельно)."""
        occ = (
            self._last_display
            if self._last_display is not None
            else self.am_adapter.builder.occ
        )
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.cell_size
        msg.info.width = occ.shape[1]
        msg.info.height = occ.shape[0]
        msg.info.origin.position.x = -self.room_x / 2.0
        msg.info.origin.position.y = -self.room_y / 2.0
        msg.info.origin.orientation.w = 1.0
        # UNKNOWN(0)→-1, FREE(1)→0, OCCUPIED(2)→100
        lut = np.array([-1, 0, 100], dtype=np.int8)
        msg.data = lut[occ].flatten().tolist()
        self.occupancy_pub.publish(msg)

    def _publish_visited_grid(self, grid: np.ndarray) -> None:
        """Отправляет grid как OccupancyGrid для Foxglove визуализации."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.cell_size
        msg.info.width = self.nx
        msg.info.height = self.ny
        # Origin = SW corner мира.
        msg.info.origin.position.x = -self.room_x / 2.0
        msg.info.origin.position.y = -self.room_y / 2.0
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
