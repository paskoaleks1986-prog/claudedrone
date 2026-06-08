"""sitl_comm.py — MavrosSITLComm: живой ROS2/MAVROS-слой под Protocol SITLComm
(rl-lab envs/sitl_drone_env.py, спринт SITL-RL).

Архитектура (Aleks 2026-06-08): SITLDroneEnv ROS2-free, brain (obs/mask/predict +
occupancy) живёт в нём через InferenceCore. Этот модуль — ИНЪЕКТИРУЕМЫЙ comm-слой:
тонкая обёртка вокруг живого стека (Gazebo+ArduPilot SITL+MAVROS+sensors).

Переиспользует существующие компоненты ноды (НЕ дублирует логику):
    ObsBuilder       — те же сенсорные подписки (/drone/perimeter, /scan/sweep, odom);
    ActionExecutor   — position-setpoint исполнение 8 действий @10Hz (D-refactor);
    action_gate      — gate_blocks / raw_free_runs (единая free_run-формула с env).
Takeoff/land/arm — драйв ЗДЕСЬ через MAVROS-сервисы, логикой/константами
takeoff_node (EKF-settle 8s, NAV_TAKEOFF, hover-stabilize, hold-in-place). Сам
takeoff_node НЕ поднимаем как процесс — один владелец /mavros/setpoint_position/local.

⛔ Правило Aleks: первый SITL-ран только после ручного flight-smoke
   (reset→takeoff→один step→hover). Этот модуль + help_scripts/smoke_flight.py.

Контракт Protocol (что гарантируем SITLDroneEnv):
    start_episode(*, randomize_spawn, rng) -> {pose_m:(x,y,heading), distances:[7], servo_deg}
        land(если летим)→disarm→[reposition]→arm→NAV_TAKEOFF→climb→hover-stabilize→settle.
    read_state() -> {pose_m, distances:[vl0..5, tf] RAW sensor-frame метры, servo_deg}
        distances RAW (build_obs/_integrate сами +mount, coord 18:02). servo_deg = commanded.
    execute(action) -> {travel_cells:int, collided:bool, crashed:bool}
        travel_cells = round(|displacement|/cell). collided = translation(0-3) с free_run
        в направлении ≤ N (gate_blocks; физически НЕ летим в стену — защита + parity
        reward.blocked). crashed = disarm / tilt>limit / z-collapse / out-of-box.
    hard_reset() -> kill+launch стека (relaunch_cmd) или soft land+disarm+warn.
    close() -> shutdown.

Запуск стека под env (рекоменд.): launch.sh --full --mavros --no-autoscan -w <world> -d
(Gazebo+SITL+bridge+drone.launch[sensors,safety_guard]+mavros; БЕЗ takeoff_node/
policy_bridge_node — полётом владеет этот comm). Затем train.py конструирует
MavrosSITLComm на живой MAVROS.
"""
from __future__ import annotations

import math
import subprocess
import threading
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import Imu

from policy_bridge.action_executor import ActionExecutor
from policy_bridge.action_gate import gate_blocks, raw_free_runs, VL_MOUNT_RADIUS_M
from policy_bridge.obs_builder import ObsBuilder


# ── константы взлёта/посадки (= takeoff_node, single source при правках сверять) ─
TARGET_ALTITUDE_M = 2.0       # NAV_TAKEOFF target (takeoff_node.TARGET_ALTITUDE)
CLIMB_ARRIVAL_M = 1.8         # z ≥ это = climb done (takeoff_node.CLIMB_ARRIVAL_M)
CLIMB_TIMEOUT_S = 15.0
EKF_SETTLE_S = 8.0            # после GUIDED ждём схождения EKF/гиро перед arm
HOVER_STABILIZE_S = 5.0       # z в полосе НЕПРЕРЫВНО столько = stable
HOVER_Z_BAND_M = 0.5
HOVER_MAX_WAIT_S = 25.0
CMD_RETRY_S = 1.0             # интервал ретрая mavros-сервисов
SERVICE_WAIT_S = 20.0         # дождаться появления mavros-сервиса
LAND_Z_M = 0.30              # z ниже = приземлился
LAND_TIMEOUT_S = 20.0
DISARM_GRACE_S = 2.0          # пауза после disarm (EKF shutdown)

# crash-детект
CRASH_TILT_DEG = 35.0         # roll/pitch круче = переворот (>safety_guard 15° avoidance)
CRASH_Z_FRACTION = 0.4        # z < frac×target в полёте = просадка/crash
SAFE_BOX_MARGIN_M = 0.6       # |pose| > room/2 + это = вылет

LINEAR_SPEED_M_S = 0.3        # carrot-скорость движений (= bridge default)
REPOSITION_TIMEOUT_S = 25.0   # перелёт к random-spawn (через всю комнату)
SPAWN_WALL_MARGIN_M = 0.7     # random-spawn не ближе к стенам


def spawn_cell_to_m(
    col: int, row: int, cell_size_m: float, room_x_m: float, room_y_m: float
) -> tuple[float, float]:
    """Центр клетки (col=x, row=y) occupancy-грида → метры (origin=центр комнаты).
    Инверсия env m_to_cells: x_m = (col+0.5)*cell − room_x/2."""
    x_m = (col + 0.5) * cell_size_m - room_x_m / 2.0
    y_m = (row + 0.5) * cell_size_m - room_y_m / 2.0
    return x_m, y_m


def displacement_cells(dx_m: float, dy_m: float, cell_size_m: float) -> int:
    """Евклидово смещение → целые клетки (round). travel_cells для reward/no_travel."""
    return int(round(math.hypot(dx_m, dy_m) / cell_size_m))


def _tilt_rad_from_quat(qw: float, qx: float, qy: float, qz: float) -> float:
    """max(|roll|, |pitch|) из кватерниона (как safety_guard._imu_cb)."""
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    return max(abs(roll), abs(pitch))


class MavrosSITLComm:
    """Живой SITLComm — реализация Protocol для SITLDroneEnv (fine-tune в Gazebo).

    Конструируется на УЖЕ поднятый стек (launch.sh ...). Создаёт свою ROS2-ноду,
    крутит MultiThreadedExecutor в фоновом потоке (callbacks + 10Hz maintenance
    ActionExecutor'а), а start_episode/execute/read_state вызываются из
    тренировочного потока и блокируют.
    """

    def __init__(
        self,
        *,
        room_x_m: float,
        room_y_m: float,
        cell_size_m: float = 0.1,
        free_mask: np.ndarray | None = None,
        target_altitude_m: float = TARGET_ALTITUDE_M,
        wall_stop_cells: int = 6,
        gate_margin_m: float = 0.55,
        mount_radius_m: float = VL_MOUNT_RADIUS_M,
        linear_speed_m_s: float = LINEAR_SPEED_M_S,
        sitl_instance: int = 1,
        node_name: str = "sitl_comm",
        relaunch_cmd: str | None = None,
        perimeter_topic: str = "/drone/perimeter",
        sweep_topic: str = "/scan/sweep",
        odom_topic: str = "/mavros/local_position/odom",
        ekf_settle_s: float = EKF_SETTLE_S,
        hover_stabilize_s: float = HOVER_STABILIZE_S,
        verbose: bool = True,
    ) -> None:
        self.room_x = float(room_x_m)
        self.room_y = float(room_y_m)
        self.cell_size = float(cell_size_m)
        self.free_mask = None if free_mask is None else np.asarray(free_mask, dtype=bool)
        self.target_altitude = float(target_altitude_m)
        self.wall_stop_cells = int(wall_stop_cells)
        self.gate_margin_m = float(gate_margin_m)
        self.mount_radius_m = float(mount_radius_m)
        self.linear_speed = float(linear_speed_m_s)
        self.sitl_instance = int(sitl_instance)
        self.relaunch_cmd = relaunch_cmd
        self.ekf_settle_s = float(ekf_settle_s)
        self.hover_stabilize_s = float(hover_stabilize_s)
        self.verbose = verbose

        # ── rclpy context (инициализируем только если ещё не поднят) ──
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self.node = Node(node_name)
        self._log = self.node.get_logger()
        self._cb_group = ReentrantCallbackGroup()

        # ── state из mavros + imu ──
        self._state = State()
        self._tilt_rad = 0.0
        self._airborne = False
        self.node.create_subscription(
            State, "/mavros/state", self._state_cb, 10, callback_group=self._cb_group
        )
        self.node.create_subscription(
            Imu, "/mavros/imu/data", self._imu_cb, qos_profile_sensor_data,
            callback_group=self._cb_group,
        )

        # ── сенсоры (тот же ObsBuilder, что в ноде) ──
        self.obs_builder = ObsBuilder(
            self.node,
            perimeter_topic=perimeter_topic,
            sweep_topic=sweep_topic,
            odom_topic=odom_topic,
            callback_group=self._cb_group,
        )

        # ── исполнитель действий (position setpoints @10Hz) ──
        # v2_sensor_mask=True: action7 travel = (free_cells−N)*cell (зеркало train),
        # get_free_run_cells = свежий ch0 free_run (= формула env/маски).
        self.executor_act = ActionExecutor(
            self.node,
            cell_size_m=self.cell_size,
            wall_threshold=self.gate_margin_m,
            get_front_distance_m=lambda: self.obs_builder.front_distance_m,
            get_pose=lambda: self.obs_builder.pose,
            target_altitude_m=self.target_altitude,
            grid_size=int(round(max(self.room_x, self.room_y) / self.cell_size)),
            linear_speed=self.linear_speed,
            get_speed_m_s=lambda: self.obs_builder.speed_m_s,
            v2_sensor_mask=True,
            wall_stop_cells=self.wall_stop_cells,
            get_free_run_cells=self._front_free_run,
        )
        # servo obs = commanded angle executor'а (training parity, как нода)
        self.obs_builder.set_servo_angle_source(lambda: self.executor_act.servo_deg)

        # ── safety_guard cooperation (latched /safety/active) ──
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
        from std_msgs.msg import Bool
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.node.create_subscription(
            Bool, "/safety/active",
            lambda m: self.executor_act.set_safety_hold(bool(m.data)),
            latched, callback_group=self._cb_group,
        )

        # ── mavros service clients ──
        self._arm_cli = self.node.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_cli = self.node.create_client(SetMode, "/mavros/set_mode")
        self._takeoff_cli = self.node.create_client(CommandTOL, "/mavros/cmd/takeoff")

        # ── фоновый spin ──
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        self._rng = np.random.default_rng()
        if self.verbose:
            self._log.info(
                f"MavrosSITLComm up · room {self.room_x}×{self.room_y}m @ {self.cell_size} · "
                f"target_alt={self.target_altitude}m · N={self.wall_stop_cells} · "
                f"gate={self.gate_margin_m}m · instance={self.sitl_instance}"
            )

    # ── фоновый поток ─────────────────────────────────────────────────────────
    def _spin(self) -> None:
        while not self._spin_stop.is_set() and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _state_cb(self, msg: State) -> None:
        self._state = msg

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        self._tilt_rad = _tilt_rad_from_quat(q.w, q.x, q.y, q.z)

    def _front_free_run(self) -> int:
        """ch0 free_run (центр-референс, как маска/env) для action7 travel."""
        return raw_free_runs(
            self.obs_builder.perimeter_distances_m,
            cell_size_m=self.cell_size, mount_radius_m=self.mount_radius_m,
        )[0]

    # ── mavros-хелперы ────────────────────────────────────────────────────────
    def _wait_service(self, cli, name: str) -> None:
        if not cli.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            raise RuntimeError(f"mavros сервис {name} не появился за {SERVICE_WAIT_S}s")

    def _call(self, cli, req, name: str, timeout_s: float = 10.0):
        """call_async + ожидание (фоновый executor обрабатывает ответ)."""
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout_s:
                raise RuntimeError(f"mavros {name} timeout {timeout_s}s")
            time.sleep(0.02)
        return fut.result()

    def _set_mode(self, mode: str) -> None:
        self._wait_service(self._mode_cli, "set_mode")
        t0 = time.monotonic()
        while self._state.mode != mode:
            if time.monotonic() - t0 > 15.0:
                raise RuntimeError(f"set_mode {mode}: текущий={self._state.mode}")
            req = SetMode.Request()
            req.custom_mode = mode
            self._call(self._mode_cli, req, f"set_mode({mode})")
            time.sleep(CMD_RETRY_S)
        if self.verbose:
            self._log.info(f"mode → {mode}")

    def _arm(self, value: bool) -> None:
        self._wait_service(self._arm_cli, "arming")
        t0 = time.monotonic()
        while self._state.armed != value:
            if time.monotonic() - t0 > 15.0:
                raise RuntimeError(f"arming={value}: armed={self._state.armed}")
            req = CommandBool.Request()
            req.value = value
            self._call(self._arm_cli, req, f"arm({value})")
            time.sleep(CMD_RETRY_S)
        if self.verbose:
            self._log.info(f"{'ARM' if value else 'DISARM'} ok")

    def _wait_odom(self, timeout_s: float = 15.0) -> None:
        t0 = time.monotonic()
        while not self.obs_builder.has_received_odom:
            if time.monotonic() - t0 > timeout_s:
                raise RuntimeError("нет odom от mavros (стек поднят? --mavros?)")
            time.sleep(0.1)

    # ── episode lifecycle ──────────────────────────────────────────────────────
    def start_episode(
        self, *, randomize_spawn: bool, rng: "np.random.Generator | None" = None
    ) -> dict[str, Any]:
        """Сброс эпизода.

        ⚠ SITL-урок (flight-smoke 2026-06-08): NAV_TAKEOFF ПОСЛЕ LAND-цикла молча
        не взлетает (climb timeout, armed но z≈0.21). Поэтому soft-reset НЕ садится:
        пока дрон здорово висит — только репозиция в воздухе (fly-to-spawn). Полный
        взлёт с земли — лишь на свежем стеке (первый эпизод / после hard_reset
        relaunch = чистый re-takeoff = очистка EKF-bias). Это совпадает с замыслом
        hard_reset_every: периодический ребут стека = единственный чистый re-takeoff.
        """
        if rng is not None:
            self._rng = rng
        self._wait_odom()

        healthy_airborne = (
            self._airborne and self._state.armed
            and self.obs_builder.pose.z_m > CRASH_Z_FRACTION * self.target_altitude
        )
        if healthy_airborne:
            # soft-reset: уже висим — стабилизируемся на текущей высоте, без land
            z_hold = self.obs_builder.pose.z_m
            pose = self.obs_builder.pose
            self.executor_act.initialize_target(
                pose.x_m, pose.y_m, z=z_hold, yaw=pose.heading_rad
            )
            self._hover_stabilize(z_hold)
        else:
            # полный взлёт с земли (свежий стек / crashed → нужен hard_reset для re-takeoff)
            z_hold = self._full_takeoff()

        # reposition к random free-XY (в воздухе; НЕ EKF-teleport)
        if randomize_spawn:
            self._reposition(z_hold)

        return self.read_state()

    def _full_takeoff(self) -> float:
        """Взлёт С ЗЕМЛИ: GUIDED→EKF-settle→arm→NAV_TAKEOFF→climb→hover. → z_hold."""
        if self._state.armed:
            # armed но не здоров (crashed/на земле) — чистим. ⚠ re-takeoff после
            # этого ненадёжен (SITL NAV_TAKEOFF-after-LAND); штатный путь = hard_reset.
            self._land_and_disarm()
        self._set_mode("GUIDED")
        if self.verbose:
            self._log.info(f"EKF-settle: ждём {self.ekf_settle_s:.0f}s до arm")
        time.sleep(self.ekf_settle_s)
        self._arm(True)
        # NAV_TAKEOFF + climb (БЕЗ setpoint-стрима — он перебивает NAV_TAKEOFF)
        self._takeoff_climb()
        self._airborne = True
        pose = self.obs_builder.pose
        z_hold = pose.z_m if pose.z_m > 0.5 else self.target_altitude
        self.executor_act.initialize_target(
            pose.x_m, pose.y_m, z=z_hold, yaw=pose.heading_rad
        )
        self._hover_stabilize(z_hold)
        return z_hold

    def _takeoff_climb(self) -> None:
        self._wait_service(self._takeoff_cli, "takeoff")
        req = CommandTOL.Request()
        req.altitude = float(self.target_altitude)
        self._call(self._takeoff_cli, req, "NAV_TAKEOFF")
        if self.verbose:
            self._log.info(f"NAV_TAKEOFF {self.target_altitude}m, climb…")
        t0 = time.monotonic()
        while True:
            z = self.obs_builder.pose.z_m
            if z >= CLIMB_ARRIVAL_M:
                if self.verbose:
                    self._log.info(f"climb done z={z:.2f}m")
                return
            if not self._state.armed:
                raise RuntimeError("disarmed во время climb")
            if time.monotonic() - t0 > CLIMB_TIMEOUT_S:
                self._log.warn(f"climb timeout z={z:.2f}m < {CLIMB_ARRIVAL_M} — продолжаю в hover")
                return
            time.sleep(0.1)

    def _hover_stabilize(self, target_z: float) -> None:
        """z в полосе ±band НЕПРЕРЫВНО hover_stabilize_s (executor стримит setpoint)."""
        t0 = time.monotonic()
        window_start = time.monotonic()
        while True:
            now = time.monotonic()
            z = self.obs_builder.pose.z_m
            if abs(z - target_z) > HOVER_Z_BAND_M:
                window_start = now      # вышли из полосы — сброс окна
            if now - window_start >= self.hover_stabilize_s:
                if self.verbose:
                    self._log.info(f"hover stable ≥{self.hover_stabilize_s:.0f}s (z={z:.2f}m)")
                return
            if now - t0 > HOVER_MAX_WAIT_S:
                self._log.warn(f"hover не стабилизировался за {HOVER_MAX_WAIT_S}s (z={z:.2f}m) — продолжаю")
                return
            time.sleep(0.1)

    def _reposition(self, z_hold: float) -> None:
        tx, ty = self._random_spawn_xy()
        yaw = float(self._rng.uniform(0.0, 2.0 * math.pi))
        if self.verbose:
            self._log.info(f"reposition → ({tx:.2f}, {ty:.2f}), yaw={math.degrees(yaw):.0f}°")
        self.executor_act.initialize_target(tx, ty, z=z_hold, yaw=yaw)
        # ждём прибытия (через всю комнату) — поллим позу
        t0 = time.monotonic()
        while time.monotonic() - t0 < REPOSITION_TIMEOUT_S:
            p = self.obs_builder.pose
            if math.hypot(p.x_m - tx, p.y_m - ty) < 0.15 and \
                    abs((p.heading_rad - yaw + math.pi) % (2 * math.pi) - math.pi) < math.radians(5):
                break
            time.sleep(0.05)
        self._hover_stabilize(z_hold)

    def _random_spawn_xy(self) -> tuple[float, float]:
        """Случайная free-позиция. free_mask → случайная FREE-клетка (с отступом
        от стен); иначе равномерно в пределах комнаты с margin."""
        if self.free_mask is not None:
            margin_cells = int(SPAWN_WALL_MARGIN_M / self.cell_size)
            ny, nx = self.free_mask.shape
            free = np.argwhere(self.free_mask)
            # отступ от краёв грида
            free = free[
                (free[:, 0] >= margin_cells) & (free[:, 0] < ny - margin_cells)
                & (free[:, 1] >= margin_cells) & (free[:, 1] < nx - margin_cells)
            ]
            if len(free) == 0:
                free = np.argwhere(self.free_mask)
            row, col = free[self._rng.integers(0, len(free))]
            return spawn_cell_to_m(int(col), int(row), self.cell_size, self.room_x, self.room_y)
        hx = self.room_x / 2.0 - SPAWN_WALL_MARGIN_M
        hy = self.room_y / 2.0 - SPAWN_WALL_MARGIN_M
        return float(self._rng.uniform(-hx, hx)), float(self._rng.uniform(-hy, hy))

    def _land_and_disarm(self) -> None:
        if self.verbose:
            self._log.info("land+disarm (старт нового эпизода)")
        try:
            self._set_mode("LAND")
        except RuntimeError as e:
            self._log.warn(f"set_mode LAND: {e} — форсирую disarm")
        t0 = time.monotonic()
        while self.obs_builder.pose.z_m > LAND_Z_M and self._state.armed:
            if time.monotonic() - t0 > LAND_TIMEOUT_S:
                self._log.warn("land timeout — форсирую disarm")
                break
            time.sleep(0.2)
        self._airborne = False
        try:
            self._arm(False)
        except RuntimeError:
            pass  # AP сам disarm'ит после land
        time.sleep(DISARM_GRACE_S)

    # ── Protocol API ────────────────────────────────────────────────────────────
    def read_state(self) -> dict[str, Any]:
        pose = self.obs_builder.pose
        distances = list(self.obs_builder.perimeter_distances_m) + [
            self.obs_builder.sweep_distance_m
        ]
        return {
            "pose_m": (pose.x_m, pose.y_m, pose.heading_rad),
            "distances": distances,                       # RAW sensor-frame [vl0..5, tf]
            "servo_deg": float(self.executor_act.servo_deg),
        }

    def execute(self, action: int) -> dict[str, Any]:
        action = int(action)
        if self._detect_crash():
            return {"travel_cells": 0, "collided": False, "crashed": True}

        # translation 0-3 в стену → отказ ДО полёта (защита + parity reward.blocked)
        perim = self.obs_builder.perimeter_distances_m
        if action in (0, 1, 2, 3) and gate_blocks(action, perim, self.gate_margin_m):
            return {"travel_cells": 0, "collided": True, "crashed": False}

        p0 = self.obs_builder.pose
        self.executor_act.execute(action, override_speed=self.linear_speed)
        p1 = self.obs_builder.pose
        travel = displacement_cells(p1.x_m - p0.x_m, p1.y_m - p0.y_m, self.cell_size)
        return {
            "travel_cells": travel,
            "collided": False,
            "crashed": self._detect_crash(),
        }

    def _detect_crash(self) -> bool:
        pose = self.obs_builder.pose
        if self._airborne and not self._state.armed:
            self._log.warn("crash: unexpected disarm в полёте")
            return True
        if self._tilt_rad > math.radians(CRASH_TILT_DEG):
            self._log.warn(f"crash: tilt {math.degrees(self._tilt_rad):.0f}° > {CRASH_TILT_DEG}°")
            return True
        if self._airborne and pose.z_m < CRASH_Z_FRACTION * self.target_altitude:
            self._log.warn(f"crash: z={pose.z_m:.2f}m просадка")
            return True
        if abs(pose.x_m) > self.room_x / 2.0 + SAFE_BOX_MARGIN_M or \
                abs(pose.y_m) > self.room_y / 2.0 + SAFE_BOX_MARGIN_M:
            self._log.warn(f"crash: вылет за safe-box ({pose.x_m:.2f}, {pose.y_m:.2f})")
            return True
        return False

    def hard_reset(self) -> None:
        if self.relaunch_cmd:
            self._log.warn(f"hard_reset: relaunch стека — {self.relaunch_cmd}")
            subprocess.run(self.relaunch_cmd, shell=True, check=False)
            self._airborne = False
            # дождаться переподключения mavros
            t0 = time.monotonic()
            while not self._state.connected and time.monotonic() - t0 < 60.0:
                time.sleep(0.5)
            self._wait_odom(timeout_s=60.0)
        else:
            self._log.warn("hard_reset без relaunch_cmd — soft land+disarm")
            if self._state.armed:
                self._land_and_disarm()

    def close(self) -> None:
        self._spin_stop.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        try:
            self._executor.remove_node(self.node)
        except Exception:  # noqa: BLE001
            pass
        self.node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
