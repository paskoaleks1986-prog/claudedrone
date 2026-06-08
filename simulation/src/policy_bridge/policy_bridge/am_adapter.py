"""ActiveMappingAdapter — obs Box(21,) + action_masks для ActiveMapping-v1.

Канон: $WS_DIR/bridge_map_protocol.md v1.0 + export obs_spec.md (FINAL).
Паритет map-математики доказан: фикстуры 5/5 (test_occupancy_map.py) +
AM-4 joint replay 44/44 bit-exact (test_parity_replay.py).

Обязанности:
    - occupancy 64×64 через OccupancyMapBuilder (карта эпизода);
    - интеграция лучей ПО ТРАНЗАКЦИЯМ (§2.5): step boundary / новая клетка
      во время action 7 и translations / первичный взгляд на /takeoff/ready;
    - obs-вектор Box(21,) по §5 (нормализации, heading wrap [0,2π));
    - action_masks() по F1 (из occupancy, НЕ ground truth) — MaskablePPO
      ОБЯЗАН получать его в predict.

Без ROS-импортов — все внешние данные через callables (как ActionExecutor),
юнит-тестируется без rclpy.
"""
from __future__ import annotations

import math
from typing import Callable

import numpy as np

from policy_bridge.occupancy_map_builder import (
    GRID,
    OccupancyMapBuilder,
    free_run_cells,
)

OBS_DIM = 21
# §2.4: VL53 сидят на радиусе 0.1 м — для карты/obs центр-референсим.
VL_MOUNT_RADIUS_M = 0.1
VL_MAX_M = 1.2
TF_MAX_M = 6.4
TWO_PI = 2.0 * math.pi


def build_obs_vector(
    *,
    x_cells: float,
    y_cells: float,
    heading_rad: float,
    vl_raw_m: list[float],          # 6 каналов, sensor-frame (БЕЗ mount)
    tf_raw_m: float,                # sweep, метры (inf уже capped)
    servo_deg: float,
    frontier_directions: np.ndarray,  # [8] из obs_fields
    frontier_count: float,            # из obs_fields (норм. /4096)
    mapped_ratio: float,              # из obs_fields
) -> np.ndarray:
    """Чистая сборка obs Box(21,) по §5 (ROS2-free). Единый источник для
    ActiveMappingAdapter.snapshot И InferenceCore.build_obs — paritetы
    (test_parity_replay) сторожат бит-точность. vl центр-референсим (+mount)
    как в integrate; heading wrap [0,2π); clip [0,1] (поза вне грида)."""
    obs = np.empty(OBS_DIM, dtype=np.float32)
    vl_centered = [d + VL_MOUNT_RADIUS_M for d in vl_raw_m]
    for i, d in enumerate(vl_centered):
        obs[i] = min(max(d, 0.0), VL_MAX_M) / VL_MAX_M
    obs[6] = min(max(tf_raw_m, 0.0), TF_MAX_M) / TF_MAX_M
    obs[7] = (servo_deg % 180.0) / 180.0
    obs[8] = x_cells / GRID
    obs[9] = y_cells / GRID
    obs[10] = (heading_rad % TWO_PI) / TWO_PI
    obs[11:19] = frontier_directions
    obs[19] = frontier_count
    obs[20] = mapped_ratio
    np.clip(obs, 0.0, 1.0, out=obs)
    return obs


class ActiveMappingAdapter:
    """Связка OccupancyMapBuilder ↔ bridge-нода для модельной семьи AM-v1."""

    def __init__(
        self,
        *,
        room_size_m: float,
        cell_size_m: float,
        free_mask: np.ndarray,
        get_pose: Callable[[], object],          # Pose2D (x_m, y_m, heading_rad)
        get_vl_raw_m: Callable[[], list[float]],  # 6 каналов, sensor-frame
        get_tf_raw_m: Callable[[], float],        # sweep, метры (inf уже capped)
        get_servo_deg: Callable[[], float],       # commanded angle executor'а
        room_y_m: float | None = None,            # Блок Б: rect-миры; None = квадрат
        min_frontier_cluster_cells: int = 1,      # §3.1: 1=parity, 3=aligned
    ) -> None:
        # Протокол §1: модельный obs-контракт = 64×64 @ 0.1 м. В мирах
        # больше карта строится rect-гридом по миру (Блок Б), но нормализации
        # obs (/64, /4096) НЕ растягиваются → model_canon=False (OOD; caller
        # обязан громко предупредить).
        if free_mask is None:
            raise ValueError(
                "ActiveMapping требует free_mask (Option A, §5.1) — "
                "mapped_ratio без него не определён. Проверь free_mask_path."
            )
        room_x_m = float(room_size_m)
        room_y_m = float(room_y_m) if room_y_m is not None else room_x_m
        nx = int(round(room_x_m / cell_size_m))
        ny = int(round(room_y_m / cell_size_m))
        if free_mask.shape != (ny, nx):
            raise ValueError(
                f"free_mask shape {free_mask.shape} != грид мира ({ny}, {nx})"
            )
        self.model_canon = (nx == GRID and ny == GRID
                            and abs(cell_size_m - 0.1) < 1e-9)
        self.cell = cell_size_m
        self.half_x = room_x_m / 2.0
        self.half_y = room_y_m / 2.0
        self.free_mask = free_mask.astype(bool)
        self._get_pose = get_pose
        self._get_vl_raw_m = get_vl_raw_m
        self._get_tf_raw_m = get_tf_raw_m
        self._get_servo_deg = get_servo_deg

        self.builder = OccupancyMapBuilder(
            nx=nx, ny=ny,
            min_frontier_cluster_cells=min_frontier_cluster_cells,
        )
        self._last_cell: tuple[int, int] | None = None
        self.integrations = 0

    # ---- координаты (§1) ----------------------------------------------------

    def _to_cells(self, x_m: float, y_m: float) -> tuple[float, float]:
        return (x_m + self.half_x) / self.cell, (y_m + self.half_y) / self.cell

    # ---- интеграция (§2.5) ---------------------------------------------------

    def reset_episode(self) -> None:
        """§2.5 п.3: /takeoff/ready — клетка спавна FREE + первичный взгляд."""
        pose = self._get_pose()
        xc, yc = self._to_cells(pose.x_m, pose.y_m)
        self.builder.reset(xc, yc)
        self._last_cell = None
        self.integrations = 0
        self.integrate_now()

    def integrate_now(self) -> None:
        """7 лучей из текущей позы (§2.5 п.1: step boundary / снапшот obs)."""
        pose = self._get_pose()
        xc, yc = self._to_cells(pose.x_m, pose.y_m)
        vl_centered = [d + VL_MOUNT_RADIUS_M for d in self._get_vl_raw_m()]
        self.builder.integrate(
            xc, yc, pose.heading_rad,
            vl_centered, self._get_servo_deg(), self._get_tf_raw_m(),
            cell_size_m=self.cell,
        )
        self._last_cell = (int(xc), int(yc))
        self.integrations += 1

    def free_run_ch0(self) -> int:
        """§3.2 v2 (STUB): free_run целых FREE-клеток вперёд (ch0 heading) по
        occupancy. Для action7-маски при v2_sensor_mask. ⚠ геометрию
        финализировать против v2 parity-фикстур."""
        pose = self._get_pose()
        xc, yc = self._to_cells(pose.x_m, pose.y_m)
        return free_run_cells(self.builder.occ, xc, yc, pose.heading_rad)

    def on_pose_update(self, x_m: float, y_m: float) -> None:
        """§2.5 п.2: хук visited_update_fn (20 Hz poll'ы arrival-ожидания) —
        интегрируем только при входе в НОВУЮ клетку (паритет: env per cell,
        не per tick)."""
        xc, yc = self._to_cells(x_m, y_m)
        cell = (int(xc), int(yc))
        if cell != self._last_cell:
            self.integrate_now()

    # ---- obs / mask (§5, F1) --------------------------------------------------

    def snapshot(self) -> tuple[np.ndarray, np.ndarray, float]:
        """(obs Box(21,), action_mask bool(8,), mapped_ratio) одним расчётом
        BFS/frontier. Лучи НЕ интегрирует — вызывающий обязан сделать
        integrate_now() на step boundary до снапшота (§2.5 п.1)."""
        pose = self._get_pose()
        xc, yc = self._to_cells(pose.x_m, pose.y_m)
        heading = pose.heading_rad % TWO_PI          # §5: wrap [0, 2π)

        fields = self.builder.obs_fields(xc, yc, heading, self.free_mask)
        obs = build_obs_vector(
            x_cells=xc, y_cells=yc, heading_rad=pose.heading_rad,
            vl_raw_m=self._get_vl_raw_m(), tf_raw_m=self._get_tf_raw_m(),
            servo_deg=self._get_servo_deg(),
            frontier_directions=fields["frontier_directions"],
            frontier_count=fields["frontier_count"],
            mapped_ratio=fields["mapped_ratio"],
        )
        return obs, fields["action_mask"], float(fields["mapped_ratio"])
