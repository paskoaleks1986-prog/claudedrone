"""inference_core — ROS2-free ядро инференса ActiveMapping-v1/v2 (Phase 0).

SITL-RL спринт (Aleks/rl-lab 2026-06-08). Выносит из policy_bridge_node всю
модельную математику в либу БЕЗ rclpy, чтобы rl-lab импортировал её в
SITLDroneEnv (общий inference-core → parity train↔sim by construction).

`import policy_bridge.inference_core` НЕ тянет rclpy (пакетный __init__ пуст;
зависимости — occupancy_map_builder / am_adapter / action_gate, все ROS2-free,
+ sb3/sb3_contrib для загрузки модели). ROS2-нода остаётся тонкой обёрткой:
читает топики → вызывает InferenceCore.

Контракт (rl-lab):
    core = InferenceCore(model_path="model.zip")
    obs  = core.build_obs(pose, distances, servo_angle, occ_map, free_mask)
    mask = core.build_mask(true_grid, pose, n_cells=6)
    action = core.predict(obs, mask)

⚠ ДВА РАЗНЫХ ГРИДА (см. сигнатуры):
  - build_obs.occ_map  = occupancy АГЕНТА (UNKNOWN=0/FREE=1/OCCUPIED=2),
    накопленная по лучам; для frontier/mapped_ratio.
  - build_mask.true_grid = ground-truth/сенсорная карта свободы (0=free,
    !=0=стена) — вход env._free_run(occ=False). v2 sensor-mask.

Pose — клетки (occupancy-frame): Pose(x_cells, y_cells, heading_rad).
distances — [vl0..vl5, tf] метры RAW (mount-radius добавляется внутри как в
integrate). servo_angle — градусы (commanded). Все нормализации — §5 протокола.

Паритет: build_obs/build_mask переиспользуют те же функции, что и фикстуры
(test_parity_replay*, test_inference_core 44/44 bit-exact).
"""
from __future__ import annotations

from collections import namedtuple
from pathlib import Path

import numpy as np

from policy_bridge.action_gate import sensor_action_mask, sensor_free_runs
from policy_bridge.am_adapter import OBS_DIM, build_obs_vector
from policy_bridge.occupancy_map_builder import (
    GRID,
    action_mask_from_occupancy,
    filter_small_frontiers,
    frontier_directions,
    mapped_ratio,
    update_frontiers,
)

# Поза в клетках occupancy-frame (НЕ метры; конверсию m→cells делает caller/
# нода: (x_m + room/2)/cell). namedtuple — без зависимостей.
Pose = namedtuple("Pose", ["x_cells", "y_cells", "heading_rad"])


class InferenceCore:
    """ROS2-free инференс AM-v1/v2: загрузка модели + obs/mask/predict.

    Parameters
    ----------
    model_path : путь к SB3 model.zip.
    family : "activemapping" (MaskablePPO, обязателен action_masks в predict)
             | "sweep02" (legacy PPO Dict obs — predict без маски).
    deterministic : predict(deterministic=...). AM-эталоны сняты True.
    min_frontier_cluster_cells : §3.1 obs-frontier фильтр (1=v1.5c/N6-v1,
             3=aligned v2). ДОЛЖЕН совпадать с тренировкой модели.
    wall_stop_cells : N для v2 sensor-mask (free_cells > N). = train.
    device : torch device ("cpu").
    """

    def __init__(
        self,
        model_path: str | Path,
        *,
        family: str = "activemapping",
        deterministic: bool = True,
        min_frontier_cluster_cells: int = 3,
        wall_stop_cells: int = 6,
        device: str = "cpu",
    ) -> None:
        self.family = family
        self.deterministic = bool(deterministic)
        self.min_frontier_cluster_cells = int(min_frontier_cluster_cells)
        self.wall_stop_cells = int(wall_stop_cells)
        self.model = self._load_model(str(model_path), family, device)

    @staticmethod
    def _load_model(path: str, family: str, device: str):
        if family == "activemapping":
            from sb3_contrib import MaskablePPO
            return MaskablePPO.load(path, device=device)
        if family == "sweep02":
            from stable_baselines3 import PPO
            return PPO.load(path, device=device)
        raise ValueError(f"family={family!r} — activemapping|sweep02")

    # ---- obs (§5) -----------------------------------------------------------

    def build_obs(
        self,
        pose: Pose,
        distances,                      # [vl0..vl5, tf] метры RAW (7)
        servo_angle: float,             # градусы (commanded)
        occ_map: np.ndarray,            # occupancy АГЕНТА (UNKNOWN/FREE/OCC)
        free_mask: np.ndarray,          # bool (для mapped_ratio §5.1)
    ) -> np.ndarray:
        """Box(21,) obs из raw-данных + occupancy агента. Frontier-поля
        пересчитываются из occ_map (update_frontiers + §3.1 фильтр), как в
        OccupancyMapBuilder.obs_fields → bit-exact с тренировкой."""
        if len(distances) < 7:
            raise ValueError(f"distances ожидает [vl0..5, tf] (7), дано {len(distances)}")
        fmask = filter_small_frontiers(
            update_frontiers(occ_map), self.min_frontier_cluster_cells
        )
        fdirs = frontier_directions(
            occ_map, fmask, pose.x_cells, pose.y_cells, pose.heading_rad
        )
        fcount = float(fmask.sum()) / (GRID * GRID)
        mapped = mapped_ratio(occ_map, free_mask.astype(bool))
        return build_obs_vector(
            x_cells=pose.x_cells, y_cells=pose.y_cells,
            heading_rad=pose.heading_rad,
            vl_raw_m=list(distances[:6]), tf_raw_m=float(distances[6]),
            servo_deg=float(servo_angle),
            frontier_directions=fdirs, frontier_count=fcount,
            mapped_ratio=mapped,
        )

    # ---- mask (F1 / §3.2 v2) -------------------------------------------------

    def build_mask(
        self,
        grid: np.ndarray,
        pose: Pose,
        n_cells: int | None = None,
        *,
        mode: str = "sensor",
    ) -> np.ndarray:
        """action_mask bool(8,).

        mode="sensor" (v2-prod, default): free_runs по 6 каналам через
            sensor_free_runs(grid, ...) (env._free_run, occ=False, !=0=стена)
            → sensor_action_mask. grid = ground-truth/сенсорная карта свободы.
        mode="occupancy" (F1): action_mask_from_occupancy(grid, ...) по
            occupancy агента (FREE=1). grid = occ_map.
        """
        n = self.wall_stop_cells if n_cells is None else int(n_cells)
        if mode == "sensor":
            runs = sensor_free_runs(
                grid, pose.x_cells, pose.y_cells, pose.heading_rad,
                grid_size=grid.shape[0],
            )
            return np.array(sensor_action_mask(runs, n), dtype=bool)
        if mode == "occupancy":
            return action_mask_from_occupancy(
                grid, pose.x_cells, pose.y_cells, pose.heading_rad
            )
        raise ValueError(f"mode={mode!r} — sensor|occupancy")

    # ---- predict ------------------------------------------------------------

    def predict(self, obs: np.ndarray, mask: np.ndarray | None = None) -> int:
        """action int. AM: predict(obs, action_masks=mask) — маска ОБЯЗАТЕЛЬНА
        (F1, иначе off-distribution). sweep02: без маски."""
        obs = np.asarray(obs, dtype=np.float32)
        if self.family == "activemapping":
            if mask is None:
                raise ValueError("activemapping requires action_masks (F1)")
            a, _ = self.model.predict(
                obs, action_masks=np.asarray(mask, dtype=bool),
                deterministic=self.deterministic,
            )
        else:
            a, _ = self.model.predict(obs, deterministic=self.deterministic)
        return int(np.asarray(a).reshape(-1)[0])
