"""OccupancyMapBuilder — occupancy 64×64 + frontier + BFS для ActiveMapping-v1.

Канон: $WS_DIR/bridge_map_protocol.md v1.0 (§2 occupancy, §3 frontier,
§4 BFS frontier_directions, §5.1 mapped_ratio). Численный паритет с
rl-lab DroneMapEnv проверяется фикстурами
$RL_LAB_ROOT/export/activemapping_v1/fixtures/*.json (atol 1e-6) в
test/test_occupancy_map.py.

Чистые numpy/deque функции без ROS — ROS-обвязка придёт в bridge-ноду
при интеграции v1.5 модели.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
from scipy import ndimage

GRID = 64
UNKNOWN, FREE, OCCUPIED = 0, 1, 2

RAY_STEP = 0.5            # клетки; sensors.py:17 / протокол §2.2
MAX_VL_RANGE_CELLS = 12.0   # 1.2 м / 0.1
MAX_TF_RANGE_CELLS = 64.0   # 6.4 м / 0.1
VL_OFFSETS_DEG = (0.0, 60.0, 120.0, 180.0, 240.0, 300.0)

N_SECTORS = 8
SECTOR_DEG = 45.0

# §4.2: порядок обхода N, S, W, E (канон rl-lab D2), в ЯВНОЙ (dx, dy)-нотации
# world frame (+y = North при SW-origin). Канон зафиксирован углами.
NEIGH = ((0, 1), (0, -1), (-1, 0), (1, 0))      # N, S, W, E — (dx, dy)
NEIGH_ANGLE_WORLD_DEG = (90.0, 270.0, 180.0, 0.0)


def integrate_ray(
    occ: np.ndarray,
    x_cells: float,
    y_cells: float,
    angle_world_rad: float,
    reading_cells: float,
    max_range_cells: float,
) -> None:
    """Протокол §2.2: bridge-вариант env._integrate_ray.

    Семплирование RAY_STEP=0.5 (НЕ Bresenham). Семплы до reading → FREE,
    клетка хита → OCCUPIED только если reading < max_range (§2.3: inf /
    saturated = «no hit» — FREE-полоса без OCCUPIED).
    """
    hit = reading_cells < (max_range_cells - 1e-6)
    dx = math.cos(angle_world_rad)
    dy = math.sin(angle_world_rad)
    span = min(reading_cells, max_range_cells)
    h, w = occ.shape  # Блок Б: границы из формы карты (rect-миры)
    d = 0.0
    while d < span:
        cx = int(x_cells + dx * d)
        cy = int(y_cells + dy * d)
        if not (0 <= cx < w and 0 <= cy < h):
            return
        occ[cy, cx] = FREE
        d += RAY_STEP
    if hit:
        cx = int(x_cells + dx * reading_cells)
        cy = int(y_cells + dy * reading_cells)
        if 0 <= cx < w and 0 <= cy < h:
            occ[cy, cx] = OCCUPIED


def integrate_pose(
    occ: np.ndarray,
    x_cells: float,
    y_cells: float,
    heading_rad: float,
    vl_readings_cells: list[float],
    servo_deg: float,
    tf_reading_cells: float,
) -> None:
    """Протокол §2.5: все 7 лучей из текущей позы.

    vl_readings_cells — 6 центр-референсных чтений (§2.4: raw_m + 0.1, clip
    1.2, в клетках) в порядке каналов 0..5; tf — sweep в клетках.
    """
    for ch, off_deg in enumerate(VL_OFFSETS_DEG):
        integrate_ray(
            occ, x_cells, y_cells,
            heading_rad + math.radians(off_deg),
            vl_readings_cells[ch], MAX_VL_RANGE_CELLS,
        )
    tf_angle = heading_rad + math.radians(servo_deg - 90.0)
    integrate_ray(occ, x_cells, y_cells, tf_angle,
                  tf_reading_cells, MAX_TF_RANGE_CELLS)


def update_frontiers(occ: np.ndarray) -> np.ndarray:
    """Протокол §3 = env._update_frontiers 1:1 (4-связность, граница ≠ UNKNOWN)."""
    free = occ == FREE
    unknown = occ == UNKNOWN
    adj_unknown = np.zeros_like(unknown)
    adj_unknown[1:, :] |= unknown[:-1, :]
    adj_unknown[:-1, :] |= unknown[1:, :]
    adj_unknown[:, 1:] |= unknown[:, :-1]
    adj_unknown[:, :-1] |= unknown[:, 1:]
    return free & adj_unknown


# §3.1 (Alignment Sprint, Aleks 2026-06-07): фильтр мелких frontier-кластеров.
# Изолированный кластер < MIN клеток (< 0.3м) = угловой артефакт, не реальное
# неизведанное — рейкаст его закроет. Ref Yamauchi 1997 + практика.
# ⚠ ПАРИТЕТ: меняет frontier-маску (она в obs). min=1 → НИ-ОДНОГО-ОП,
# bit-exact с историческим env (v1.5c обучена без фильтра, AM-4 фикстуры
# тоже). Aligned-build = 3 на ОБЕИХ сторонах (env + bridge) после retrain.
MIN_FRONTIER_CLUSTER_CELLS_DEFAULT = 1   # off = parity-safe; aligned = 3
_FRONTIER_4CONN = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)


def filter_small_frontiers(mask: np.ndarray, min_cells: int) -> np.ndarray:
    """Убрать 4-связные frontier-кластеры размером < min_cells.

    min_cells ≤ 1 → возврат без изменений (parity-safe no-op). Кластеризация
    через connected components (4-связность) — детерминированный результат,
    эквивалент BFS из §3.1 протокола."""
    if min_cells <= 1 or not mask.any():
        return mask
    labels, n = ndimage.label(mask, structure=_FRONTIER_4CONN)
    if n == 0:
        return mask
    sizes = np.bincount(labels.ravel())
    keep = np.isin(labels, np.flatnonzero(sizes >= min_cells))
    return mask & keep


def frontier_bfs(
    occ_free: np.ndarray, ix0: int, iy0: int
) -> tuple[np.ndarray, np.ndarray]:
    """Протокол §4.2: один BFS от клетки дрона с метками первого шага.

    Возвращает (dist, first): dist int32 (-1 = недостижимо), first int8
    (индекс NEIGH первого шага кратчайшего пути; -1 у источника).
    Tie-breaking детерминирован: FIFO + порядок NEIGH.
    Клетка дрона принудительно проходима (§4.1).
    """
    h, w = occ_free.shape
    dist = np.full((h, w), -1, np.int32)
    first = np.full((h, w), -1, np.int8)
    dist[iy0, ix0] = 0
    q: deque[tuple[int, int]] = deque([(ix0, iy0)])
    while q:
        x, y = q.popleft()
        for k, (dx, dy) in enumerate(NEIGH):
            nx, ny = x + dx, y + dy
            if (
                0 <= nx < w and 0 <= ny < h
                and occ_free[ny, nx] and dist[ny, nx] < 0
            ):
                dist[ny, nx] = dist[y, x] + 1
                first[ny, nx] = k if dist[y, x] == 0 else first[y, x]
                q.append((nx, ny))
    return dist, first


def frontier_directions(
    occ: np.ndarray,
    frontier_mask: np.ndarray,
    x_cells: float,
    y_cells: float,
    heading_rad: float,
) -> np.ndarray:
    """Протокол §4.3: 8 ego-секторов, сектор = угол ПЕРВОГО BFS-шага,
    value = 1/(bfs_dist+1). Недостижимые frontier'ы = 0."""
    out = np.zeros(N_SECTORS, dtype=np.float32)
    h, w = occ.shape
    ix0, iy0 = int(x_cells), int(y_cells)
    if not (0 <= ix0 < w and 0 <= iy0 < h):
        return out
    occ_free = occ == FREE
    occ_free[iy0, ix0] = True  # §4.1: источник принудительно проходим
    dist, first = frontier_bfs(occ_free, ix0, iy0)

    best: dict[int, int] = {}
    heading_deg = math.degrees(heading_rad)
    for fy, fx in np.argwhere(frontier_mask):
        d = int(dist[fy, fx])
        if d <= 0:  # недостижим (-1) или дрон стоит на frontier (0)
            continue
        ang_world = NEIGH_ANGLE_WORLD_DEG[first[fy, fx]]
        rel = (ang_world - heading_deg) % 360.0
        s = int((rel + SECTOR_DEG / 2) // SECTOR_DEG) % N_SECTORS
        if s not in best or d < best[s]:
            best[s] = d
    for s, d in best.items():
        out[s] = 1.0 / (float(d) + 1.0)
    return out


def mapped_ratio(occ: np.ndarray, free_mask: np.ndarray) -> float:
    """Протокол §5.1 (согласовано E3): (known ∩ free_mask) / free_count."""
    free_count = int(free_mask.sum())
    if free_count <= 0:
        return 0.0
    known_free = int(((occ != UNKNOWN) & free_mask).sum())
    return known_free / free_count


def action_mask_from_occupancy(
    occ: np.ndarray, x_cells: float, y_cells: float, heading_rad: float
) -> np.ndarray:
    """Протокол F1: mask из occupancy (НЕ из ActionGate-клиренсов).

    0-3: целевая клетка в 1 cell по body-направлению == FREE на occupancy;
    7 = mask[0]; 4/5/6 всегда True. Семантика env.action_masks(), но по
    известной карте вместо ground truth.
    """
    mask = np.ones(8, dtype=bool)
    h, w = occ.shape
    for a, body_ang in enumerate((0.0, math.pi, math.pi / 2, -math.pi / 2)):
        ang = heading_rad + body_ang
        cx = int(x_cells + math.cos(ang))
        cy = int(y_cells + math.sin(ang))
        mask[a] = (
            0 <= cx < w and 0 <= cy < h and occ[cy, cx] == FREE
        )
    mask[7] = mask[0]
    return mask


class OccupancyMapBuilder:
    """Stateful обёртка для bridge-ноды: §2.1 глобальная карта на эпизод.

    Блок Б (2026-06-07): размер карты параметрен (rect-миры, indoor_room
    160×100). Дефолт 64×64 — модельный канон; нормализации obs_fields
    (frontier_count /4096) остаются модельным контрактом НЕЗАВИСИМО от
    фактического размера (в больших мирах модель OOD — ответственность
    вызывающего предупредить).
    """

    def __init__(
        self, *, nx: int = GRID, ny: int = GRID,
        min_frontier_cluster_cells: int = MIN_FRONTIER_CLUSTER_CELLS_DEFAULT,
    ) -> None:
        self.nx, self.ny = int(nx), int(ny)
        # §3.1: 1 = parity-safe (v1.5c, фикстуры); 3 = aligned-build (retrain)
        self.min_frontier_cluster_cells = int(min_frontier_cluster_cells)
        self.occ = np.zeros((self.ny, self.nx), dtype=np.uint8)
        self._frontier_mask = np.zeros((self.ny, self.nx), dtype=bool)

    def reset(self, x_cells: float, y_cells: float) -> None:
        """§2.1: всё UNKNOWN, клетка спавна FREE (первый integrate — снаружи)."""
        self.occ.fill(UNKNOWN)
        ix, iy = int(x_cells), int(y_cells)
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            self.occ[iy, ix] = FREE

    def integrate(
        self,
        x_cells: float,
        y_cells: float,
        heading_rad: float,
        vl_centered_m: list[float],
        servo_deg: float,
        tf_m: float,
        cell_size_m: float = 0.1,
    ) -> None:
        """Перевод метров (центр-референсных §2.4) в клетки + интеграция."""
        vl_cells = [
            min(max(d, 0.0), 1.2) / cell_size_m for d in vl_centered_m
        ]
        tf_cells = min(max(tf_m, 0.0), 6.4) / cell_size_m
        integrate_pose(
            self.occ, x_cells, y_cells, heading_rad,
            vl_cells, servo_deg, tf_cells,
        )
        # §3.1: фильтр мелких кластеров (no-op при min ≤ 1 → bit-exact)
        self._frontier_mask = filter_small_frontiers(
            update_frontiers(self.occ), self.min_frontier_cluster_cells
        )

    @property
    def frontier_mask(self) -> np.ndarray:
        return self._frontier_mask

    def obs_fields(
        self,
        x_cells: float,
        y_cells: float,
        heading_rad: float,
        free_mask: np.ndarray,
    ) -> dict[str, float | np.ndarray]:
        """frontier_directions[8], frontier_count, mapped_ratio, action_mask."""
        return {
            "frontier_directions": frontier_directions(
                self.occ, self._frontier_mask, x_cells, y_cells, heading_rad
            ),
            "frontier_count": float(self._frontier_mask.sum()) / (GRID * GRID),
            "mapped_ratio": mapped_ratio(self.occ, free_mask),
            "action_mask": action_mask_from_occupancy(
                self.occ, x_cells, y_cells, heading_rad
            ),
        }
