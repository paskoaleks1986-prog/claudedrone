"""perimeter_sweep.py — детерминированная фаза облёта периметра ПОСЛЕ MISSION
COMPLETE (Стенд-спринт З3, Aleks 2026-06-08).

НЕ RL. Чистая геометрия: дрон знает габариты комнаты (room_x/y из worlds.yaml).
После того как модель завершила картирование (mapped ≥ threshold), bridge
переключается в PERIMETER_MODE и делает один чёткий прямоугольный облёт на
фиксированном standoff от стен, со snap'ом курса к осям (90°), затем
возвращается в hover центра. Цель — на видео виден аккуратный прямоугольник
после хаотичного mapping.

Алгоритм (ТЗ Aleks):
    1. Найти ближайшую стену/угол (по позе — геометрия комнаты известна).
    2. Повернуться параллельно стене (snap к ближайшим 90°).
    3. Лететь вдоль стены на standoff до следующего угла.
    4. Повернуть на 90° (snap).
    5. Повторять до полного обхода (4 стены).
    6. Вернуться в hover центра.

Прямоугольник: углы на inset = room/2 − wall_inset − standoff от центра.
Путь corner→corner — straight position-setpoint (как wall_follow), поэтому
траектория = чистый прямоугольник независимо от yaw; yaw snap'ается для
ориентации sweep-дальномера. Только rect-миры (4 стены): для L/apartment
вызывающий не включает фазу (или она деградирует — см. dev-log 30).

Контракт: вызывающий (нода) гонит current() → летит к таргету → по прибытии
advance(). complete=True → нода уводит дрон в центр-hover.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

HALF_PI = math.pi / 2.0


def snap_yaw_90(rad: float) -> float:
    """Snap угла к ближайшему кратному 90° в диапазоне (−π, π]."""
    k = round(rad / HALF_PI)
    snapped = k * HALF_PI
    # нормализуем в (−π, π]
    while snapped > math.pi:
        snapped -= 2 * math.pi
    while snapped <= -math.pi:
        snapped += 2 * math.pi
    return snapped


@dataclass(frozen=True)
class PerimeterWaypoint:
    x: float
    y: float
    yaw: float
    kind: str        # "rotate" | "translate" | "to_center"
    tag: str
    leg: int         # 0..3 индекс стены; -1 = возврат в центр


class PerimeterSweep:
    """План облёта периметра прямоугольной комнаты.

    room_x/room_y — внешний bbox (worlds.yaml). standoff_m — зазор до ВНУТРЕННЕЙ
    грани стены. wall_inset_m — половина+ толщины стены (внутр.грань = room/2 −
    wall_inset_m). corner inset от центра = room/2 − wall_inset_m − standoff_m.
    """

    def __init__(
        self,
        room_x_m: float,
        room_y_m: float,
        standoff_m: float = 0.6,
        wall_inset_m: float = 0.1,
    ) -> None:
        self.room_x_m = float(room_x_m)
        self.room_y_m = float(room_y_m)
        self.standoff_m = float(standoff_m)
        self.ax = room_x_m / 2.0 - wall_inset_m - standoff_m
        self.ay = room_y_m / 2.0 - wall_inset_m - standoff_m
        # CCW от нижне-левого: BL, BR, TR, TL.
        self._corners = [
            (-self.ax, -self.ay),
            (self.ax, -self.ay),
            (self.ax, self.ay),
            (-self.ax, self.ay),
        ]
        self._wps: list[PerimeterWaypoint] = []
        self._idx = 0
        self._planned = False

    @property
    def feasible(self) -> bool:
        """Комната достаточно велика, чтобы облёт имел смысл (inset > 0)."""
        return self.ax > 0.05 and self.ay > 0.05

    def plan(self, pose_x: float, pose_y: float) -> None:
        """Построить waypoints, начиная с ближайшего к дрону угла, CCW,
        полный обход (назад к старт-углу), затем центр."""
        if not self.feasible:
            self._planned = True
            self._wps = []
            return
        # ближайший угол к текущей позе = старт
        start = min(
            range(4),
            key=lambda i: (self._corners[i][0] - pose_x) ** 2
            + (self._corners[i][1] - pose_y) ** 2,
        )
        order = [(start + k) % 4 for k in range(4)] + [start]  # 5 точек, 4 ноги
        wps: list[PerimeterWaypoint] = []

        # --- начальный подлёт к старт-углу ---
        sx, sy = self._corners[order[0]]
        approach_yaw = snap_yaw_90(math.atan2(sy - pose_y, sx - pose_x))
        wps.append(PerimeterWaypoint(pose_x, pose_y, approach_yaw, "rotate",
                                     "approach-rotate", order[0]))
        wps.append(PerimeterWaypoint(sx, sy, approach_yaw, "translate",
                                     "approach", order[0]))

        # --- 4 ноги вдоль стен ---
        for leg in range(4):
            ax_, ay_ = self._corners[order[leg]]
            bx_, by_ = self._corners[order[leg + 1]]
            leg_yaw = snap_yaw_90(math.atan2(by_ - ay_, bx_ - ax_))
            wps.append(PerimeterWaypoint(ax_, ay_, leg_yaw, "rotate",
                                         f"corner{leg}-turn", order[leg]))
            wps.append(PerimeterWaypoint(bx_, by_, leg_yaw, "translate",
                                         f"wall{leg}", order[leg]))

        # --- возврат в центр ---
        wps.append(PerimeterWaypoint(0.0, 0.0, 0.0, "to_center",
                                     "to-center", -1))
        self._wps = wps
        self._idx = 0
        self._planned = True

    @property
    def planned(self) -> bool:
        return self._planned

    @property
    def complete(self) -> bool:
        return self._planned and self._idx >= len(self._wps)

    @property
    def total_waypoints(self) -> int:
        return len(self._wps)

    @property
    def index(self) -> int:
        return self._idx

    def current(self) -> PerimeterWaypoint | None:
        if not self._planned or self._idx >= len(self._wps):
            return None
        return self._wps[self._idx]

    def advance(self) -> None:
        if self._idx < len(self._wps):
            self._idx += 1
