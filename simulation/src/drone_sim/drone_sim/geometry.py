#!/usr/bin/env python3
"""geometry.py — загрузка drone_geometry.yaml + вычисление safety-полосы ФОРМУЛАМИ.

Единый источник порогов для v5-krot Этап-1 (контракт v2 §1). Пороги НЕ зашиты:
меняешь вход в drone_geometry.yaml → DroneGeometry пересчитывает всё.

Использование:
    from drone_sim.geometry import DroneGeometry
    g = DroneGeometry.load()              # ищет config/drone_geometry.yaml
    g.d_graze        # HARD no-graze floor (кадр луча)
    g.d_star_min     # мин. командуемый standoff
    g.beam_angles_rad  # [0, π/3, ...] раскладка лучей

RL читает тот же yaml (parity); этот класс — удобная sim-сторона + проверка формул.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

# config/drone_geometry.yaml относительно корня simulation/
_DEFAULT_YAML = (
    Path(__file__).resolve().parents[3] / "config" / "drone_geometry.yaml"
)


@dataclass
class DroneGeometry:
    # входные (физика)
    body_half: float
    motor_arm: float
    prop_radius: float
    sensor_ring_r: float
    sensor_angles_deg: list[float]
    tof_clip_m: float
    tof_min_m: float
    margin: float
    clear: float
    # управление
    v_max: float
    v_cruise: float
    control_hz: float
    d_star_target: float
    d_star_range: list[float]
    _src: str = field(default="", repr=False)

    # ── вычисляемое (формулы контракта §1) ──────────────────────────────
    @property
    def prop_tip(self) -> float:
        """Опасный радиус от ЦЕНТРА (кончик винта)."""
        return self.motor_arm + self.prop_radius

    @property
    def prop_tip_beam(self) -> float:
        """Тот же предел В КАДРЕ ЛУЧА (что читает VL53). Луч < этого → винт касается."""
        return self.prop_tip - self.sensor_ring_r

    @property
    def d_graze(self) -> float:
        """HARD no-graze floor (кадр луча)."""
        return self.prop_tip_beam + self.margin

    @property
    def d_star_min(self) -> float:
        """Мин. командуемый standoff (кадр луча)."""
        return self.d_graze + self.clear

    @property
    def beam_angles_rad(self) -> list[float]:
        return [math.radians(a) for a in self.sensor_angles_deg]

    @property
    def dt(self) -> float:
        return 1.0 / self.control_hz

    @classmethod
    def load(cls, path: str | Path | None = None) -> "DroneGeometry":
        p = Path(path) if path else _DEFAULT_YAML
        if not p.exists():
            raise FileNotFoundError(
                f"drone_geometry.yaml не найден: {p} — это БЛОКЕР чисел v5-krot"
            )
        g = yaml.safe_load(p.read_text())["geometry"]
        return cls(
            body_half=float(g["body_half"]),
            motor_arm=float(g["motor_arm"]),
            prop_radius=float(g["prop_radius"]),
            sensor_ring_r=float(g["sensor_ring_r"]),
            sensor_angles_deg=[float(a) for a in g["sensor_angles_deg"]],
            tof_clip_m=float(g["tof_clip_m"]),
            tof_min_m=float(g["tof_min_m"]),
            margin=float(g["margin"]),
            clear=float(g["clear"]),
            v_max=float(g["v_max"]),
            v_cruise=float(g["v_cruise"]),
            control_hz=float(g["control_hz"]),
            d_star_target=float(g["d_star_target"]),
            d_star_range=[float(x) for x in g["d_star_range"]],
            _src=str(p),
        )

    def summary(self) -> str:
        return (
            f"DroneGeometry({Path(self._src).name}): "
            f"prop_tip={self.prop_tip:.3f} prop_tip_beam={self.prop_tip_beam:.3f} "
            f"d_graze={self.d_graze:.3f} d_star_min={self.d_star_min:.3f} "
            f"target={self.d_star_target:.2f} v_max={self.v_max} @ {self.control_hz:.0f}Hz"
        )


if __name__ == "__main__":
    g = DroneGeometry.load()
    print(g.summary())
    print(f"  beam_angles_deg = {g.sensor_angles_deg}")
    print(f"  запас target→GRAZE = {g.d_star_target - g.d_graze:+.3f} м")
