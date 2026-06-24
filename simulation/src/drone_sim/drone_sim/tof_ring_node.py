#!/usr/bin/env python3
"""tof_ring_node — gz VL53×6 (per-канал LaserScan) → канонический ToF-массив для obs.

v5-krot Этап-1. Sim-сторона obs-фида: собирает 6 лучей gz в ЕДИНЫЙ массив в
КАНОН-ПОРЯДКЕ контракта (index0=+x нос, CCW по 60°: [0,60,120,180,240,300]°),
клипит на tof_clip_m. RL читает этот массив → строит obs[0:6] tof + obs[6:12] hit.

ВХОД (gz, sensor QoS BEST_EFFORT):
    /drone/vl53l0x/ch{0..5}   sensor_msgs/LaserScan (ranges[0])
ВЫХОД (proposal-имя, согласовать с interface):
    /krot/tof_ring            std_msgs/Float32MultiArray, 6 float, МЕТРЫ, клип [tof_min, tof_clip]
                              no-hit (inf / > clip) → tof_clip (sentinel «чисто до предела»)

PARITY: порядок лучей = sensor_angles_deg из drone_geometry.yaml. RL обязан читать в ЭТОМ
порядке. Клип/sentinel — единая конвенция sim↔rl (контракт §2: no-return→1.0 после норм).
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Empty

from drone_sim.geometry import DroneGeometry

import random

N_BEAMS = 6


class TofRingNode(Node):
    def __init__(self) -> None:
        super().__init__("tof_ring_node")
        g = DroneGeometry.load()
        self.clip = g.tof_clip_m
        self.tof_min = g.tof_min_m
        self.declare_parameter("out_topic", "/krot/tof_ring")
        self.declare_parameter("rate_hz", g.control_hz)
        # ── ToF sim2real шум/дропаут (DR, off по умолчанию; модель research dev-log/83) ──
        # σ = max(noise_sigma_floor_m, noise_sigma_frac·d); p_dropout → SENTINEL (clip), НЕ skip
        # (включает §6 LOST-рефлекс + §2 validity). gz gpu_lidar чист → инжектим здесь.
        # КАНОН модели = research dev-log/83 (единый источник, как drone_geometry — оба инжектят 1-в-1)
        self.declare_parameter("noise_sigma_frac", 0.0)      # research: 0.03 (3%)
        self.declare_parameter("noise_sigma_floor_m", 0.015)  # research: 15мм
        self.declare_parameter("p_dropout", 0.0)             # research: 0.03, DR[0.03,0.15]→0.3
        self.declare_parameter("bias_max_m", 0.0)            # research: U[−15,+15]мм per-episode
        out_topic = self.get_parameter("out_topic").value
        rate = float(self.get_parameter("rate_hz").value)
        self.noise_frac = float(self.get_parameter("noise_sigma_frac").value)
        self.noise_floor = float(self.get_parameter("noise_sigma_floor_m").value)
        self.p_dropout = float(self.get_parameter("p_dropout").value)
        self.bias_max = float(self.get_parameter("bias_max_m").value)
        self._noise_on = self.noise_frac > 0.0 or self.p_dropout > 0.0 or self.bias_max > 0.0
        self._bias = random.uniform(-self.bias_max, self.bias_max) if self.bias_max > 0 else 0.0
        # per-episode bias: resample по сигналу (rl шлёт Empty на reset эпизода). Иначе = per-run.
        self.create_subscription(Empty, "/krot/noise_reset", self._resample_bias, 10)

        # no-hit инициализация = clip (сенсор жив, чисто до предела)
        self._beams = [self.clip] * N_BEAMS

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )
        for ch in range(N_BEAMS):
            self.create_subscription(
                LaserScan, f"/drone/vl53l0x/ch{ch}",
                lambda m, c=ch: self._scan_cb(m, c), sensor_qos,
            )
        self._pub = self.create_publisher(Float32MultiArray, out_topic, 10)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"tof_ring_node: /drone/vl53l0x/ch0..5 → {out_topic} @ {rate:.0f}Hz "
            f"(canon {g.sensor_angles_deg}, clip {self.clip}m"
            + (f"; NOISE σfrac={self.noise_frac} σfloor={self.noise_floor} "
               f"p_drop={self.p_dropout} bias_max={self.bias_max} (canon dev-log/83))"
               if self._noise_on else ")")
        )

    def _resample_bias(self, _msg: Empty) -> None:
        """per-episode bias resample (rl шлёт на reset эпизода) — канон dev-log/83."""
        if self.bias_max > 0:
            self._bias = random.uniform(-self.bias_max, self.bias_max)

    def _noisy(self, d: float) -> float:
        """ToF sim2real (канон dev-log/83): dropout→SENTINEL(clip); иначе +bias +Гаусс σ=max(floor,frac·d)."""
        if random.random() < self.p_dropout:
            return self.clip                       # invalid-return → sentinel (no-hit), включает LOST
        sigma = max(self.noise_floor, self.noise_frac * d)
        d = d + self._bias + random.gauss(0.0, sigma)
        if d >= self.clip:
            return self.clip
        return self.tof_min if d < self.tof_min else d

    def _scan_cb(self, msg: LaserScan, ch: int) -> None:
        if not msg.ranges:
            return
        d = float(msg.ranges[0])
        if math.isnan(d) or not math.isfinite(d) or d > self.clip:
            d = self.clip          # no-hit → sentinel
        elif d < self.tof_min:
            d = self.tof_min
        self._beams[ch] = d

    def _publish(self) -> None:
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = "beam_canon_0_60_120_180_240_300_deg"
        dim.size = N_BEAMS
        dim.stride = N_BEAMS
        msg.layout.dim = [dim]
        beams = self._beams if not self._noise_on else [self._noisy(b) for b in self._beams]
        msg.data = [float(b) for b in beams]
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TofRingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
