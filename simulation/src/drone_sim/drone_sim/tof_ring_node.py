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
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from drone_sim.geometry import DroneGeometry

N_BEAMS = 6


class TofRingNode(Node):
    def __init__(self) -> None:
        super().__init__("tof_ring_node")
        g = DroneGeometry.load()
        self.clip = g.tof_clip_m
        self.tof_min = g.tof_min_m
        self.declare_parameter("out_topic", "/krot/tof_ring")
        self.declare_parameter("rate_hz", g.control_hz)
        out_topic = self.get_parameter("out_topic").value
        rate = float(self.get_parameter("rate_hz").value)

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
            f"(canon {g.sensor_angles_deg}, clip {self.clip}m)"
        )

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
        msg.data = [float(b) for b in self._beams]
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
