#!/usr/bin/env python3
"""tfluna_sweep_adapter.py — gz серво-sweep → `tfluna_arc` (контракт §1).

Pure-функция (без ROS) + тонкая ROS2-нода. Конвертит завершённый sweep-проход
`/drone/sweep/result` (sensor_msgs/LaserScan, ranges[i] @ θ_servo=angle_min+i·inc)
в формат памяти §1: list[(angle_rad_body, dist_m | None)].

Геометрия (из model.sdf / scan_points_node): серво θ∈[0,π], θ=π/2 = НОС →
  bearing_body = θ_servo − π/2 ∈ [−π/2, +π/2]   (контракт-веер ±90°).
🔒 None = no-return (луч ушёл за max / inf / nan) — НЕ max-число (контракт §1:
отличаем «открыто за горизонтом» от далёкого хита).

⚠ Знак/zero bearing выведены из SDF — требуют Gazebo stand-verify перед закрытием
(правило: SDF/sim-таск без смоука не закрывается). Веер ОБЩИЙ с B4 D̂ — строим раз.
"""
from __future__ import annotations

import math

NOSE_OFFSET = math.pi / 2.0     # θ_servo носа (π/2); bearing = θ − NOSE_OFFSET


def laserscan_to_arc(angle_min, angle_increment, ranges, range_max,
                     range_min=0.0, nose_offset=NOSE_OFFSET):
    """LaserScan-поля → list[(angle_rad_body, dist|None)] по контракту §1.

    None если: inf/nan, r ≥ range_max (no-return), r ≤ range_min (невалид).
    """
    arc = []
    for i, r in enumerate(ranges):
        theta = angle_min + i * angle_increment
        bearing = theta - nose_offset
        if r is None or math.isinf(r) or math.isnan(r) \
                or r >= range_max or r <= range_min:
            dist = None
        else:
            dist = float(r)
        arc.append((float(bearing), dist))
    return arc


# ───────────────────────── ROS2 нода (тонкая обёртка) ───────────────────────
def main(args=None):  # pragma: no cover (требует стенда)
    import json

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    class TflunaArcNode(Node):
        """/drone/sweep/result (LaserScan) → /scan/tfluna_arc (String JSON §1).

        JSON: {"t": sec, "arc": [[angle_rad, dist|null], ...]}. Для памяти/мониторинга;
        MemoryModule.step ест tfluna_arc прямо из этого (interface/rl-lab парсят)."""

        def __init__(self):
            super().__init__("tfluna_arc_node")
            self.declare_parameter("nose_offset_rad", NOSE_OFFSET)
            self.nose = self.get_parameter("nose_offset_rad").value
            self.pub = self.create_publisher(String, "/scan/tfluna_arc", 10)
            self.sub = self.create_subscription(
                LaserScan, "/drone/sweep/result", self._on_sweep,
                qos_profile_sensor_data)
            self.get_logger().info(
                "tfluna_arc_node: /drone/sweep/result → /scan/tfluna_arc "
                f"(nose_offset={self.nose:.4f})")

        def _on_sweep(self, msg: LaserScan):
            arc = laserscan_to_arc(
                msg.angle_min, msg.angle_increment, list(msg.ranges),
                msg.range_max, msg.range_min, self.nose)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            out = String()
            out.data = json.dumps({"t": t, "arc": [[a, d] for (a, d) in arc]})
            self.pub.publish(out)

    rclpy.init(args=args)
    node = TflunaArcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
