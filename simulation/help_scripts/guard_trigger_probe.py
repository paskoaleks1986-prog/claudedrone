#!/usr/bin/env python3
"""guard_trigger_probe — лог vl[0..5] в момент каждого guard-триггера.

Run H (2026-06-07, Aleks): floor 0.40 / margin 0.45 — проверяем, закрыл ли
буфер 0.05 парковочные триггеры рана G (135/135 в полосе 0.437-0.450 на
vl[1]/vl[5]). Критерий: триггер при vl[1]/vl[5] > 0.40 = угловая геометрия
→ вариант (а) margin 0.55.

Подписки: /safety/active (Bool, rising edge = триггер), /drone/perimeter
(Float32MultiArray 10 Hz, 6×VL53 raw sensor-frame метры).

Запуск (tmux окно probe):
    python3 help_scripts/guard_trigger_probe.py \
        | tee /data/drone_media/sim/_runtime_logs/guard_probe-runH.log
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class GuardTriggerProbe(Node):
    def __init__(self) -> None:
        super().__init__("guard_trigger_probe")
        self._vl = [float("nan")] * 6
        self._active = False
        self._trigger_n = 0
        self.create_subscription(
            Float32MultiArray, "/drone/perimeter", self._perimeter_cb, 10
        )
        self.create_subscription(Bool, "/safety/active", self._active_cb, 10)
        self.get_logger().info(
            "probe ready: rising edge /safety/active -> snapshot vl[0..5]"
        )

    def _perimeter_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 6:
            self._vl = [float(d) for d in msg.data[:6]]

    def _active_cb(self, msg: Bool) -> None:
        if msg.data and not self._active:
            self._trigger_n += 1
            vl = self._vl
            self.get_logger().warn(
                f"TRIGGER #{self._trigger_n} · "
                f"vl[1]={vl[1]:.3f} vl[5]={vl[5]:.3f} · "
                f"all=[{', '.join(f'{d:.3f}' for d in vl)}] · "
                f"min_oblique={min(vl[1], vl[5]):.3f}"
            )
        self._active = msg.data


def main() -> None:
    rclpy.init()
    node = GuardTriggerProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"probe done: {node._trigger_n} triggers total")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
