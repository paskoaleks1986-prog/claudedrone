#!/usr/bin/env python3
"""yaw_slave_node — нос (yaw) СЛЕЙВ к касательной стены (v5-krot Этап-1).

Контракт v2 §0/§3: yaw НЕ выход политики. Отдельный канал, на этапе-1 слейвится
к касательной стены (из GT-геометрии). Так «нос привязан к стене», а пилот учит
только трансляцию (мир-скорость) → краб исчезает structurally.

ВХОД:
    /krot/wall_gt   Float32MultiArray (от wall_gt_node) — берём t̂ (idx 6,7) + validity (10)
    param follow_dir ∈ {-1,+1} — вдоль какой стороны касательной смотреть нос
ВЫХОД (proposal-имя, согласовать с interface):
    /krot/yaw_cmd   std_msgs/Float64 — желаемый АБСОЛЮТНЫЙ yaw носа в МИРЕ (рад)

LOST (validity=0): держим последний валидный yaw (контракт §6 LOST-рефлекс, нос-канал).
Реальный поворот носа выполняет vel_setpoint_mux (yaw в PositionTarget); это демпфируется
ArduPilot yaw-контроллером (gain «Cr» ~критич.демпфир., контракт §8).
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64


class YawSlaveNode(Node):
    def __init__(self) -> None:
        super().__init__("yaw_slave_node")
        self.declare_parameter("wall_gt_topic", "/krot/wall_gt")
        self.declare_parameter("out_topic", "/krot/yaw_cmd")
        self.declare_parameter("follow_dir", 1)     # +1/-1 направление вдоль стены
        self.follow_dir = float(self.get_parameter("follow_dir").value)
        self._last_yaw = 0.0
        self._have = False

        self.create_subscription(
            Float32MultiArray, self.get_parameter("wall_gt_topic").value,
            self._gt_cb, 10)
        self._pub = self.create_publisher(
            Float64, self.get_parameter("out_topic").value, 10)
        self.get_logger().info(
            f"yaw_slave_node: {self.get_parameter('wall_gt_topic').value} t̂ → "
            f"{self.get_parameter('out_topic').value} (follow_dir={self.follow_dir:+.0f})"
        )

    def _gt_cb(self, m: Float32MultiArray) -> None:
        if len(m.data) < 11:
            return
        tx, ty = m.data[6], m.data[7]
        validity = m.data[10]
        if validity >= 0.5 and (abs(tx) > 1e-6 or abs(ty) > 1e-6):
            # нос вдоль касательной в направлении follow_dir
            self._last_yaw = math.atan2(self.follow_dir * ty, self.follow_dir * tx)
            self._have = True
        # LOST (validity<0.5): держим последний yaw
        if self._have:
            self._pub.publish(Float64(data=float(self._last_yaw)))


def main() -> None:
    rclpy.init()
    node = YawSlaveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
