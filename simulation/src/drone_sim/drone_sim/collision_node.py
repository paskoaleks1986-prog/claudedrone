#!/usr/bin/env python3
"""collision_node — физконтакт crash-ring → /krot/collision (TASK-099).

Краш-детект обучения (задание rl-lab по слову Aleks 2026-10-17, боль №1:
«сессия шла, а дрон уже крашнулся — простой»). Источник — contact-sensor
`crash_contact` на crash_ring_link модели iris_claudedrone (невидимый
collision-цилиндр R = prop_tip 0.319 на высоте винтов): истина от физдвижка,
не ToF/EKF-прокси.

ВХОД:  /drone/crash_contacts  ros_gz_interfaces/Contacts (мост parameter_bridge
       из gz.msgs.Contacts; сенсор публикует только ПРИ контактах)
ВЫХОД: /krot/collision        std_msgs/Bool @ rate_hz — СОСТОЯНИЕ (true = контакт
       в последние window_s); СОБЫТИЕ = фронт false→true (+ WARN-лог с участником).
       Не latched (по заданию rl-lab).

Подъём: в составе krot_bridges.launch.py (мост+нода) или вручную:
    ros2 run ros_gz_bridge parameter_bridge \
        '/drone/crash_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
    ros2 run drone_sim collision_node
Мир должен грузить gz-sim-contact-system (генератор v5 + indoor_room/corridor).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool


class CollisionNode(Node):
    def __init__(self) -> None:
        super().__init__("collision_node")
        self.declare_parameter("contacts_topic", "/drone/crash_contacts")
        self.declare_parameter("out_topic", "/krot/collision")
        self.declare_parameter("rate_hz", 20.0)
        # окно удержания состояния: сенсор шлёт только при контакте; пропуск
        # одного-двух тиков физики не должен дребезжать false/true
        self.declare_parameter("window_s", 0.15)
        gp = lambda n: self.get_parameter(n).value
        self.window_s = float(gp("window_s"))

        self._last_contact_t: float | None = None
        self._state = False
        self._last_other = ""

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )
        self.create_subscription(Contacts, str(gp("contacts_topic")),
                                 self._contacts_cb, sensor_qos)
        self._pub = self.create_publisher(Bool, str(gp("out_topic")), 10)
        self.create_timer(1.0 / float(gp("rate_hz")), self._tick)
        self.get_logger().info(
            f"collision_node: {gp('contacts_topic')} → {gp('out_topic')} "
            f"@ {float(gp('rate_hz')):.0f}Hz (window {self.window_s}s, crash-ring prop_tip)"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _contacts_cb(self, msg: Contacts) -> None:
        if not msg.contacts:
            return
        self._last_contact_t = self._now()
        # имя второго участника (для лога события): collision1/2 — одна из них ring
        c = msg.contacts[0]
        names = [c.collision1.name, c.collision2.name]
        self._last_other = next(
            (n for n in names if "crash_ring" not in n), names[0])

    def _tick(self) -> None:
        now = self._now()
        state = (self._last_contact_t is not None
                 and (now - self._last_contact_t) < self.window_s)
        if state and not self._state:
            self.get_logger().warn(f"💥 КОНТАКТ (crash-ring ↔ {self._last_other})")
        elif self._state and not state:
            self.get_logger().info("контакт снят")
        self._state = state
        self._pub.publish(Bool(data=state))


def main() -> None:
    rclpy.init()
    node = CollisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
