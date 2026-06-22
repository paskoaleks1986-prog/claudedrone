#!/usr/bin/env python3
"""teleop_keyboard.py — клавиатурный teleop → /drone/cmd_vel_body (FlightRL-v1, Aleks 2026-06-10).

Ручной драйв пока Interface-GUI не готов. Публикует geometry_msgs/Twist (BODY-frame) 10 Гц.
  W/S — vx вперёд/назад      A/D — vy влево/вправо
  Q/E — yaw_rate влево/вправо (CCW/CW)
  SPACE — полный стоп (нули)  X/Ctrl-C — выход
Скорости в нормализованных долях (×V_MAX на стороне executor): шаг 0.25, клип ±1.

Предусловие: запущен manual_fly_node (он взлетел и слушает /drone/cmd_vel_body).
Запуск: `ros2 run policy_bridge teleop_keyboard`  (в ОТДЕЛЬНОМ терминале от manual_fly_node)
"""
from __future__ import annotations

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

CMD_TOPIC = "/drone/cmd_vel_body"
STEP = 0.25
HELP = (
    "teleop /drone/cmd_vel_body — W/S=vx  A/D=vy  Q/E=yaw  SPACE=stop  X=quit "
    "(vx вперёд+, vy влево+)"
)


def _get_key(timeout: float) -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> int:
    rclpy.init()
    node = Node("teleop_keyboard")
    pub = node.create_publisher(Twist, CMD_TOPIC, 10)
    vx = vy = wz = 0.0

    def clamp(v: float) -> float:
        return max(-1.0, min(1.0, v))

    print(HELP)
    try:
        while rclpy.ok():
            k = _get_key(0.1).lower()
            if k == "w": vx = clamp(vx + STEP)
            elif k == "s": vx = clamp(vx - STEP)
            elif k == "a": vy = clamp(vy + STEP)
            elif k == "d": vy = clamp(vy - STEP)
            elif k == "q": wz = clamp(wz + STEP)
            elif k == "e": wz = clamp(wz - STEP)
            elif k == " ": vx = vy = wz = 0.0
            elif k == "x": break
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = wz
            pub.publish(msg)
            if k:
                print(f"\rvx={vx:+.2f} vy={vy:+.2f} yaw_rate={wz:+.2f}        ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        pub.publish(Twist())  # стоп
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
