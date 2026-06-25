#!/usr/bin/env python3
"""vel_setpoint_mux — склейка RL-скорости (мир) + нос-слейв yaw → MAVROS setpoint.

v5-krot Этап-1. РАЗВЯЗКА: пилот (RL) публикует ЧИСТУЮ 2D мировую скорость; нос ведёт
отдельный yaw_slave_node. Этот mux сводит их в ОДИН PositionTarget и шлёт в ArduPilot.
Так RL структурно НЕ может управлять носом (контракт §3: yaw вне политики).

ВХОД (proposal-имена, согласовать с interface):
    /krot/cmd_vel_world   geometry_msgs/Twist — (linear.x, linear.y) МИР-скорость м/с (RL-пилот)
    /krot/yaw_cmd         std_msgs/Float64    — абсолютный yaw носа в МИРЕ, рад (yaw_slave)
ВЫХОД:
    /mavros/setpoint_raw/local  mavros_msgs/PositionTarget
      FRAME_LOCAL_NED: VX,VY = мир-скорость (клип v_max), VZ = alt-hold, YAW = нос.

Высота — hold через VELOCITY.Z (P-контроллер vz=kp·(hold_alt−z), клип vz_max).
🛑 LIVE-RCA 2026-06-25: position.z-hold + velocity-xy (старый mask=2531, PZ не-игнор)
НЕ работает на ArduCopter 4.8-dev GUIDED — AP считает pos_ignore по ВСЕМ 3 осям
позиции; PZ не-игнор → НЕ входит в velocity-режим → латераль ИГНОРИТСЯ (дрон стоит).
Доказано на стенде: mask 2531 (PZ+VXY) дрон стоит; mask 2503 (игнор всей позиции,
VX/VY/VZ) дрон ЛЕТИТ + z держится. Контракт §3 «высота через position.z» физически
неисполним → высоту держим velocity.z (функц. эквивалент: z пинится к hold_alt).
v_max берётся из drone_geometry.yaml (единый источник).
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from drone_sim.geometry import DroneGeometry

PT = PositionTarget
# используем VX, VY, VZ, YAW; игнор ВСЮ позицию (incl PZ) + accel + yaw_rate.
# 🛑 PZ ОБЯЗАН быть игнорирован — иначе AP не входит в velocity-режим (см. docstring RCA).
TYPE_MASK = (PT.IGNORE_PX | PT.IGNORE_PY | PT.IGNORE_PZ |
             PT.IGNORE_AFX | PT.IGNORE_AFY | PT.IGNORE_AFZ | PT.IGNORE_YAW_RATE)


class VelSetpointMux(Node):
    def __init__(self) -> None:
        super().__init__("vel_setpoint_mux")
        g = DroneGeometry.load()
        self.v_max = g.v_max
        self.declare_parameter("cmd_vel_topic", "/krot/cmd_vel_world")
        self.declare_parameter("yaw_topic", "/krot/yaw_cmd")
        self.declare_parameter("out_topic", "/mavros/setpoint_raw/local")
        self.declare_parameter("hold_alt", 2.0)
        self.declare_parameter("latch_alt_from_odom", True)
        self.declare_parameter("rate_hz", g.control_hz)
        self.declare_parameter("alt_kp", 1.0)      # P-gain alt-hold (vz = kp·err)
        self.declare_parameter("vz_max", 0.5)      # клип |vz| alt-hold, м/с

        self.hold_alt = float(self.get_parameter("hold_alt").value)
        self._latch = bool(self.get_parameter("latch_alt_from_odom").value)
        self._alt_latched = not self._latch
        self.alt_kp = float(self.get_parameter("alt_kp").value)
        self.vz_max = float(self.get_parameter("vz_max").value)

        self._vx = self._vy = 0.0
        self._yaw = 0.0
        self._have_yaw = False
        self._z = 0.0
        self._have_odom = False

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Twist, self.get_parameter("cmd_vel_topic").value,
                                 self._vel_cb, 10)
        self.create_subscription(Float64, self.get_parameter("yaw_topic").value,
                                 self._yaw_cb, 10)
        # odom нужен ВСЕГДА (alt-hold через velocity.z требует текущий z).
        self.create_subscription(Odometry, "/mavros/local_position/odom",
                                 self._odom_cb, sensor_qos)
        self._pub = self.create_publisher(
            PT, self.get_parameter("out_topic").value, 10)
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"vel_setpoint_mux: cmd_vel(world)+yaw → setpoint_raw/local @ {rate:.0f}Hz, "
            f"v_max={self.v_max}, hold_alt={'odom-latch' if self._latch else self.hold_alt}"
        )

    def _vel_cb(self, m: Twist) -> None:
        vx, vy = float(m.linear.x), float(m.linear.y)
        sp = math.hypot(vx, vy)
        if sp > self.v_max and sp > 1e-9:          # клип |v| ≤ v_max
            k = self.v_max / sp
            vx, vy = vx * k, vy * k
        self._vx, self._vy = vx, vy

    def _yaw_cb(self, m: Float64) -> None:
        self._yaw = float(m.data)
        self._have_yaw = True

    def _odom_cb(self, m: Odometry) -> None:
        self._z = float(m.pose.pose.position.z)
        self._have_odom = True
        if not self._alt_latched:
            self.hold_alt = self._z
            self._alt_latched = True
            self.get_logger().info(f"vel_setpoint_mux: hold_alt latched = {self.hold_alt:.2f}m")

    def _publish(self) -> None:
        if not self._alt_latched or not self._have_odom:
            return
        # alt-hold через velocity.z (position.z НЕ работает с velocity-xy на AP, см. RCA).
        vz = self.alt_kp * (self.hold_alt - self._z)
        vz = max(-self.vz_max, min(self.vz_max, vz))
        m = PT()
        m.coordinate_frame = PT.FRAME_LOCAL_NED
        m.type_mask = TYPE_MASK
        m.velocity.x, m.velocity.y, m.velocity.z = self._vx, self._vy, vz
        m.yaw = self._yaw if self._have_yaw else 0.0
        self._pub.publish(m)


def main() -> None:
    rclpy.init()
    node = VelSetpointMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
