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
      FRAME_LOCAL_NED: VX,VY = мир-скорость (клип v_max), PZ = hold_alt, YAW = нос.

Высота — hold (фикс z, контракт §3): шлём position.z = hold_alt (НЕ velocity z).
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
# используем VX, VY, PZ, YAW; игнор всё прочее
TYPE_MASK = (PT.IGNORE_PX | PT.IGNORE_PY | PT.IGNORE_VZ |
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

        self.hold_alt = float(self.get_parameter("hold_alt").value)
        self._latch = bool(self.get_parameter("latch_alt_from_odom").value)
        self._alt_latched = not self._latch

        self._vx = self._vy = 0.0
        self._yaw = 0.0
        self._have_yaw = False

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Twist, self.get_parameter("cmd_vel_topic").value,
                                 self._vel_cb, 10)
        self.create_subscription(Float64, self.get_parameter("yaw_topic").value,
                                 self._yaw_cb, 10)
        if self._latch:
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
        if not self._alt_latched:
            self.hold_alt = float(m.pose.pose.position.z)
            self._alt_latched = True
            self.get_logger().info(f"vel_setpoint_mux: hold_alt latched = {self.hold_alt:.2f}m")

    def _publish(self) -> None:
        if not self._alt_latched:
            return
        m = PT()
        m.coordinate_frame = PT.FRAME_LOCAL_NED
        m.type_mask = TYPE_MASK
        m.velocity.x, m.velocity.y, m.velocity.z = self._vx, self._vy, 0.0
        m.position.z = self.hold_alt
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
