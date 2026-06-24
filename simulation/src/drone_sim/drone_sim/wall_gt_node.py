#!/usr/bin/env python3
"""wall_gt_node — privileged GT-геометрия стены для критика + reward (v5-krot Этап-1).

Контракт v2 §4 + ответ research 12:4x (п.3). Sim — ЕДИНСТВЕННЫЙ источник истинной
геометрии: считает аналитически из дескриптора арены + истинной позы, БЕЗ сенсор-шума
(шум живёт в obs/DR). Критик асимметричный (privileged) и reward тейк-4 строятся ОТСЮДА.

ВХОД:
    pose-источник (param pose_topic, default /mavros/local_position/odom — на SITL
        fake-GPS odom≈GT ~1см; для строгого GT бриджить gz /world/<w>/pose/info)
    arena-дескриптор (param arena, *.arena.yaml): followable-стены + все препятствия
ВЫХОД (proposal-имя, согласовать с interface):
    /krot/wall_gt   std_msgs/Float32MultiArray, 12 float:
      [0]=x [1]=y [2]=yaw [3]=vx [4]=vy           ← истинная поза+скорость (мир)
      [5]=d_perp(знаковый, += дрон на свободной стороне)
      [6]=t_x [7]=t_y    ← касательная (unit, мир)
      [8]=n_x [9]=n_y    ← нормаль (unit, мир, стена→дрон)
      [10]=validity (1.0 если followable-стена в range R_usable иначе 0.0)
      [11]=min_obstacle_dist (до ЛЮБОГО препятствия, центр дрона; collision/no-graze)

ДЕСКРИПТОР АРЕНЫ (*.arena.yaml):
    followable:                       # стены, которые ведём (для d_perp/t̂/n̂)
      - {type: segment, p1: [x,y], p2: [x,y]}
      - {type: circle, center: [x,y], radius: R, inside: true}
    obstacles:                        # ВСЁ для min_obstacle_dist (стены+столбы)
      - {type: segment, p1: [x,y], p2: [x,y]}
      - {type: circle, center: [x,y], radius: R, inside: true}
    r_usable: 1.2                     # дальше → validity=0 (стена «не видна»)
"""
from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from drone_sim.geometry import DroneGeometry


def _yaw_from_quat(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def _nearest_on_segment(px, py, ax, ay, bx, by):
    """Ближайшая точка на отрезке a→b к (px,py); + касательная (unit, направление a→b)."""
    abx, aby = bx - ax, by - ay
    seg2 = abx * abx + aby * aby
    if seg2 < 1e-12:
        return ax, ay, 1.0, 0.0
    t = ((px - ax) * abx + (py - ay) * aby) / seg2
    t = max(0.0, min(1.0, t))
    qx, qy = ax + t * abx, ay + t * aby
    L = math.hypot(abx, aby)
    return qx, qy, abx / L, aby / L


def _wall_geom(px, py, wall):
    """→ (dist, qx, qy, tx, ty): дист до стены, ближайшая точка, касательная (unit)."""
    if wall["type"] == "segment":
        ax, ay = wall["p1"]
        bx, by = wall["p2"]
        qx, qy, tx, ty = _nearest_on_segment(px, py, ax, ay, bx, by)
        return math.hypot(px - qx, py - qy), qx, qy, tx, ty
    if wall["type"] == "circle":
        cx, cy = wall["center"]
        R = float(wall["radius"])
        r = math.hypot(px - cx, py - cy)
        if r < 1e-9:
            return R, cx + R, cy, 0.0, 1.0
        ux, uy = (px - cx) / r, (py - cy) / r        # центр→дрон
        qx, qy = cx + R * ux, cy + R * uy            # ближайшая точка на окружности
        dist = abs(R - r)
        tx, ty = -uy, ux                             # касательная = rot90 радиали (CCW)
        return dist, qx, qy, tx, ty
    raise ValueError(f"unknown wall type: {wall['type']}")


class WallGtNode(Node):
    def __init__(self) -> None:
        super().__init__("wall_gt_node")
        g = DroneGeometry.load()
        self.declare_parameter("arena", "")
        self.declare_parameter("pose_topic", "/mavros/local_position/odom")
        self.declare_parameter("out_topic", "/krot/wall_gt")
        self.declare_parameter("rate_hz", g.control_hz)
        arena_path = self.get_parameter("arena").value
        if not arena_path or not Path(arena_path).exists():
            raise SystemExit(f"wall_gt_node: --arena дескриптор не найден: '{arena_path}'")
        spec = yaml.safe_load(Path(arena_path).read_text())
        self.followable = spec.get("followable", [])
        self.obstacles = spec.get("obstacles", self.followable)
        self.r_usable = float(spec.get("r_usable", 1.2))
        if not self.followable:
            raise SystemExit(f"{arena_path}: пустой followable — нечего вести")

        self._x = self._y = self._yaw = 0.0
        self._vx = self._vy = 0.0
        self._have_pose = False

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, self.get_parameter("pose_topic").value,
                                 self._odom_cb, sensor_qos)
        self._pub = self.create_publisher(
            Float32MultiArray, self.get_parameter("out_topic").value, 10)
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"wall_gt_node: arena={Path(arena_path).name} "
            f"({len(self.followable)} followable, {len(self.obstacles)} obstacles, "
            f"r_usable={self.r_usable}) → {self.get_parameter('out_topic').value} @ {rate:.0f}Hz"
        )

    def _odom_cb(self, m: Odometry) -> None:
        p = m.pose.pose.position
        self._x, self._y = p.x, p.y
        self._yaw = _yaw_from_quat(m.pose.pose.orientation)
        # velocity_body? odom.twist в child_frame (body на ArduPilot) → переводим в мир
        vbx, vby = m.twist.twist.linear.x, m.twist.twist.linear.y
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        self._vx = vbx * c - vby * s
        self._vy = vbx * s + vby * c
        self._have_pose = True

    def _publish(self) -> None:
        if not self._have_pose:
            return
        px, py = self._x, self._y

        # followable: выбираем БЛИЖАЙШУЮ стену
        best = None
        for w in self.followable:
            dist, qx, qy, tx, ty = _wall_geom(px, py, w)
            if best is None or dist < best[0]:
                best = (dist, qx, qy, tx, ty)
        dist, qx, qy, tx, ty = best
        # нормаль стена→дрон (unit)
        nx, ny = px - qx, py - qy
        nlen = math.hypot(nx, ny)
        if nlen > 1e-9:
            nx, ny = nx / nlen, ny / nlen
        else:
            nx, ny = 0.0, 0.0
        d_perp = dist          # знак: + (дрон снаружи стены по нормали); абс-дист до поверхности
        validity = 1.0 if dist <= self.r_usable else 0.0

        # min_obstacle_dist по ВСЕМ препятствиям
        min_obs = float("inf")
        for w in self.obstacles:
            d = _wall_geom(px, py, w)[0]
            if d < min_obs:
                min_obs = d

        msg = Float32MultiArray()
        msg.data = [float(v) for v in (
            px, py, self._yaw, self._vx, self._vy,
            d_perp, tx, ty, nx, ny, validity, min_obs,
        )]
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = WallGtNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
