#!/usr/bin/env python3
"""m1c_track_recorder.py — DUAL-траектория + M1c-команды + zone-event → CSV (с t=0).

Заточен под V4-A1 M1c-прогон (Box3 cmd_vel_body + семант-зоны), в отличие от
track_recorder.py (старая дискретная схема /rl_policy/action Int32 + coverage).

Закрывает хвост #1 RUN-REPORT rl-lab (17:1x): «zone/odom-лог стартовал ПОСЛЕ
прилёта → нет per-step drift/perp по ходу». ⇒ запускать ДО взлёта manual_fly,
тогда захват с t=0.

Директива dual-trajectory (Aleks 2026-06-10, memory feedback_dual_trajectory_*):
пишем ОБЕ траектории — gz GROUND-TRUTH (мой слой) + sensor odom; их Δ = диагностика.
На SITL fake-GPS Δ≈0 (memory project_sitl_fakegps_near_zero_drift), но захват
обязателен (на hardware/optflow Δ раскроется).

Пишет (заголовок один раз, корректное закрытие по Ctrl-C/SIGTERM):
    <prefix>_gt.csv    t,x,y,z,yaw     gz ground-truth (drone pose из PoseArray,
                                       /world/<world>/pose/info, индекс = ближайший к odom)
    <prefix>_odom.csv  t,x,y,z,yaw     sensor (/mavros/local_position/odom)
    <prefix>_cmd.csv   t,vx,vy,yaw_rate  M1c action (/drone/cmd_vel_body, Twist)
    <prefix>_zone.csv  t,type,label,score_value,in_course,sectors_ok
                                       (/drone/zone_event, std_msgs/String JSON)

perp-to-follow-wall профиль = gt_y (низ-стена y=0, side=right) → drift/perp считается
оффлайн из _gt.csv + геометрии коридора (track_plot / summary).

QoS: odom + pose/info — qos_profile_sensor_data (BEST_EFFORT, memory
feedback_mavros_qos_best_effort); cmd_vel/zone_event — дефолт RELIABLE.

Usage:
    python3 help_scripts/m1c_track_recorder.py --world corridor_straight_m \
        --prefix /tmp/m1c_corridor_run2
    # запускать ПЕРЕД `manual_fly_node --auto-takeoff` (захват взлёта+траверса с t=0)
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class M1cTrackRecorder(Node):
    def __init__(self, prefix: str, world: str) -> None:
        super().__init__("m1c_track_recorder")
        self._files = {}
        self._writers = {}
        for name, header in (
            ("gt", ["t", "x", "y", "z", "yaw"]),
            ("odom", ["t", "x", "y", "z", "yaw"]),
            ("cmd", ["t", "vx", "vy", "yaw_rate"]),
            ("zone", ["t", "type", "label", "score_value", "in_course", "sectors_ok"]),
        ):
            f = open(f"{prefix}_{name}.csv", "w", newline="")
            w = csv.writer(f)
            w.writerow(header)
            self._files[name] = f
            self._writers[name] = w
        self.n = {"gt": 0, "odom": 0, "cmd": 0, "zone": 0}
        self._last_odom: tuple[float, float, float] | None = None

        self.create_subscription(
            Odometry, "/mavros/local_position/odom", self._odom_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseArray, f"/world/{world}/pose/info", self._gt_cb, qos_profile_sensor_data
        )
        self.create_subscription(Twist, "/drone/cmd_vel_body", self._cmd_cb, 10)
        self.create_subscription(String, "/drone/zone_event", self._zone_cb, 10)
        self.get_logger().info(
            f"m1c_track_recorder up: world={world} prefix-файлы открыты. "
            f"gt=/world/{world}/pose/info odom=/mavros/local_position/odom "
            f"cmd=/drone/cmd_vel_body zone=/drone/zone_event. Запускать ДО взлёта."
        )

    def _t(self) -> float:
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns * 1e-9

    def _odom_cb(self, m: Odometry) -> None:
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self._last_odom = (p.x, p.y, p.z)
        self._writers["odom"].writerow(
            [f"{self._t():.3f}", f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}",
             f"{_yaw_from_quat(q.x, q.y, q.z, q.w):.4f}"]
        )
        self.n["odom"] += 1

    def _gt_cb(self, m: PoseArray) -> None:
        # PoseArray не несёт имён → дрон = pose, ближайший к odom в 3D (x,y,z).
        # ⚠ XY-only матч ловил пол/основание ПОД дроном (тот же xy, z≈0) →
        # z-артефакт (фикс после ре-рана #2: z бимодально 0/1). 3D-матч (вкл. z)
        # снимает вертикальную неоднозначность стека сущностей.
        if not m.poses:
            return
        ref = self._last_odom
        if ref is None:
            return  # ждём первый odom для якоря (доли секунды)
        best, bestd = None, 1e18
        for pose in m.poses:
            d = ((pose.position.x - ref[0]) ** 2 + (pose.position.y - ref[1]) ** 2
                 + (pose.position.z - ref[2]) ** 2)
            if d < bestd:
                bestd, best = d, pose
        if best is None:
            return
        q = best.orientation
        self._writers["gt"].writerow(
            [f"{self._t():.3f}", f"{best.position.x:.4f}", f"{best.position.y:.4f}",
             f"{best.position.z:.4f}", f"{_yaw_from_quat(q.x, q.y, q.z, q.w):.4f}"]
        )
        self.n["gt"] += 1

    def _cmd_cb(self, m: Twist) -> None:
        self._writers["cmd"].writerow(
            [f"{self._t():.3f}", f"{m.linear.x:.4f}", f"{m.linear.y:.4f}", f"{m.angular.z:.4f}"]
        )
        self.n["cmd"] += 1

    def _zone_cb(self, m: String) -> None:
        try:
            e = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self._writers["zone"].writerow(
            [f"{self._t():.3f}", e.get("type", ""), e.get("label", ""),
             e.get("score_value", ""), e.get("in_course", ""), e.get("sectors_ok", "")]
        )
        self.n["zone"] += 1

    def close(self) -> None:
        for f in self._files.values():
            try:
                f.close()
            except OSError:
                pass
        self.get_logger().info(
            f"закрыто: gt={self.n['gt']} odom={self.n['odom']} "
            f"cmd={self.n['cmd']} zone={self.n['zone']} строк."
        )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True, help="префикс выходных CSV")
    ap.add_argument("--world", default="corridor_straight_m", help="имя мира (для pose/info топика)")
    args = ap.parse_args()

    rclpy.init()
    node = M1cTrackRecorder(args.prefix, args.world)

    def _sig(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _sig)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
