#!/usr/bin/env python3
"""track_recorder.py — odom-трек + действия политики → CSV.

Инструмент директивы «каждый полёт в новом мире → трек» (world-switch.md):
вчера CSV собирался ad-hoc, теперь — скриптом.

Пишет (append-safe, заголовок один раз):
    <prefix>_odom.csv      t,x,y,z          (/mavros/local_position/odom)
    <prefix>_actions.csv   t,source,action  (source: raw=/rl_policy/action_raw
                                             ДО stuck/adaptive/gate, exec=/rl_policy/action)
    <prefix>_metrics.csv   t,name,value     (coverage, mapped_ratio)

QoS: odom — qos_profile_sensor_data (MAVROS BEST_EFFORT, см. memory
feedback_mavros_qos_best_effort); /rl_policy/* — дефолтный RELIABLE.

Usage:
    python3 help_scripts/track_recorder.py --prefix /tmp/track_v15c_empty6x6
    (Ctrl-C / SIGTERM — корректное закрытие файлов.)
"""
from __future__ import annotations

import argparse
import csv
import math
import signal
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Int32


class TrackRecorder(Node):
    def __init__(self, prefix: str) -> None:
        super().__init__("track_recorder")
        self._files = {}
        self._writers = {}
        for name, header in (
            ("odom", ["t", "x", "y", "z", "yaw"]),
            ("actions", ["t", "source", "action"]),
            ("metrics", ["t", "name", "value"]),
        ):
            f = open(f"{prefix}_{name}.csv", "w", newline="")
            w = csv.writer(f)
            w.writerow(header)
            self._files[name] = f
            self._writers[name] = w
        self.n_odom = 0

        self.create_subscription(
            Odometry, "/mavros/local_position/odom",
            self._odom_cb, qos_profile_sensor_data,
        )
        self.create_subscription(
            Int32, "/rl_policy/action_raw",
            lambda m: self._row("actions", "raw", m.data), 50,
        )
        self.create_subscription(
            Int32, "/rl_policy/action",
            lambda m: self._row("actions", "exec", m.data), 50,
        )
        self.create_subscription(
            Float32, "/rl_policy/coverage",
            lambda m: self._row("metrics", "coverage", m.data), 50,
        )
        self.create_subscription(
            Float32, "/rl_policy/mapped_ratio",
            lambda m: self._row("metrics", "mapped_ratio", m.data), 50,
        )
        self.get_logger().info(f"recording → {prefix}_{{odom,actions,metrics}}.csv")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _row(self, dest: str, tag: str, value) -> None:
        self._writers[dest].writerow([f"{self._now():.3f}", tag, value])

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw для heading-drift диагностики (план Aleks 2026-06-07)
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._writers["odom"].writerow(
            [f"{self._now():.3f}", f"{p.x:.4f}", f"{p.y:.4f}",
             f"{p.z:.4f}", f"{yaw:.5f}"]
        )
        self.n_odom += 1
        if self.n_odom % 500 == 0:
            self.get_logger().info(f"odom points: {self.n_odom}")

    def close(self) -> None:
        for f in self._files.values():
            f.flush()
            f.close()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True,
                    help="префикс CSV-файлов, например /tmp/track_v15c_empty6x6")
    args = ap.parse_args()

    rclpy.init()
    node = TrackRecorder(args.prefix)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.close()
        node.get_logger().info(f"closed, odom total: {node.n_odom}")
        node.destroy_node()


if __name__ == "__main__":
    main()
