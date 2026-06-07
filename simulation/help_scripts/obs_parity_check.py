#!/usr/bin/env python3
"""obs_parity_check.py — численная эквивалентность obs: live стек vs Drone2DEnv.

Ревью Aleks (2026-06-06): «структурное совпадение ≠ численное. Взять state,
собрать obs в боевом стеке и в DroneEnv для того же state, сравнить числа».

Метод: N снапшотов с ЖИВОГО стека (odom + /drone/perimeter + /scan/sweep +
joint_state) → для каждой позы рейкаст в grid'е env → сравнение нормализованных
векторов distances[7] и servo_angle. Каналы соответствуют offset'ам
heading + 0..300°; рейкаст из той же точки в метрах→клетках ((x+3.2)/0.1).

Запуск при летящем стеке (из simulation/):
    PYTHONPATH=/data/git/rl-lab:/opt/ros/jazzy/lib/python3.12/site-packages \
        ./.venv-policy/bin/python3 help_scripts/obs_parity_check.py --samples 20

Caveats: дрон в движении → snapshot рассинхрон сенсоров/позы даёт шум —
смотрим mean/max |Δ| по каналам, не точное равенство. Идеал — повторить
на зависшем дроне.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

RL_LAB = os.environ.get("RL_LAB_ROOT", "/data/git/rl-lab")
sys.path.insert(0, RL_LAB)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan, JointState  # noqa: E402
from std_msgs.msg import Float32MultiArray  # noqa: E402

from envs import sensors  # noqa: E402

GRID = np.load(f"{RL_LAB}/maps/rl_rooms/rl_room_empty_6x6.npy").astype(np.uint8)
ROOM_HALF_M = 3.2
CELL_M = 0.1
VL_MAX_M = 1.2
TF_MAX_M = 6.4
WORLD = os.environ.get("DEFAULT_WORLD", "rl_room_empty_6x6")


class Snap(Node):
    def __init__(self) -> None:
        super().__init__("obs_parity_check")
        self.pose = None        # (x_m, y_m, yaw)
        self.perimeter = None   # 6 raw meters
        self.sweep = None       # raw meters
        self.servo_rad = None
        self.create_subscription(
            Odometry, "/mavros/local_position/odom", self._odom,
            qos_profile_sensor_data)
        self.create_subscription(
            Float32MultiArray, "/drone/perimeter", self._perim, 10)
        self.create_subscription(LaserScan, "/scan/sweep", self._sweep, 10)
        self.create_subscription(
            JointState,
            f"/world/{WORLD}/model/iris_claudedrone/joint_state",
            self._joint, 10)

    def _odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def _perim(self, m):
        if len(m.data) >= 6:
            self.perimeter = list(m.data[:6])

    def _sweep(self, m):
        if m.ranges and math.isfinite(m.ranges[0]):
            self.sweep = min(max(m.ranges[0], 0.0), TF_MAX_M)

    def _joint(self, m):
        try:
            i = list(m.name).index("sg90_joint")
            self.servo_rad = max(0.0, min(float(m.position[i]), math.pi))
        except (ValueError, IndexError):
            pass

    def ready(self):
        return None not in (self.pose, self.perimeter, self.sweep, self.servo_rad)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=20)
    ap.add_argument("--interval", type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = Snap()
    deltas_vl = []   # (6,) per sample
    deltas_tf = []
    rows = []

    t_end = time.monotonic() + 120
    while not node.ready() and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.ready():
        print("FAIL: нет данных со стека (стек запущен?)", file=sys.stderr)
        sys.exit(1)

    for n in range(args.samples):
        t = time.monotonic() + args.interval
        while time.monotonic() < t:
            rclpy.spin_once(node, timeout_sec=0.05)
        x_m, y_m, yaw = node.pose
        # метры → клетки env
        cx = (x_m + ROOM_HALF_M) / CELL_M
        cy = (y_m + ROOM_HALF_M) / CELL_M
        servo_deg = math.degrees(node.servo_rad)

        env_vl = sensors.vl53l0x_distances(GRID, cx, cy, yaw)        # norm [0,1]
        env_tf = sensors.tf_luna_distance(GRID, cx, cy, yaw, servo_deg)
        live_vl = np.clip(np.array(node.perimeter), 0, VL_MAX_M) / VL_MAX_M
        live_tf = node.sweep / TF_MAX_M

        deltas_vl.append(np.abs(env_vl - live_vl))
        deltas_tf.append(abs(env_tf - live_tf))
        rows.append((x_m, y_m, math.degrees(yaw), servo_deg,
                     float(np.max(np.abs(env_vl - live_vl))),
                     abs(env_tf - live_tf)))
        print(f"[{n:02d}] pose=({x_m:+.2f},{y_m:+.2f},{math.degrees(yaw):+6.1f}°) "
              f"servo={servo_deg:5.1f}° maxΔvl={rows[-1][4]:.3f} Δtf={rows[-1][5]:.3f}",
              flush=True)

    dvl = np.array(deltas_vl)
    dtf = np.array(deltas_tf)
    print("\n=== PARITY: live obs vs Drone2DEnv raycast (нормализованные [0,1]) ===")
    for i in range(6):
        print(f"  VL ch{i} (+{i*60}°): meanΔ {dvl[:, i].mean():.3f} "
              f"maxΔ {dvl[:, i].max():.3f}")
    print(f"  TF sweep:      meanΔ {dtf.mean():.3f} maxΔ {dtf.max():.3f}")
    print("Ориентир: Δ ≲ 0.05 (≈1 клетка/шум движения) — паритет; "
          "систематический Δ на канале — рассинхрон угла/фрейма; "
          "Δ ~ const на всех — масштаб/нормализация.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
