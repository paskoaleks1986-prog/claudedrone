#!/usr/bin/env python3
"""zone_semantic_server.py — V4-A1: семантический зона-сервер для Gazebo-гейта.

ТЗ rl-lab 2026-06-20 09:1x (директива Aleks: rl-lab владеет train+tests/launch/
monitor; sim = инфра). Грузит `<map>.zones.yaml` (схема v4-zones/1, owner rl-lab,
спека `rl-lab:v4/semantic/v4_zone_config_v1.md`), по позе дрона из coord/odom →
активная зона → уведомление топиком `/drone/zone_event` (std_msgs/String JSON):
    {zone_id, type, label, score_value, sectors_ok, in_course}

⚠ ПАРИТИ (критично, шрам TF-Luna): зона-расчёт ДОЛЖЕН совпасть с rl-lab
`BlindCorridorEnv._perp_to_wall`/`in_zone`, иначе Gazebo-поведение ≠ обучению.
Достигается репликой их `sensors._raycast` (перп-к-heading до стены) на ТОЙ ЖЕ
occupancy.npz (IoU=1.0 бит-в-бит с их train-стабом) + теми же порогами.

Кадры:
  - occupancy.npz: gz-центрированный, origin_gz=(-w/2,-h/2), res=0.1, iy=0 юг.
  - поза дрона (odom): метры, gz-кадр. cell = (pose_gz - origin_gz)/res (continuous,
    как у них x,y в клетках) → raycast int()-floor совпадает.
  - zones.yaml: метры в FLYABLE-SW-кадре (их конвенция: finish x[9.5,10], wall:bottom).
    flyable_sw_origin = meta.interior_bounds_gz[0]; pose_sw = pose_gz - flyable_sw_origin.
  §0 parity: юг(−y)=ПРАВО=env idx{5,4}; север(+y)=ЛЕВО=idx{1,2}. wall:bottom→side right.

Usage (ros2):
  ros2 run drone_sim zone_semantic_server --ros-args \
    -p world:=corridor_straight_m -p zones_file:=<path>.zones.yaml
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, String

from drone_sim.zone_geometry import classify_zone


class ZoneSemanticServer(Node):
    def __init__(self):
        super().__init__("zone_semantic_server")
        self.declare_parameter("world", "corridor_straight_m")
        self.declare_parameter("zones_file", "")
        self.declare_parameter("worlds_root", "")
        self.declare_parameter("rate_hz", 10.0)
        world = self.get_parameter("world").value
        zones_file = self.get_parameter("zones_file").value
        worlds_root = self.get_parameter("worlds_root").value or self._default_worlds_root()

        world_dir = Path(worlds_root) / world
        self._load_map(world_dir)
        if not zones_file:
            zones_file = str(world_dir / f"{world}.zones.yaml")
        self._load_zones(zones_file)

        # состояние позы
        self.x_gz = self.y_gz = self.heading = 0.0
        self.vel_world = (0.0, 0.0)                       # из odom twist (для course-score)
        self.have_pose = False
        self.vl = np.full(6, np.inf, dtype=np.float32)   # /drone/perimeter (м), inf=нет стены

        self.create_subscription(Odometry, "/mavros/local_position/odom",
                                 self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "/drone/perimeter",
                                 self._perim_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(String, "/drone/zone_event", 10)
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"ZoneSemanticServer up: world={world} zones={Path(zones_file).name} "
            f"grid={self.grid.shape} origin_gz={self.origin_gz.tolist()} "
            f"flyable_sw_origin={self.flyable_sw_origin} zones={len(self.zones)}")

    # ── загрузка карты/зон ──
    def _default_worlds_root(self):
        # Канон ROS2: миры ставятся install(DIRECTORY worlds DESTINATION share/drone_sim)
        # → берём через ament share-dir, а НЕ относительно __file__ (symlink-install
        # резолвил __file__ в src → parents[2] мимо). Source-fallback для оффлайн/dev.
        try:
            from ament_index_python.packages import get_package_share_directory
            cand = Path(get_package_share_directory("drone_sim")) / "worlds" / "worlds_v4a1"
            if cand.exists():
                return str(cand)
        except Exception:
            pass
        here = Path(__file__).resolve()
        return str(here.parents[1] / "worlds" / "worlds_v4a1")  # src/drone_sim/worlds/...

    def _load_map(self, world_dir: Path):
        occ = np.load(world_dir / "occupancy.npz")
        o = occ["occupancy"]
        self.grid = (o != 0).astype(np.uint8)        # бинаризация == их env (!=0)
        self.res = float(occ["resolution_m"])
        self.origin_gz = occ["origin_gz"].astype(np.float64)   # (-w/2,-h/2)
        meta = json.loads((world_dir / "meta.json").read_text())
        # flyable SW-угол в gz = нижне-левый угол flyable-прямоугольника
        self.flyable_sw_origin = meta["interior_bounds_gz"][0]   # [x0,y0]

    def _load_zones(self, zones_file: str):
        cfg = yaml.safe_load(Path(zones_file).read_text())
        self.task = cfg.get("task")
        self.course_dir = np.array(cfg.get("course_dir", [1.0, 0.0]), dtype=np.float64)
        self.zones = cfg["zones"]

    # ── колбэки ──
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x_gz, self.y_gz = p.x, p.y
        self.heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tw = msg.twist.twist.linear           # world-ish планарная скорость (course-score)
        self.vel_world = (tw.x, tw.y)
        self.have_pose = True

    def _perim_cb(self, msg: Float32MultiArray):
        d = list(msg.data)
        if len(d) >= 6:
            self.vl = np.array(d[:6], dtype=np.float32)

    # ── основной тик (вся геометрия — в zone_geometry.classify_zone, пар-тест оффлайн) ──
    def _tick(self):
        if not self.have_pose:
            return
        evt = classify_zone(
            self.grid, self.res, self.origin_gz, self.flyable_sw_origin,
            self.zones, self.course_dir, self.x_gz, self.y_gz, self.heading,
            self.vl, vel_world=self.vel_world)
        if evt is None:
            return
        m = String(); m.data = json.dumps(evt, ensure_ascii=False)
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ZoneSemanticServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
