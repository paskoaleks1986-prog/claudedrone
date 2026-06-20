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

from drone_sim.zone_geometry import (VL_RANGE_CELLS, WALL_TO_SIDE,
                                      perp_to_wall_cells)


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
        here = Path(__file__).resolve()
        return str(here.parents[2] / "worlds" / "worlds_v4a1")

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
        self.have_pose = True

    def _perim_cb(self, msg: Float32MultiArray):
        d = list(msg.data)
        if len(d) >= 6:
            self.vl = np.array(d[:6], dtype=np.float32)

    # ── геометрия (парити) ──
    def _cell_xy(self):
        """gz-метры → continuous cell coords (как rl-lab pose в клетках)."""
        return ((self.x_gz - self.origin_gz[0]) / self.res,
                (self.y_gz - self.origin_gz[1]) / self.res)

    def _perp_to_wall(self, side: str) -> float:
        """Перп-дистанция до следуемой стены (м). Реплика BlindCorridorEnv."""
        cx, cy = self._cell_xy()
        return perp_to_wall_cells(self.grid, cx, cy, self.heading, side) * self.res

    def _sectors_in_contact(self, idxs) -> bool:
        """require_sectors в контакте: VL-reading < VL_max (= их vl_norm<0.999)."""
        vl_max = VL_RANGE_CELLS * self.res
        return all(self.vl[i] < vl_max - 1e-9 for i in idxs)

    def _pose_sw(self):
        return (self.x_gz - self.flyable_sw_origin[0],
                self.y_gz - self.flyable_sw_origin[1])

    def _in_zone(self, z) -> tuple[bool, bool]:
        """(in_zone, sectors_ok) для зоны z по текущей позе."""
        g = z["geometry"]
        kind = g["kind"]
        req = z.get("require_sectors", [])
        sectors_ok = self._sectors_in_contact(req) if req else True
        if kind == "wall_band":
            side = WALL_TO_SIDE.get(g["wall"], "right")
            perp = self._perp_to_wall(side)
            inside = perp <= g["width_m"]
            return (inside and sectors_ok, sectors_ok)
        if kind == "rect":
            xs, ys = self._pose_sw()
            xlo, xhi = g["x_m"]; ylo, yhi = g["y_m"]
            return (xlo <= xs <= xhi and ylo <= ys <= yhi, sectors_ok)
        if kind == "dead_band":
            # середина шириной width_m по центру коридора (поперёк); стены вне зоны
            _, ys = self._pose_sw()
            half = g["width_m"] / 2.0
            cy = self._corridor_mid_sw()
            return (abs(ys - cy) <= half, sectors_ok)
        if kind == "polygon":
            return (self._in_polygon(g["points"]), sectors_ok)
        # complement резолвится на уровне _tick (нужны прочие зоны)
        return (False, sectors_ok)

    def _corridor_mid_sw(self):
        # центр flyable поперёк оси (для dead_band); коридор вдоль X → середина по Y
        y0 = self.flyable_sw_origin[1]
        # interior высота = grid_h*res - 2*res (border); проще: half flyable
        return (self.grid.shape[0] - 2) * self.res / 2.0

    def _in_polygon(self, pts) -> bool:
        xs, ys = self._pose_sw()
        n = len(pts); inside = False; j = n - 1
        for i in range(n):
            xi, yi = pts[i]; xj, yj = pts[j]
            if ((yi > ys) != (yj > ys)) and \
               (xs < (xj - xi) * (ys - yi) / (yj - yi + 1e-12) + xi):
                inside = not inside
            j = i
        return inside

    def _score(self, z) -> float:
        s = z.get("reaction", {}).get("score")
        if s == "course_progress":
            v = self._vel_world()
            lam = z["reaction"].get("lambda_drift", 3.0)
            on = float(np.dot(v, self.course_dir))
            perp = abs(float(v[0] * self.course_dir[1] - v[1] * self.course_dir[0]))
            return on - lam * perp
        if s == "finish_bonus":
            return 1.0
        if s == "soft_neg":
            return -0.1
        return 0.0

    def _vel_world(self):
        return np.array([0.0, 0.0])   # twist-слой добавим при wire с odom twist

    # ── основной тик ──
    def _tick(self):
        if not self.have_pose:
            return
        active = None
        for z in self.zones:
            if z["geometry"]["kind"] == "complement":
                continue
            inside, sectors_ok = self._in_zone(z)
            if inside:
                active = (z, sectors_ok)
                break
        if active is None:   # ни одна именованная → complement (FORBIDDEN)
            comp = next((z for z in self.zones
                         if z["geometry"]["kind"] == "complement"), None)
            if comp is None:
                return
            z, sectors_ok = comp, True
        else:
            z, sectors_ok = active
        on_course = float(np.dot(self._vel_world(), self.course_dir))
        evt = {
            "zone_id": z["id"], "type": z["type"], "label": z.get("label", ""),
            "score_value": round(self._score(z), 4),
            "sectors_ok": bool(sectors_ok), "in_course": round(on_course, 4),
        }
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
