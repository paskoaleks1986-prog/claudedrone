#!/usr/bin/env python3
"""flight_recorder — полётные CSV-логи v5-krot: gt/odom/cmd/tof/events per эпизод (TASK-104).

Возврат рекордера в V5-стек (m1c_track_recorder вырезан фильтром 3a3d44e — записи
некому вести; прецедент teleop/TASK-094, decision K-SIM-0003). Канон распределения
(rl-lab HANDOFF 11:35): формат = rl-lab (csv_record_format v2, TASK-103) · рекордер =
sim (этот файл) · конвертер CSV→flight_log + реплей GUI = interface (TASK-105).
Директива dual-trajectory (Aleks 2026-06-10): пишем ОБЕ траектории — gz ground-truth
и sensor odom; их Δ = диагностика. Приёмка rl-lab: интеграл cmd ≈ gt-траектория.

ПОТОКИ (CSV per эпизод, каталог <out_root>/<run_id>/ep_NNN/):
    gt.csv      t,x,y,z,yaw        gz-истина: /world/<world>/pose/info (PoseArray без
                                   имён → дрон = ближайшая к odom поза в 3D; XY-матч
                                   ловил пол под дроном — z-бимодальность, см. m1c)
    odom.csv    t,x,y,z,yaw        sensor EKF: /mavros/local_position/odom
    cmd.csv     t,vx_world,vy_world,yaw_cmd
                                   ЭХО команд: /krot/cmd_vel_world (Twist, WORLD-frame,
                                   НЕ body v4!) + /krot/yaw_cmd (Float64, латчится);
                                   строка на каждое сообщение любого из двух топиков.
    tof.csv     t,ch0..ch5         /krot/tof_ring (канон, клип tof_clip — RL-контракт)
    events.csv  t,event,info       collision (фронт)/collision_clear/episode_reset
                                   (info=x;y;theta)/episode_ready (info=true|false)

ВРЕМЯ: все t = sim-time — топики /krot/* без header, штампуем на приёме часами ноды;
launch ставит use_sim_time:=true (/clock бриджится базовым стеком drone.launch).

ГРАНИЦЫ ЭПИЗОДОВ: /krot/episode_reset → закрыть текущий ep_NNN, открыть ep_NNN+1
(reset-событие — первая строка events НОВОГО эпизода). До первого reset пишем в ep_000
(ручные прогоны без reset-цикла тоже записываются).

УПРАВЛЕНИЕ: параметр enable (true в krot-профиле) + топик /krot/recorder/enable
(Bool; false = пауза записи, файлы флашатся). meta.json в корне рана.

Запуск: включён в krot_bridges.launch.py (record:=true). Отдельно:
    ros2 run drone_sim flight_recorder --ros-args -p run_id:=OTLADKA/krot_v5/manual
"""
from __future__ import annotations

import csv
import json
import math
import os
import re
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose2D, PoseArray, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray, Float64

STREAMS = {
    "gt": ["t", "x", "y", "z", "yaw"],
    "odom": ["t", "x", "y", "z", "yaw"],
    "cmd": ["t", "vx_world", "vy_world", "yaw_cmd"],
    "tof": ["t", "ch0", "ch1", "ch2", "ch3", "ch4", "ch5"],
    "events": ["t", "event", "info"],
}


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _detect_world() -> str:
    """Автодетект мира по gz topic -l (как reset_node); partition — из env стека."""
    try:
        out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                             text=True, timeout=5).stdout
        m = re.search(r"^/world/([^/]+)/", out, re.M)
        return m.group(1) if m else ""
    except (OSError, subprocess.TimeoutExpired):
        return ""


class FlightRecorder(Node):
    def __init__(self) -> None:
        super().__init__("flight_recorder")
        p = self.declare_parameter
        # out_root: медиа-зона sim ($DRONE_MEDIA_ROOT/sim — RW по AGENTS_MAP), не репо.
        default_root = os.path.join(
            os.environ.get("DRONE_MEDIA_ROOT", os.path.expanduser("~")), "sim", "flight_logs")
        p("out_root", default_root)
        p("run_id", "")                 # ""=<дата-время>; формат rl-lab: <sprint>/<model>/<ep>
        p("world", "")                  # ""=env DEFAULT_WORLD → автодетект gz topic -l
        p("enable", True)               # старт включённым (krot-профиль); runtime-гейт топиком
        p("flush_period_s", 1.0)        # периодический flush (crash-защита журнала)
        gp = lambda n: self.get_parameter(n).value

        self._enabled = bool(gp("enable"))
        world = str(gp("world")) or os.environ.get("DEFAULT_WORLD", "") or _detect_world()
        self._world = world
        run_id = str(gp("run_id")) or time.strftime("%Y%m%d_%H%M%S")
        root = os.path.join(str(gp("out_root")), run_id)
        base = root
        n = 1
        while os.path.exists(root):    # уникальный каталог = нет конкурентной записи в CSV
            root = f"{base}_{n}"
            n += 1
        self._root = root
        os.makedirs(self._root, exist_ok=True)

        self._ep = -1
        self._files: dict[str, object] = {}
        self._writers: dict[str, csv.writer] = {}
        self._counts: dict[str, int] = {}
        self._open_episode()            # ep_000: ручные прогоны без reset-цикла тоже пишутся

        self._last_odom: tuple[float, float, float] | None = None
        self._vx = self._vy = 0.0
        self._yaw_cmd: float | None = None
        self._collision = False

        with open(os.path.join(self._root, "meta.json"), "w") as f:
            json.dump({"run_id": run_id, "world": world,
                       "started_wall": time.strftime("%Y-%m-%dT%H:%M:%S"),
                       "frames": {"cmd": "world (v5, НЕ body v4)", "gt": "gz pose/info",
                                  "odom": "mavros EKF"},
                       "time_base": "sim-time (штамп на приёме, use_sim_time)",
                       "streams": {k: v for k, v in STREAMS.items()}}, f,
                      ensure_ascii=False, indent=1)

        sq = qos_profile_sensor_data
        self.create_subscription(Odometry, "/mavros/local_position/odom", self._odom_cb, sq)
        if world:
            self.create_subscription(PoseArray, f"/world/{world}/pose/info", self._gt_cb, sq)
        else:
            self.get_logger().warn("мир не определён (world/DEFAULT_WORLD/автодетект пусты) — "
                                   "gt.csv будет ПУСТ, остальные потоки пишутся")
        self.create_subscription(Twist, "/krot/cmd_vel_world", self._vel_cb, 10)
        self.create_subscription(Float64, "/krot/yaw_cmd", self._yaw_cb, 10)
        self.create_subscription(Float32MultiArray, "/krot/tof_ring", self._tof_cb, 10)
        self.create_subscription(Bool, "/krot/collision", self._coll_cb, 10)
        self.create_subscription(Pose2D, "/krot/episode_reset", self._reset_cb, 10)
        self.create_subscription(Bool, "/krot/episode_ready", self._ready_cb, 10)
        self.create_subscription(Bool, "/krot/recorder/enable", self._enable_cb, 10)

        self.create_timer(float(gp("flush_period_s")), self._flush)
        self.get_logger().info(
            f"flight_recorder: → {self._root} (world={world or '∅'}, "
            f"enable={self._enabled}, эпизод ep_000; границы = /krot/episode_reset)")

    # ── файлы/эпизоды ──
    def _open_episode(self) -> None:
        self._close_files()
        self._ep += 1
        d = os.path.join(self._root, f"ep_{self._ep:03d}")
        os.makedirs(d, exist_ok=True)
        for name, header in STREAMS.items():
            f = open(os.path.join(d, f"{name}.csv"), "w", newline="")
            w = csv.writer(f)
            w.writerow(header)
            self._files[name] = f
            self._writers[name] = w
        self._counts = {k: 0 for k in STREAMS}

    def _close_files(self) -> None:
        for f in self._files.values():
            try:
                f.close()
            except OSError:
                pass
        if self._files:
            self.get_logger().info(
                f"ep_{self._ep:03d} закрыт: " +
                " ".join(f"{k}={self._counts[k]}" for k in STREAMS))
        self._files, self._writers = {}, {}

    def _flush(self) -> None:
        for f in self._files.values():
            try:
                f.flush()
            except OSError:
                pass

    def _t(self) -> float:
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns * 1e-9

    def _row(self, stream: str, row: list) -> None:
        if not self._enabled or stream not in self._writers:
            return
        self._writers[stream].writerow(row)
        self._counts[stream] += 1

    # ── callbacks ──
    def _odom_cb(self, m: Odometry) -> None:
        pp, q = m.pose.pose.position, m.pose.pose.orientation
        self._last_odom = (pp.x, pp.y, pp.z)
        self._row("odom", [f"{self._t():.3f}", f"{pp.x:.4f}", f"{pp.y:.4f}", f"{pp.z:.4f}",
                           f"{_yaw_from_quat(q.x, q.y, q.z, q.w):.4f}"])

    def _gt_cb(self, m: PoseArray) -> None:
        # PoseArray без имён → дрон = ближайшая к odom поза в 3D (XY-матч ловил пол: m1c).
        if not m.poses or self._last_odom is None:
            return
        rx, ry, rz = self._last_odom
        best, bestd = None, 1e18
        for pose in m.poses:
            d = ((pose.position.x - rx) ** 2 + (pose.position.y - ry) ** 2
                 + (pose.position.z - rz) ** 2)
            if d < bestd:
                bestd, best = d, pose
        q = best.orientation
        self._row("gt", [f"{self._t():.3f}", f"{best.position.x:.4f}",
                         f"{best.position.y:.4f}", f"{best.position.z:.4f}",
                         f"{_yaw_from_quat(q.x, q.y, q.z, q.w):.4f}"])

    def _cmd_row(self) -> None:
        yc = "" if self._yaw_cmd is None else f"{self._yaw_cmd:.4f}"
        self._row("cmd", [f"{self._t():.3f}", f"{self._vx:.4f}", f"{self._vy:.4f}", yc])

    def _vel_cb(self, m: Twist) -> None:
        self._vx, self._vy = float(m.linear.x), float(m.linear.y)
        self._cmd_row()

    def _yaw_cb(self, m: Float64) -> None:
        self._yaw_cmd = float(m.data)
        self._cmd_row()

    def _tof_cb(self, m: Float32MultiArray) -> None:
        if len(m.data) < 6:
            return
        self._row("tof", [f"{self._t():.3f}"] + [f"{v:.4f}" for v in m.data[:6]])

    def _coll_cb(self, m: Bool) -> None:
        if m.data != self._collision:    # событийно: фронт/спад, не 20Гц состояние
            self._collision = bool(m.data)
            self._row("events", [f"{self._t():.3f}",
                                 "collision" if m.data else "collision_clear", ""])

    def _reset_cb(self, m: Pose2D) -> None:
        self._open_episode()             # reset = первая строка events НОВОГО эпизода
        self._row("events", [f"{self._t():.3f}", "episode_reset",
                             f"{m.x:.3f};{m.y:.3f};{m.theta:.3f}"])
        self.get_logger().info(f"→ ep_{self._ep:03d} (reset {m.x:.2f},{m.y:.2f})")

    def _ready_cb(self, m: Bool) -> None:
        self._row("events", [f"{self._t():.3f}", "episode_ready",
                             "true" if m.data else "false"])

    def _enable_cb(self, m: Bool) -> None:
        if bool(m.data) != self._enabled:
            self._enabled = bool(m.data)
            self._flush()
            self.get_logger().info(f"запись {'ВКЛ' if m.data else 'ПАУЗА (flush)'}")

    def close(self) -> None:
        self._close_files()


def main() -> None:
    rclpy.init()
    node = FlightRecorder()
    # SIGTERM (timeout/tmux kill — штатный стоп) → graceful close (финальный flush+лог)
    def _sig(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sig)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
