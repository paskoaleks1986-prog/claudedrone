#!/usr/bin/env python3
"""flight_recorder — полётные CSV-логи v5-krot по формату rl-lab v2 (TASK-104).

Возврат рекордера в V5-стек (m1c_track_recorder вырезан фильтром 3a3d44e — записи
некому вести; прецедент teleop/TASK-094, decision K-SIM-0003). Канон распределения
(rl-lab HANDOFF 11:35): ФОРМАТ = rl-lab `rl-lab/docs/csv_record_format_v2.md` (TASK-103)
· рекордер = sim (этот файл) · конвертер CSV→flight_log/v4 + реплей GUI = interface
(TASK-105). Директива dual-trajectory (Aleks 2026-06-10): пишем ОБЕ траектории —
gz ground-truth и sensor EKF; их Δ = диагностика. Приёмка rl-lab (v2 §3):
кумулятивный интеграл cmd ≈ gt-траектория, дыры t > 0.5с внутри эпизода = FAIL.

ПОТОКИ (v2 §1 — 5 CSV на RUN; эпизодных файлов НЕТ, нарезка ТОЛЬКО по events §2):
    <out_root>/<run_id>/            run_id = <sprint>/<model>/<session> («/» → подкаталоги)
        gt.csv      t,x,y,z,yaw     gz-истина. Источник: /world/<world>/pose/info
                                    (PoseArray без имён → дрон = ближайшая к EKF-позе
                                    в 3D; XY-матч ловил пол — z-бимодальность, см. m1c).
                                    Строже рекомендации v2 (wall_gt+gz z): ВСЕ координаты
                                    из физдвижка. Колонки v2 — совпадают.
        odom.csv    t,x,y,z,yaw     sensor EKF: /mavros/local_position/pose (v2: z отсюда
                                    = канал alt-капа)
        cmd.csv     t,vx_w,vy_w,yaw_cmd
                                    ЭХО /krot/cmd_vel_world (Twist, WORLD-frame, НЕ body
                                    v4!) + последний /krot/yaw_cmd (латч; пусто → nan);
                                    строка на каждое сообщение любого из двух топиков.
        tof.csv     t,ch0..ch5      /krot/tof_ring (канон-порядок 0..300°, клип 1.2)
        events.csv  t,event,value   словарь v2 §2: run_meta (JSON, 1-я строка) ·
                                    episode_reset ("x,y,theta") · episode_ready (1/0) ·
                                    collision (1, только фронт) · guard_verdict (String
                                    /krot/guard_verdict — топик rl-lab, появится при
                                    интеграции guard; молчание — норма)

ВРЕМЯ: все t = sim-time — топики /krot/* без header, штампуем на приёме часами ноды;
launch ставит use_sim_time:=true (/clock бриджится базовым стеком drone.launch).
Flush ≥1/сек (v2: запись живёт до последней секунды перед смертью процесса).

УПРАВЛЕНИЕ: параметр enable (true в krot-профиле) + топик /krot/recorder/enable
(Bool; false = пауза записи, файлы флашатся).

Запуск: включён в krot_bridges.launch.py (record:=true). Отдельно:
    ros2 run drone_sim flight_recorder --ros-args -p run_id:=otladka/v5-p4b/train01
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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose2D, PoseArray, PoseStamped, Twist
from std_msgs.msg import Bool, Float32MultiArray, Float64, String

STREAMS = {
    "gt": ["t", "x", "y", "z", "yaw"],
    "odom": ["t", "x", "y", "z", "yaw"],
    "cmd": ["t", "vx_w", "vy_w", "yaw_cmd"],
    "tof": ["t", "ch0", "ch1", "ch2", "ch3", "ch4", "ch5"],
    "events": ["t", "event", "value"],
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
        p("run_id", "")                 # ""=<дата-время>; v2: <sprint>/<model>/<session>
        p("world", "")                  # ""=env DEFAULT_WORLD → автодетект gz topic -l
        p("model", "")                  # id модели/политики — в run_meta (v2 §2)
        p("hold_alt_m", 0.0)            # эшелон рана — в run_meta (0.0 = не передан)
        p("enable", True)               # старт включённым (krot-профиль); runtime-гейт топиком
        p("flush_period_s", 1.0)        # v2: flush ≥1/сек (crash-защита записи)
        gp = lambda n: self.get_parameter(n).value

        self._enabled = bool(gp("enable"))
        world = str(gp("world")) or os.environ.get("DEFAULT_WORLD", "") or _detect_world()
        self._world = world
        run_id = str(gp("run_id")) or time.strftime("%Y%m%d_%H%M%S")
        root = os.path.join(str(gp("out_root")), run_id)
        base, n = root, 1
        while os.path.exists(root):    # уникальный каталог = нет конкурентной записи в CSV
            root = f"{base}_{n}"
            n += 1
        self._root = root
        os.makedirs(self._root, exist_ok=True)

        self._files: dict[str, object] = {}
        self._writers: dict[str, csv.writer] = {}
        self._counts: dict[str, int] = {}
        for name, header in STREAMS.items():
            f = open(os.path.join(self._root, f"{name}.csv"), "w", newline="")
            w = csv.writer(f)
            w.writerow(header)
            self._files[name], self._writers[name] = f, w
            self._counts[name] = 0

        self._last_ekf: tuple[float, float, float] | None = None
        self._vx = self._vy = 0.0
        self._yaw_cmd: float | None = None
        self._collision = False
        self._ep = 0

        # run_meta — 1-я строка events.csv (v2 §2)
        self._row("events", [f"{self._t():.3f}", "run_meta", json.dumps(
            {"model": str(gp("model")), "world": world,
             "hold_alt": float(gp("hold_alt_m")),
             "run_id": run_id, "format": "csv_record_format_v2"}, ensure_ascii=False)])

        sq = qos_profile_sensor_data
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._pose_cb, sq)
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
        # guard_verdict — будущий паблишер rl-lab (gz_env/guard); молчание топика — норма (v2 §2)
        self.create_subscription(String, "/krot/guard_verdict", self._guard_cb, 10)
        self.create_subscription(Bool, "/krot/recorder/enable", self._enable_cb, 10)

        self.create_timer(float(gp("flush_period_s")), self._flush)
        self.get_logger().info(
            f"flight_recorder (формат v2): → {self._root} (world={world or '∅'}, "
            f"enable={self._enabled}; нарезка эпизодов — по events)")

    # ── файлы ──
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
        if not self._enabled:
            return
        self._writers[stream].writerow(row)
        self._counts[stream] += 1

    # ── callbacks ──
    def _pose_cb(self, m: PoseStamped) -> None:
        pp, q = m.pose.position, m.pose.orientation
        self._last_ekf = (pp.x, pp.y, pp.z)
        self._row("odom", [f"{self._t():.3f}", f"{pp.x:.4f}", f"{pp.y:.4f}", f"{pp.z:.4f}",
                           f"{_yaw_from_quat(q.x, q.y, q.z, q.w):.4f}"])

    def _gt_cb(self, m: PoseArray) -> None:
        # PoseArray без имён → дрон = ближайшая к EKF-позе в 3D (XY-матч ловил пол: m1c).
        if not m.poses or self._last_ekf is None:
            return
        rx, ry, rz = self._last_ekf
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
        yc = "nan" if self._yaw_cmd is None else f"{self._yaw_cmd:.4f}"
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
        if m.data and not self._collision:   # словарь v2: только фронт, value=1
            self._row("events", [f"{self._t():.3f}", "collision", "1"])
        self._collision = bool(m.data)

    def _reset_cb(self, m: Pose2D) -> None:
        self._ep += 1
        self._row("events", [f"{self._t():.3f}", "episode_reset",
                             f"{m.x:.3f},{m.y:.3f},{m.theta:.3f}"])
        self.get_logger().info(f"эпизод #{self._ep} (reset {m.x:.2f},{m.y:.2f})")

    def _ready_cb(self, m: Bool) -> None:
        self._row("events", [f"{self._t():.3f}", "episode_ready", "1" if m.data else "0"])

    def _guard_cb(self, m: String) -> None:
        self._row("events", [f"{self._t():.3f}", "guard_verdict", m.data])

    def _enable_cb(self, m: Bool) -> None:
        if bool(m.data) != self._enabled:
            self._enabled = bool(m.data)
            self._flush()
            self.get_logger().info(f"запись {'ВКЛ' if m.data else 'ПАУЗА (flush)'}")

    def close(self) -> None:
        for f in self._files.values():
            try:
                f.close()
            except OSError:
                pass
        if self._files:
            self.get_logger().info(
                "закрыто: " + " ".join(f"{k}={self._counts[k]}" for k in STREAMS) +
                f" (эпизодов {self._ep})")
        self._files, self._writers = {}, {}


def main() -> None:
    rclpy.init()
    node = FlightRecorder()
    # SIGTERM (timeout/tmux kill — штатный стоп) → graceful close (финальный flush+лог)
    def _sig(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sig)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass            # внешний Ctrl-C/launch-shutdown — штатно; CSV закрываем ниже
    finally:
        node.close()    # close НЕ зависит от rclpy-контекста (чистый file I/O)
        try:
            rclpy.shutdown()
        except Exception:
            pass        # контекст уже закрыт внешним shutdown'ом (rcl_shutdown already called)


if __name__ == "__main__":
    main()
