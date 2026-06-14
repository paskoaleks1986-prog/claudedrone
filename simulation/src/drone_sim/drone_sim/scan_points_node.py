#!/usr/bin/env python3
"""scan_points_node — C1: реальные точки скана с живого стека → gen_0 `scan`-записи.

Контракт: `claudedrone-git/docs/c1_scan_points_source.md` (sim-сторона) под схему
rl-lab `point_scan_generation_contract.md`. Конвертит завершённый sweep-проход (или
одиночный precise-выстрел) в `scan`-запись формата flight_log/scan_store: точки в
world-координатах (вычислены в ДРЕЙФЕ по odom), `from_pose` ДВОЙНОЙ (odom + gt),
`bearing` body-frame на КАЖДОЙ точке, `origin` fan/precise.

Поток:
  fan:     /drone/sweep/result (LaserScan, ranges[i] @ θ=i·angle_increment) ─┐
  precise: /drone/scan/precise (Empty) + /scan/sweep (ranges[0]) + servo θ ──┤
                                                                              ▼
   собрать from_pose: odom (/mavros/local_position/odom = дрейф)
                      gt   (/world/<world>/pose/info PoseArray = Gazebo-факт,
                            дрон = запись ближайшая к odom XY — стены статичны на ±6м)
   геометрия (model.sdf): bearing_body = θ_servo − π/2  (θ=π/2 → нос, [0,π]→[−π/2,+π/2])
                          точка_world = odom_xy + (MOUNT_FWD+range)·(cos,sin)(yaw+bearing)
                                        z = odom_z + MOUNT_Z   (свип z-плоскостной)
   эмит: /drone/scan/record (std_msgs/String = JSON одной `scan`-записи)
         + опц. аппенд в jsonl_path (standalone). Writer flight_log.jsonl на живом
         стеке — сторона interface (слушает record-топик); см. контракт §5.

⚠ Геометрия (знак/zero bearing) выведена из SDF, но требует Gazebo stand-verify перед
закрытием C1 (правило: SDF/sim-таск без смоука не закрывается). См. контракт §5.

Запуск: ros2 run drone_sim scan_points --ros-args -p world:=base_stand_12x12 \
        -p use_sim_time:=true [-p jsonl_path:=/tmp/scan_store.jsonl]
"""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float64, String

# Геометрия серво/луча (iris_claudedrone/model.sdf: sg90_arm yaw −π/2, ось Z, θ∈[0,π];
# tf_luna_sweep вдоль +x руки). bearing_body = θ_servo − π/2 (REP-103 body: x=нос, y=влево).
SERVO_ZERO_OFFSET_RAD = math.pi / 2.0   # θ=π/2 → нос (bearing_body=0)
MOUNT_FWD_M = 0.025                     # сенсор вдоль луча от центра дрона (XY)
MOUNT_Z_M = 0.151                       # высота сенсора над центром (0.146 рука + 0.005)


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """ENU yaw из кватерниона (поворот вокруг Z)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class ScanPointsNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_points_node")
        p = self.declare_parameter
        world = str(p("world", "base_stand_12x12").value)
        self.result_topic = str(p("result_topic", "/drone/sweep/result").value)
        self.sweep_topic = str(p("sweep_topic", "/scan/sweep").value)
        self.precise_topic = str(p("precise_topic", "/drone/scan/precise").value)
        self.odom_topic = str(p("odom_topic", "/mavros/local_position/odom").value)
        self.servo_topic = str(p("servo_topic", "/drone/sg90/cmd").value)
        # /scan/record (согласовано с interface 2026-06-14, parity §6): interface
        # control_api (scan_record_node) подписан сюда → пишет flight_log.jsonl.
        self.record_topic = str(p("record_topic", "/scan/record").value)
        self.pose_info_topic = str(
            p("pose_info_topic", f"/world/{world}/pose/info").value
        )
        self.jsonl_path = str(p("jsonl_path", "").value)  # '' → файл не пишем
        self.range_min = float(p("range_min", 0.2).value)
        self.range_max = float(p("range_max", 8.0).value)
        # порог «это дрон»: gt-запись PoseArray ближе порога к odom XY (дрейф мал, стены далеко)
        self.gt_match_max_m = float(p("gt_match_max_m", 1.5).value)
        self.run_id = str(p("run_id", "live").value)

        # последнее состояние (кэш для сборки from_pose на момент скана)
        self._odom: dict | None = None      # {x,y,z,yaw,t}
        self._poses: list | None = None     # PoseArray.poses
        self._servo_rad: float = SERVO_ZERO_OFFSET_RAD  # последний угол серво
        self._scan_seq = 0
        self._meta_written = False

        self.create_subscription(Odometry, self.odom_topic, self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseArray, self.pose_info_topic, self._on_poses, 10)
        self.create_subscription(Float64, self.servo_topic, self._on_servo, 10)
        self.create_subscription(LaserScan, self.result_topic, self._on_result, 10)
        self.create_subscription(LaserScan, self.sweep_topic, self._on_sweep,
                                 qos_profile_sensor_data)
        self.create_subscription(Empty, self.precise_topic, self._on_precise, 1)

        self.pub_record = self.create_publisher(String, self.record_topic, 10)
        self._last_sweep_range: float | None = None
        # meta (self-describing mount) re-публикуется по таймеру: топик НЕ latched, а
        # interface-writer может подписаться ПОЗЖЕ первого скана → иначе пропустит meta.
        # Идемпотентно (одинаковая meta); writer дедупит/игнорит повтор. 2026-06-14 hand-over.
        self._meta: dict | None = None
        self.create_timer(5.0, self._republish_meta)

        self.get_logger().info(
            f"scan_points: result←{self.result_topic} precise←{self.precise_topic} "
            f"odom←{self.odom_topic} gt←{self.pose_info_topic} → {self.record_topic}"
            + (f" + jsonl {self.jsonl_path}" if self.jsonl_path else "")
        )

    # ---- кэш входов ----

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        self._odom = {
            "x": float(pos.x), "y": float(pos.y), "z": float(pos.z),
            "yaw": _yaw_from_quat(q.x, q.y, q.z, q.w),
            "t": self._now(),
        }

    def _on_poses(self, msg: PoseArray) -> None:
        self._poses = list(msg.poses)

    def _on_servo(self, msg: Float64) -> None:
        self._servo_rad = float(msg.data)

    def _on_sweep(self, msg: LaserScan) -> None:
        if msg.ranges:
            r = float(msg.ranges[0])
            self._last_sweep_range = r if math.isfinite(r) else None

    # ---- gt-поза: ближайшая к odom запись PoseArray (стены статичны и далеко) ----

    def _gt_pose(self, odom: dict) -> dict | None:
        # ⚠ /world/<w>/pose/info = PoseArray БЕЗ имён, содержит корни моделей И
        # саб-линки. У дрона много линков на XY≈(0,0) (sg90, ротеры) → XY-nearest
        # неоднозначен (ничья в начале координат). Матчим по 3D (с Z): корень дрона
        # на z≈odom.z, саб-линки на иных Z дальше. Подтверждено stand-verify 2026-06-14
        # (base_stand: корень [16] z=0.22 vs саб-линки z=0.02/0.10/0.13/0.15).
        if not self._poses:
            return None
        best = None
        best_d = self.gt_match_max_m
        for ps in self._poses:
            dx = ps.position.x - odom["x"]
            dy = ps.position.y - odom["y"]
            dz = ps.position.z - odom["z"]
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d <= best_d:
                best_d = d
                best = ps
        if best is None:
            return None
        q = best.orientation
        return {
            "x": float(best.position.x), "y": float(best.position.y),
            "z": float(best.position.z),
            "yaw": _yaw_from_quat(q.x, q.y, q.z, q.w),
        }

    # ---- конвертация (servo θ, range) → world-точка по odom-дрейфу ----

    def _world_point(self, odom: dict, bearing_body: float, rng: float) -> tuple:
        phi = odom["yaw"] + bearing_body
        d = MOUNT_FWD_M + rng
        return (
            odom["x"] + d * math.cos(phi),
            odom["y"] + d * math.sin(phi),
            odom["z"] + MOUNT_Z_M,
        )

    # ---- сборка и эмит scan-записи ----

    def _publish_record(self, record: dict) -> None:
        """Эмит одной записи: на топик /scan/record (interface-writer) + опц. в jsonl."""
        self.pub_record.publish(String(data=json.dumps(record, ensure_ascii=False)))
        self._append_jsonl(record)

    def _republish_meta(self) -> None:
        """Периодический re-emit meta (для late-subscriber interface-writer)."""
        if self._meta is not None:
            self.pub_record.publish(String(data=json.dumps(self._meta, ensure_ascii=False)))

    def _emit(self, record: dict) -> None:
        # meta — ПЕРВОЙ записью (на топик И в jsonl), self-describing mount: interface
        # recover_range/true_point восстанавливает gt-точку без хардкода sim-констант.
        if not self._meta_written:
            self._meta = {
                "rec": "meta", "schema": "scan_store/1.0", "run_id": self.run_id,
                "world": self.pose_info_topic.split("/")[2] if "/world/" in self.pose_info_topic else "",
                "frame": {"world": "x_right_y_north_m", "angle": "rad_ccw"},
                "mount": {"fwd": MOUNT_FWD_M, "z": MOUNT_Z_M},
                "servo_zero_offset": SERVO_ZERO_OFFSET_RAD,  # bearing_body = θ_servo − offset
                "units": "m", "t0": self._now(),
            }
            self._publish_record(self._meta)
            self._meta_written = True
        self._publish_record(record)

    def _append_jsonl(self, record: dict) -> None:
        if not self.jsonl_path:
            return
        try:
            with open(self.jsonl_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError as e:
            self.get_logger().warn(f"jsonl append failed: {e}")

    def _build_scan(self, origin: str, servo_range: list,
                    samples: list) -> dict | None:
        """samples = [(θ_servo, range)]. Возвращает scan-запись или None если нет позы."""
        odom = self._odom
        if odom is None:
            self.get_logger().warn("нет odom — скан пропущен (поза неизвестна)")
            return None
        gt = self._gt_pose(odom)
        if gt is None:
            self.get_logger().warn(
                "gt-поза не найдена в pose/info (нет PoseArray / нет записи в пороге) — gt=null"
            )
        t = self._now()
        sid = f"s{self._scan_seq}"
        self._scan_seq += 1
        dots = []
        for i, (theta, rng) in enumerate(samples):
            if not math.isfinite(rng) or rng < self.range_min or rng > self.range_max:
                continue  # inf/вне диапазона = нет препятствия → точку не создаём
            bearing = theta - SERVO_ZERO_OFFSET_RAD
            x, y, z = self._world_point(odom, bearing, rng)
            dots.append({
                "id": f"{sid}d{i}", "scan_id": sid, "t": t, "gen": 0,
                "x": round(x, 4), "y": round(y, 4), "z": round(z, 4),
                "origin": origin, "bearing": round(bearing, 4),
                "trust": True, "cat": None, "source": "sensor",
            })
        return {
            "rec": "scan", "id": sid, "t": t, "origin": origin,
            "servo_range": servo_range,
            "from_pose": {
                "odom": {k: round(odom[k], 4) for k in ("x", "y", "z", "yaw")},
                "gt": ({k: round(gt[k], 4) for k in ("x", "y", "z", "yaw")}
                       if gt else None),
            },
            "dots": dots,
        }

    def _on_result(self, msg: LaserScan) -> None:
        """fan-проход завершён → scan-запись (точка на каждый валидный луч)."""
        samples = [
            (msg.angle_min + i * msg.angle_increment, float(r))
            for i, r in enumerate(msg.ranges)
        ]
        rec = self._build_scan(
            "fan", [round(msg.angle_min, 4), round(msg.angle_max, 4)], samples
        )
        if rec is not None:
            self._emit(rec)
            self.get_logger().info(
                f"fan scan {rec['id']}: {len(rec['dots'])} точек "
                f"(из {len(msg.ranges)} лучей), gt={'есть' if rec['from_pose']['gt'] else 'null'}"
            )

    def _on_precise(self, _msg: Empty) -> None:
        """precise-выстрел: текущий луч /scan/sweep @ текущем угле серво → 1 точка."""
        if self._last_sweep_range is None:
            self.get_logger().warn("precise: нет свежего /scan/sweep — пропуск")
            return
        theta = self._servo_rad
        rec = self._build_scan("precise", [round(theta, 4), round(theta, 4)],
                               [(theta, self._last_sweep_range)])
        if rec is not None:
            self._emit(rec)
            self.get_logger().info(
                f"precise scan {rec['id']}: θ={theta:.3f} r={self._last_sweep_range:.2f} "
                f"({len(rec['dots'])} точек)"
            )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ScanPointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
