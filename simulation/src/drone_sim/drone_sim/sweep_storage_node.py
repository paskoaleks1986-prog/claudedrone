"""sweep_storage_node — TASK-047 prototype: servo sweep + scan storage.

Реализация H1 (Triangular sweep + LaserScan aggregation) из dev-log/14
(`docs/dev-log/14-task047-sweep-storage-research.md`).

Контракт от researchbest's mission_fsm_node + sim's stop_scan FSM:
    subs:  /drone/sweep/start (Empty)   — старт sweep cycle
           /drone/sweep/stop  (Empty)   — прервать
           /scan/sweep (LaserScan)      — TF-Luna single ray @10 Hz (gz_bridge)
    pubs:  /drone/sg90/cmd (Float64)    — target angle 0..π
           /drone/sweep/result          — aggregated cycle (LaserScan or PointCloud2)
           /scan/status (String)        — SCANNING/COMPLETE/STOPPED

θ(t) triangular: 0 → π → 0 за cycle_period_s.

NPZ dump: $RESEARCHBEST_ROOT/output_data/TASK-047/sweep_<ts>.npz
    (theta_target, range_m, t_relative_s, cycle_period_s, frame_id)
"""

from __future__ import annotations

import math
import os
import time
from collections import deque
from enum import Enum, auto
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Empty, Float64, String


class State(Enum):
    IDLE = auto()
    SWEEPING = auto()


class SweepStorageNode(Node):
    def __init__(self) -> None:
        super().__init__("sweep_storage_node")

        # --- params ---
        self.declare_parameter("cycle_period_s", 9.0)
        self.declare_parameter("sweep_min_rad", 0.0)
        self.declare_parameter("sweep_max_rad", math.pi)
        self.declare_parameter("cmd_rate_hz", 50.0)
        self.declare_parameter("output_mode", "laserscan")  # laserscan | pointcloud2
        self.declare_parameter("output_topic_laserscan", "/drone/sweep/result")
        self.declare_parameter("output_topic_pointcloud2", "/drone/sweep/cloud")
        self.declare_parameter("status_topic", "/scan/status")
        self.declare_parameter("sweep_in_topic", "/scan/sweep")
        self.declare_parameter("cmd_topic", "/drone/sg90/cmd")
        self.declare_parameter("start_topic", "/drone/sweep/start")
        self.declare_parameter("stop_topic", "/drone/sweep/stop")
        self.declare_parameter("frame_id", "sg90_base")
        self.declare_parameter("n_bins", 90)
        self.declare_parameter("range_min", 0.2)
        self.declare_parameter("range_max", 8.0)
        self.declare_parameter("dump_npz", True)
        default_dump = os.path.join(
            os.environ.get("RESEARCHBEST_ROOT", str(Path.home() / "researchbest")),
            "output_data",
            "TASK-047",
        )
        self.declare_parameter("dump_dir", default_dump)

        p = self.get_parameter
        self.cycle_period = float(p("cycle_period_s").value)
        self.sweep_min = float(p("sweep_min_rad").value)
        self.sweep_max = float(p("sweep_max_rad").value)
        self.cmd_rate = float(p("cmd_rate_hz").value)
        self.output_mode = str(p("output_mode").value).lower()
        self.frame_id = str(p("frame_id").value)
        self.n_bins = int(p("n_bins").value)
        self.range_min = float(p("range_min").value)
        self.range_max = float(p("range_max").value)
        self.dump_npz = bool(p("dump_npz").value)
        self.dump_dir = Path(str(p("dump_dir").value))

        if self.output_mode not in {"laserscan", "pointcloud2"}:
            self.get_logger().warn(
                f"output_mode={self.output_mode} unknown, falling back to 'laserscan'"
            )
            self.output_mode = "laserscan"

        # --- state ---
        self.state = State.IDLE
        self.t_start: float | None = None
        self.samples: deque[tuple[float, float, float]] = deque()  # (t_rel, theta, range)

        # --- pubs ---
        self.pub_cmd = self.create_publisher(Float64, str(p("cmd_topic").value), 10)
        self.pub_status = self.create_publisher(String, str(p("status_topic").value), 10)
        if self.output_mode == "laserscan":
            self.pub_out: rclpy.publisher.Publisher = self.create_publisher(
                LaserScan, str(p("output_topic_laserscan").value), 10
            )
        else:
            self.pub_out = self.create_publisher(
                PointCloud2, str(p("output_topic_pointcloud2").value), 10
            )

        # --- subs ---
        self.create_subscription(
            LaserScan, str(p("sweep_in_topic").value),
            self._on_sweep_sample, qos_profile_sensor_data,
        )
        self.create_subscription(Empty, str(p("start_topic").value), self._on_start, 10)
        self.create_subscription(Empty, str(p("stop_topic").value), self._on_stop, 10)

        # --- cmd timer ---
        self.cmd_timer = self.create_timer(1.0 / self.cmd_rate, self._on_cmd_tick)

        self.get_logger().info(
            f"sweep_storage_node ready — cycle={self.cycle_period}s, "
            f"sweep=[{self.sweep_min:.2f},{self.sweep_max:.2f}]rad, "
            f"output_mode={self.output_mode}, n_bins={self.n_bins}, "
            f"dump_npz={self.dump_npz} → {self.dump_dir}"
        )

    # ---- callbacks ----

    def _on_start(self, _msg: Empty) -> None:
        if self.state == State.SWEEPING:
            self.get_logger().warn("/drone/sweep/start while SWEEPING — ignoring")
            return
        self.t_start = self._now()
        self.samples.clear()
        self.state = State.SWEEPING
        self._publish_status("SCANNING")
        self.get_logger().info(f"sweep START @ t={self.t_start:.3f}")

    def _on_stop(self, _msg: Empty) -> None:
        if self.state != State.SWEEPING:
            self.get_logger().info("/drone/sweep/stop while IDLE — noop")
            return
        self.state = State.IDLE
        self.t_start = None
        self._publish_status("STOPPED")
        # park servo back to min
        self._publish_cmd(self.sweep_min)
        self.get_logger().info("sweep STOPPED")

    def _on_sweep_sample(self, msg: LaserScan) -> None:
        if self.state != State.SWEEPING or self.t_start is None:
            return
        if not msg.ranges:
            return
        r = float(msg.ranges[0])
        # tf_luna_sweep single ray; out-of-range → inf
        if not math.isfinite(r) or r < self.range_min or r > self.range_max:
            r = math.inf
        t_rel = self._now() - self.t_start
        theta = self._theta_target(t_rel)
        self.samples.append((t_rel, theta, r))

    def _on_cmd_tick(self) -> None:
        if self.state != State.SWEEPING or self.t_start is None:
            return
        t_rel = self._now() - self.t_start
        if t_rel >= self.cycle_period:
            self._finish_sweep()
            return
        theta = self._theta_target(t_rel)
        self._publish_cmd(theta)

    # ---- core ----

    def _theta_target(self, t_rel: float) -> float:
        half = self.cycle_period / 2.0
        span = self.sweep_max - self.sweep_min
        if t_rel < half:
            return self.sweep_min + span * (t_rel / half)
        return self.sweep_max - span * ((t_rel - half) / half)

    def _finish_sweep(self) -> None:
        self.get_logger().info(
            f"sweep COMPLETE — samples={len(self.samples)}, "
            f"cycle_actual={self._now() - (self.t_start or 0):.2f}s"
        )
        # park servo
        self._publish_cmd(self.sweep_min)

        if self.samples:
            theta_arr = np.array([s[1] for s in self.samples], dtype=np.float32)
            range_arr = np.array([s[2] for s in self.samples], dtype=np.float32)
            t_arr = np.array([s[0] for s in self.samples], dtype=np.float32)

            if self.output_mode == "laserscan":
                msg = self._pack_laserscan(theta_arr, range_arr)
            else:
                msg = self._pack_pointcloud2(theta_arr, range_arr)
            self.pub_out.publish(msg)

            if self.dump_npz:
                self._dump_npz(theta_arr, range_arr, t_arr)
        else:
            self.get_logger().warn("sweep cycle ended with 0 samples (gz_bridge stuck?)")

        self.state = State.IDLE
        self.t_start = None
        self._publish_status("COMPLETE")

    def _pack_laserscan(self, theta: np.ndarray, ranges: np.ndarray) -> LaserScan:
        n = self.n_bins
        angle_increment = (self.sweep_max - self.sweep_min) / n
        # bin index per sample (clamp)
        bin_idx = np.clip(
            ((theta - self.sweep_min) / angle_increment).astype(int), 0, n - 1
        )
        out = np.full(n, math.inf, dtype=np.float32)
        counts = np.zeros(n, dtype=np.int32)
        for i in range(len(theta)):
            r = float(ranges[i])
            if not math.isfinite(r):
                continue
            b = int(bin_idx[i])
            if counts[b] == 0:
                out[b] = r
            else:
                out[b] = (out[b] * counts[b] + r) / (counts[b] + 1)
            counts[b] += 1

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = float(self.sweep_min)
        msg.angle_max = float(self.sweep_max)
        msg.angle_increment = float(angle_increment)
        msg.time_increment = float(self.cycle_period / n) if n > 0 else 0.0
        msg.scan_time = float(self.cycle_period)
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = out.tolist()
        msg.intensities = []
        return msg

    def _pack_pointcloud2(self, theta: np.ndarray, ranges: np.ndarray) -> PointCloud2:
        mask = np.isfinite(ranges)
        x = (ranges * np.cos(theta))[mask].astype(np.float32)
        y = (ranges * np.sin(theta))[mask].astype(np.float32)
        z = np.zeros_like(x, dtype=np.float32)
        pts = np.stack([x, y, z], axis=1)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = int(pts.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = pts.tobytes()
        msg.is_dense = True
        return msg

    def _dump_npz(self, theta: np.ndarray, ranges: np.ndarray, t_rel: np.ndarray) -> None:
        try:
            self.dump_dir.mkdir(parents=True, exist_ok=True)
            ts = int(time.time())
            path = self.dump_dir / f"sweep_{ts}.npz"
            np.savez(
                path,
                theta_target=theta,
                range_m=ranges,
                t_relative_s=t_rel,
                cycle_period_s=np.float32(self.cycle_period),
                frame_id=self.frame_id,
            )
            self.get_logger().info(f"NPZ dump → {path}")
        except OSError as e:
            self.get_logger().warn(f"NPZ dump failed: {e}")

    # ---- helpers ----

    def _publish_cmd(self, theta: float) -> None:
        msg = Float64()
        msg.data = float(theta)
        self.pub_cmd.publish(msg)

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.pub_status.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SweepStorageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
