#!/usr/bin/env python3
"""chain_monitor.py — сторож потоков данных input/output цепочек (v2, ночные раны).

Директива Aleks 2026-06-06: «во время тестов трекай потоки данных и
output/input чейны — важно, чтобы на этом уровне не было сбоев».

INPUT chain  : gz сенсоры → bridge → sensor_monitor → obs
    /drone/vl53l0x/ch0 · /scan/sweep · /drone/perimeter ·
    /mavros/local_position/odom
OUTPUT chain : policy → executor → MAVROS → SITL
    /mavros/setpoint_position/local · /rl_policy/coverage (1 msg = 1 step) ·
    /drone/sg90/cmd
STATE        : /mavros/state (connected/armed/mode)

Каждые WINDOW_S печатает строку со всеми частотами; WARN при rate ниже
порога, ERROR при disconnected/disarmed/нулевом setpoint-стриме (GUIDED
без стрима = неуправляемый дрон). Stdout → tee в лог tmux-окна.

Запуск (ночной ран):
    ros2 run / python3 help_scripts/chain_monitor.py
"""
from __future__ import annotations

import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32, Float64
from mavros_msgs.msg import State

WINDOW_S = 30.0

# topic → (type, qos, warn_below_hz, chain)
WATCH = {
    "/drone/vl53l0x/ch0": (LaserScan, 10, 5.0, "IN "),
    "/scan/sweep": (LaserScan, 10, 5.0, "IN "),
    "/drone/perimeter": (Float32MultiArray, 10, 5.0, "IN "),
    "/mavros/local_position/odom": (Odometry, "sensor", 4.0, "IN "),
    "/mavros/setpoint_position/local": (PoseStamped, 10, 5.0, "OUT"),
    "/rl_policy/coverage": (Float32, 10, 0.0, "OUT"),   # 1 msg = 1 policy step
    "/drone/sg90/cmd": (Float64, 10, 0.0, "OUT"),       # событийный, без порога
}
STEP_STALL_S = 300.0  # нет policy-шагов дольше → WARN


class ChainMonitor(Node):
    def __init__(self) -> None:
        super().__init__("chain_monitor")
        self.counts: dict[str, int] = defaultdict(int)
        self.last_cov = float("nan")
        self.last_step_mono = time.monotonic()
        self.mav_connected = False
        self.mav_armed = False
        self.mav_mode = "?"

        for topic, (mtype, qos, _, _) in WATCH.items():
            q = qos_profile_sensor_data if qos == "sensor" else qos
            self.create_subscription(
                mtype, topic,
                (lambda m, t=topic: self._count(t, m)), q)
        self.create_subscription(State, "/mavros/state", self._state, 10)
        self.create_timer(WINDOW_S, self._report)
        self.get_logger().info(
            f"chain_monitor: окно {WINDOW_S:.0f}s, step-stall {STEP_STALL_S:.0f}s")

    def _count(self, topic: str, msg) -> None:
        self.counts[topic] += 1
        if topic == "/rl_policy/coverage":
            self.last_cov = float(msg.data)
            self.last_step_mono = time.monotonic()

    def _state(self, m: State) -> None:
        self.mav_connected = m.connected
        self.mav_armed = m.armed
        self.mav_mode = m.mode

    def _report(self) -> None:
        parts, problems = [], []
        for topic, (_, _, warn_hz, chain) in WATCH.items():
            hz = self.counts[topic] / WINDOW_S
            short = topic.rsplit("/", 1)[-1] or topic
            parts.append(f"{chain.strip()}:{short}={hz:.1f}Hz")
            if warn_hz > 0 and hz < warn_hz:
                problems.append(f"{topic} {hz:.1f}Hz < {warn_hz}")
            self.counts[topic] = 0

        step_age = time.monotonic() - self.last_step_mono
        line = (f"cov={self.last_cov:.3f} step_age={step_age:.0f}s "
                f"mavros={'CONN' if self.mav_connected else 'DISC'}/"
                f"{'ARM' if self.mav_armed else 'DISARM'}/{self.mav_mode} "
                + " ".join(parts))

        if not self.mav_connected:
            self.get_logger().error(f"CHAIN BROKEN: MAVROS disconnected · {line}")
        elif not self.mav_armed:
            self.get_logger().error(f"CHAIN: DISARMED (crash/land?) · {line}")
        elif problems:
            self.get_logger().warning(f"CHAIN DEGRADED: {'; '.join(problems)} · {line}")
        elif step_age > STEP_STALL_S:
            self.get_logger().warning(f"CHAIN: policy steps stalled {step_age:.0f}s · {line}")
        else:
            self.get_logger().info(f"CHAIN OK · {line}")


def main() -> None:
    rclpy.init()
    node = ChainMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
