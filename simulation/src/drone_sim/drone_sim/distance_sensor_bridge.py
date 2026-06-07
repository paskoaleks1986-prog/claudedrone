#!/usr/bin/env python3
"""distance_sensor_bridge — ToF показания → FC через MAVLink DISTANCE_SENSOR.

Groundwork для железа (backlog Block 3, 2026-06-07). В GUIDED с position
setpoints на навигацию НЕ влияет (AC_Avoidance не работает с position
setpoints — ресёрч Block 3); нужно для будущих Loiter/AltHold на железе:

    6×VL53L0X (горизонт)  → DISTANCE_SENSOR id 1-6, yaw-ориентации →
                            AP Proximity MAV-драйвер (PRX1_TYPE=2 MAV)
    TF-Luna down (высота) → DISTANCE_SENSOR id 7, PITCH_270 →
                            AP rangefinder (RNGFND1_TYPE=10 MAVLink)

Путь: /drone/perimeter + /drone/altitude → sensor_msgs/Range на
/mavros/distance_sensor/<name> → mavros distance_sensor plugin (subscriber
блоки в config/mavros/apm_config_claudedrone.yaml) → MAVLink → FC.

Ориентации body frame — из bridge_map_protocol.md §6 (каналы 0..5 на
0/60/.../300°), квантованы в 45°-сетку MAV_SENSOR_ORIENTATION (ближайший
yaw; ошибка ≤15° — для PRX-секторов AP это допустимо, сектор 45°).
Sweep TF-Luna (серво) НЕ шлём: у DISTANCE_SENSOR статическая ориентация
на сенсор; вращающийся луч — это OBSTACLE_DISTANCE (future work).

НЕ в RL-пайплайне: нода опциональная, RL-обвязка её не требует.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Float32MultiArray

# §6 протокола: физические капы сенсоров
VL_MIN_RANGE_M = 0.02
VL_MAX_RANGE_M = 2.0
TF_MIN_RANGE_M = 0.2
TF_MAX_RANGE_M = 8.0
VL_FOV_RAD = 0.44   # VL53L0X ~25°
TF_FOV_RAD = 0.04   # TF-Luna ~2°

# Канал (0..5, body 0/60/.../300°) → имя mavros-блока. Ориентация задаётся
# в apm_config_claudedrone.yaml (NONE/YAW_45/.../YAW_315), здесь только тракт.
VL_TOPIC_FMT = "/mavros/distance_sensor/vl53_ch{ch}"
TF_DOWN_TOPIC = "/mavros/distance_sensor/tf_luna_down"

PUBLISH_RATE_HZ = 10.0


def make_range(
    reading_m: float,
    min_range_m: float,
    max_range_m: float,
    fov_rad: float,
) -> Range | None:
    """Range-сообщение из чтения; None = не публиковать.

    inf/NaN (stale-канал sensor_monitor либо no-hit) → None: FC не получает
    устаревших/пустых чтений — отсутствие сообщения честнее, чем кэш
    (паритет с фиксом stale 4c283e6). Конечные значения клампятся в
    [min, max] (MAVLink DISTANCE_SENSOR в см, uint16).
    """
    if not math.isfinite(reading_m):
        return None
    msg = Range()
    msg.radiation_type = Range.INFRARED  # ToF
    msg.field_of_view = fov_rad
    msg.min_range = min_range_m
    msg.max_range = max_range_m
    msg.range = min(max(reading_m, min_range_m), max_range_m)
    return msg


class DistanceSensorBridge(Node):
    def __init__(self) -> None:
        super().__init__("distance_sensor_bridge")
        self._vl = [float("inf")] * 6
        self._alt = float("inf")

        self.create_subscription(
            Float32MultiArray, "/drone/perimeter", self._perimeter_cb, 10
        )
        self.create_subscription(
            Float32, "/drone/altitude", self._altitude_cb, 10
        )

        self._vl_pubs = [
            self.create_publisher(Range, VL_TOPIC_FMT.format(ch=ch), 10)
            for ch in range(6)
        ]
        self._tf_pub = self.create_publisher(Range, TF_DOWN_TOPIC, 10)

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)
        self.get_logger().info(
            "distance_sensor_bridge: 6×VL53 (PRX yaw) + TF-Luna down "
            "(PITCH_270) → /mavros/distance_sensor/* @ 10 Hz"
        )

    def _perimeter_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 6:
            self._vl = [float(d) for d in msg.data[:6]]

    def _altitude_cb(self, msg: Float32) -> None:
        self._alt = float(msg.data)

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        for ch, pub in enumerate(self._vl_pubs):
            msg = make_range(
                self._vl[ch], VL_MIN_RANGE_M, VL_MAX_RANGE_M, VL_FOV_RAD
            )
            if msg is not None:
                msg.header.stamp = now
                msg.header.frame_id = f"vl53_ch{ch}"
                pub.publish(msg)
        msg = make_range(self._alt, TF_MIN_RANGE_M, TF_MAX_RANGE_M, TF_FOV_RAD)
        if msg is not None:
            msg.header.stamp = now
            msg.header.frame_id = "tf_luna_down"
            self._tf_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DistanceSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
