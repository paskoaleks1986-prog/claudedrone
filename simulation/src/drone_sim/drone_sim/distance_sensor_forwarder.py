#!/usr/bin/env python3
"""distance_sensor_forwarder — сырые ToF-сканы Gazebo → MAVLink DISTANCE_SENSOR.

Стенд-спринт (Aleks 2026-06-08). Без активного forwarder'а AP с PRX1_TYPE=2 +
RNGFND1_TYPE=10 даёт `PreArm: No Data` и отказывает в арме (find: бенч-ран
стенда). Эта нода кормит mavros distance_sensor плагин, который форвардит
sensor_msgs/Range → MAVLink DISTANCE_SENSOR → FC.

Тракт:
    /drone/vl53l0x/ch{0..5} (LaserScan, gz)  → Range → /mavros/vl53_ch{0..5}
    /drone/tf_luna_down     (LaserScan, gz)  → Range → /mavros/tf_luna_down
    → mavros distance_sensor (config/mavros/apm_config_claudedrone.yaml:
      vl53_ch0..5 id 1-6 yaw-ориентации PRX1; tf_luna_down id 7 PITCH_270 RNGFND)
    → MAVLink → AP.

⚠ Имя выходного топика: mavros distance_sensor плагин (Jazzy 2.x) подписывается
на `~/<config-name>` СВОЕГО узла = `/mavros/vl53_ch0` (FC-smoke verified
2026-06-07, dev-log 24; `/mavros/distance_sensor/*` был паблишем в пустоту).
Параметр mavros_ns_prefix позволяет переопределить, если runtime покажет иначе.

⚠ no-hit (inf/range>max) → публикуем MAX_RANGE (сенсор жив, «чисто до макса»),
НЕ пропускаем: иначе в пустой зоне AP не получает сообщений → `PreArm: No Data`
не снимается. NaN → пропуск (битое чтение). Это и есть фикс prearm.

Сырые per-канал LaserScan (НЕ агрегированный /drone/perimeter): per-сенсор
свежесть, без зависимости от sensor_monitor (он капает inf→MAX, теряя «нет
данных»). Заменяет опциональный distance_sensor_bridge.py (тот читал агрегат).

НЕ в RL-пайплайне; в GUIDED с position-setpoints на навигацию не влияет
(AC_Avoidance не работает с position setpoints). Нужен для prearm-готовности
+ будущих AltHold/Loiter + SITLDroneEnv prearm-sequence.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range

# Спека Aleks (Стенд 2026-06-08) + indoor.parm RNGFND1_MIN/MAX 0.10/6.40:
VL_MIN_RANGE_M = 0.02
VL_MAX_RANGE_M = 1.2     # VL53L0X эфф. в indoor (obs clip 1.2); = max для PRX
VL_FOV_RAD = 0.44        # ~25°
TF_MIN_RANGE_M = 0.10
TF_MAX_RANGE_M = 6.40    # TF-Luna; = RNGFND1_MAX
TF_FOV_RAD = 0.04        # ~2°
RADIATION_INFRARED = Range.INFRARED   # =1; ToF (VL53/TF-Luna) — ИК, не ультразвук


class DistanceSensorForwarder(Node):
    def __init__(self) -> None:
        super().__init__("distance_sensor_forwarder")
        # FC-verified дефолт; переопределяемо если runtime покажет иначе.
        self.declare_parameter("vl_topic_fmt", "/mavros/vl53_ch{ch}")
        self.declare_parameter("tf_topic", "/mavros/tf_luna_down")
        vl_fmt = self.get_parameter("vl_topic_fmt").value
        tf_topic = self.get_parameter("tf_topic").value

        self._vl_pubs = []
        for ch in range(6):
            pub = self.create_publisher(Range, vl_fmt.format(ch=ch), 10)
            self._vl_pubs.append(pub)
            self.create_subscription(
                LaserScan, f"/drone/vl53l0x/ch{ch}",
                lambda msg, c=ch: self._vl_cb(msg, c), 10,
            )
        self._tf_pub = self.create_publisher(Range, tf_topic, 10)
        self.create_subscription(
            LaserScan, "/drone/tf_luna_down", self._tf_cb, 10
        )
        self.get_logger().info(
            f"distance_sensor_forwarder: 6×VL53 → {vl_fmt} (PRX) + "
            f"TF-Luna → {tf_topic} (RNGFND); no-hit→MAX, NaN→skip"
        )

    @staticmethod
    def _scan_range_m(msg: LaserScan) -> float | None:
        """ranges[0]; пусто/NaN → None (skip), inf/нет-хита → +inf (→MAX)."""
        if not msg.ranges:
            return None
        d = float(msg.ranges[0])
        if math.isnan(d):
            return None
        return d

    def _make_range(self, d_m: float, lo: float, hi: float, fov: float) -> Range:
        msg = Range()
        msg.radiation_type = RADIATION_INFRARED
        msg.field_of_view = float(fov)
        msg.min_range = float(lo)
        msg.max_range = float(hi)
        # no-hit (inf или > max) → MAX (сенсор жив, чисто). < min → min.
        if not math.isfinite(d_m) or d_m > hi:
            d_m = hi
        elif d_m < lo:
            d_m = lo
        msg.range = float(d_m)
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _vl_cb(self, msg: LaserScan, ch: int) -> None:
        d = self._scan_range_m(msg)
        if d is None:
            return
        out = self._make_range(d, VL_MIN_RANGE_M, VL_MAX_RANGE_M, VL_FOV_RAD)
        out.header.frame_id = f"vl53_ch{ch}"
        self._vl_pubs[ch].publish(out)

    def _tf_cb(self, msg: LaserScan) -> None:
        d = self._scan_range_m(msg)
        if d is None:
            return
        out = self._make_range(d, TF_MIN_RANGE_M, TF_MAX_RANGE_M, TF_FOV_RAD)
        out.header.frame_id = "tf_luna_down"
        self._tf_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = DistanceSensorForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
