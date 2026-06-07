#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Float32

CRITICAL = 0.30
HIGH = 0.80
VL_MAX_RANGE_M = 2.0   # VL53L0X физический max range; bridge нормализует к 1.2 m,
                        # но physical max = 2.0 m (gazebo plugin range_max)
TF_LUNA_MAX_RANGE_M = 8.0  # TF-Luna physical max range

# v2 (2026-06-07, backlog): канал, молчащий дольше этого при штатных 10 Hz,
# считается мёртвым. Раньше 10 Hz-таймер вечно республиковал последний кэш —
# freshness-гейт bridge видел «свежие» данные и дрон летел с замороженными
# показаниями. Теперь stale-канал публикуется как inf («нет чтения», даунстрим
# обрабатывает по TASK-059 #5: cap на max = no-obstacle, без NaN/0.0).
STALE_TIMEOUT_S = 0.5

DIRECTIONS = [
    'Перед', 'Перед-право', 'Зад-право',
    'Зад', 'Зад-лево', 'Перед-лево'
]


def apply_staleness(
    values: list[float],
    stamps_ns: list[int | None],
    now_ns: int,
    timeout_s: float = STALE_TIMEOUT_S,
) -> tuple[list[float], list[bool]]:
    """Чистая функция: stale-каналы → inf.

    stamps_ns[i] — наносекунды последнего callback'а канала i (None = ещё
    не было ни одного чтения). Возвращает (out_values, stale_mask).
    """
    timeout_ns = int(timeout_s * 1e9)
    out: list[float] = []
    stale: list[bool] = []
    for v, ts in zip(values, stamps_ns):
        is_stale = ts is None or (now_ns - ts) > timeout_ns
        out.append(float('inf') if is_stale else v)
        stale.append(is_stale)
    return out, stale


class SensorMonitor(Node):

    def __init__(self):
        super().__init__('sensor_monitor')

        # init = MAX_RANGE = "никаких препятствий". НЕ None — иначе bridge
        # получает 0.0 (см. RCA 2026-05-19 attempt #1 fly-away).
        self.vl_data = [VL_MAX_RANGE_M] * 6
        # v2: per-channel метки свежести (ns, None = чтений ещё не было).
        # До первого чтения канал публикуется как inf — честное «нет данных».
        self.vl_stamp_ns: list[int | None] = [None] * 6
        self._stale_logged = [False] * 6

        # Реальные данные из Gazebo
        for i in range(6):
            self.create_subscription(
                LaserScan,
                f'/drone/vl53l0x/ch{i}',
                lambda msg, idx=i: self._vl_callback(msg, idx),
                10
            )

        self.create_subscription(
            LaserScan,
            '/drone/tf_luna_down',
            self._altitude_callback,
            10
        )

        self.pub_perimeter = self.create_publisher(
            Float32MultiArray, '/drone/perimeter', 10)

        self.pub_altitude = self.create_publisher(
            Float32, '/drone/altitude', 10)

        # v2 Block 2 (2026-06-06): perimeter публикуем 10 Hz таймером, а не из
        # каждого vl-callback'а. Раньше массив летел 6×10 = ~58 Hz, причём 5 из 6
        # значений в каждом сообщении были несвежими. 10 Hz = частота сенсоров.
        self.create_timer(0.1, self._publish_perimeter)

        self.get_logger().info('SensorMonitor — реальные данные Gazebo (inf→MAX_RANGE)')

    def _publish_perimeter(self):
        now_ns = self.get_clock().now().nanoseconds
        out, stale = apply_staleness(self.vl_data, self.vl_stamp_ns, now_ns)
        for i, is_stale in enumerate(stale):
            if is_stale and not self._stale_logged[i]:
                # None-каналы на старте не алярмим — это прогрев, не смерть
                if self.vl_stamp_ns[i] is not None:
                    self.get_logger().error(
                        f'VL53 ch{i} ({DIRECTIONS[i]}) STALE >'
                        f' {STALE_TIMEOUT_S}s — публикую inf вместо кэша'
                    )
                self._stale_logged[i] = True
            elif not is_stale and self._stale_logged[i]:
                self.get_logger().info(f'VL53 ch{i} ({DIRECTIONS[i]}) ожил')
                self._stale_logged[i] = False
        msg_out = Float32MultiArray()
        msg_out.data = out
        self.pub_perimeter.publish(msg_out)

    def _vl_callback(self, msg: LaserScan, idx: int):
        if not msg.ranges:
            return
        dist = msg.ranges[0]
        # inf / nan / out-of-range → cap на MAX. "Sensor read max" = "no obstacle",
        # НЕ "obstacle at 0m" (старый bug 2026-05-19).
        if not math.isfinite(dist) or dist > VL_MAX_RANGE_M:
            dist = VL_MAX_RANGE_M
        elif dist < 0.0:
            dist = 0.0

        self.vl_data[idx] = dist
        self.vl_stamp_ns[idx] = self.get_clock().now().nanoseconds

        if dist < CRITICAL:
            self.get_logger().error(
                f'CRITICAL! {DIRECTIONS[idx]} = {dist:.2f}м'
            )
        elif dist < HIGH:
            self.get_logger().warn(
                f'HIGH: {DIRECTIONS[idx]} = {dist:.2f}м'
            )

    def _altitude_callback(self, msg: LaserScan):
        if not msg.ranges:
            return
        alt = msg.ranges[0]
        # inf → MAX_RANGE. Не drop'аем callback — bridge ожидает publish'и steady rate.
        if not math.isfinite(alt) or alt > TF_LUNA_MAX_RANGE_M:
            alt = TF_LUNA_MAX_RANGE_M
        elif alt < 0.0:
            alt = 0.0

        self.get_logger().info(f'Высота: {alt:.2f}м')

        alt_msg = Float32()
        alt_msg.data = float(alt)
        self.pub_altitude.publish(alt_msg)


def main():
    rclpy.init()
    node = SensorMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()