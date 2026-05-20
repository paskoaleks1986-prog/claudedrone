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

DIRECTIONS = [
    'Перед', 'Перед-право', 'Зад-право',
    'Зад', 'Зад-лево', 'Перед-лево'
]


class SensorMonitor(Node):

    def __init__(self):
        super().__init__('sensor_monitor')

        # init = MAX_RANGE = "никаких препятствий". НЕ None — иначе bridge
        # получает 0.0 (см. RCA 2026-05-19 attempt #1 fly-away).
        self.vl_data = [VL_MAX_RANGE_M] * 6

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

        self.get_logger().info('SensorMonitor — реальные данные Gazebo (inf→MAX_RANGE)')

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

        if dist < CRITICAL:
            self.get_logger().error(
                f'CRITICAL! {DIRECTIONS[idx]} = {dist:.2f}м'
            )
        elif dist < HIGH:
            self.get_logger().warn(
                f'HIGH: {DIRECTIONS[idx]} = {dist:.2f}м'
            )

        msg_out = Float32MultiArray()
        msg_out.data = list(self.vl_data)
        self.pub_perimeter.publish(msg_out)

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