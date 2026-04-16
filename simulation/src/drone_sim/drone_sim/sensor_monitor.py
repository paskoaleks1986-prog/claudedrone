#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

# Пороги приоритетов
CRITICAL = 0.30   # 30 см
HIGH     = 0.80   # 80 см


class SensorMonitor(Node):

    def __init__(self):
        super().__init__('sensor_monitor')

        # Подписываемся на периметр
        self.create_subscription(
            Float32MultiArray,
            '/drone/perimeter',
            self.perimeter_callback,
            10
        )

        # Подписываемся на высоту
        self.create_subscription(
            Float32,
            '/drone/altitude',
            self.altitude_callback,
            10
        )

        self.get_logger().info('SensorMonitor запущен — слушаем датчики')

    def perimeter_callback(self, msg):
        distances = list(msg.data)
        directions = ['Перед', 'Перед-право', 'Зад-право',
                      'Зад', 'Зад-лево', 'Перед-лево']

        for i, dist in enumerate(distances):
            if dist < CRITICAL:
                self.get_logger().error(
                    f'CRITICAL! {directions[i]} = {dist:.2f}м'
                )
            elif dist < HIGH:
                self.get_logger().warn(
                    f'HIGH: {directions[i]} = {dist:.2f}м'
                )

    def altitude_callback(self, msg):
        alt = msg.data
        if alt < 0.3:
            self.get_logger().error(f'CRITICAL! Высота = {alt:.2f}м')
        elif alt < 0.5:
            self.get_logger().warn(f'HIGH: Высота = {alt:.2f}м')
        else:
            self.get_logger().info(f'Высота: {alt:.2f}м')


def main():
    rclpy.init()
    node = SensorMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()