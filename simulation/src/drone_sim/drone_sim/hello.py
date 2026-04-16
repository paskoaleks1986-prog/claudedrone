#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32


class HelloDrone(Node):

    def __init__(self):
        # node name
        super().__init__('hello_drone')

        # Публикуем данные VL53L0X x6 — мок
        self.pub_perimeter = self.create_publisher(
            Float32MultiArray,
            '/drone/perimeter',
            10
        )

        # Публикуем высоту TF Luna — мок
        self.pub_altitude = self.create_publisher(
            Float32,
            '/drone/altitude',
            10
        )

        self.create_timer(1.0, self.tick)
        self.get_logger().info('Нода запущена!')

    def tick(self):
        # Мок данные VL53L0X x6 (в метрах)
        perimeter = Float32MultiArray()
        perimeter.data = [1.2, 0.8, 1.5, 2.0, 1.1, 0.9]
        self.pub_perimeter.publish(perimeter)

        # Мок высота TF Luna (в метрах)
        altitude = Float32()
        altitude.data = 1.2
        self.pub_altitude.publish(altitude)

        self.get_logger().info(
            f'Периметр: {perimeter.data} | Высота: {altitude.data}м'
        )


def main():
    rclpy.init()
    node = HelloDrone()
    # делает цикл
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
