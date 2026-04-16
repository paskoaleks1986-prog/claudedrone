#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class HelloDrone(Node):

    def __init__(self):
        super().__init__('hello_drone')
        self.get_logger().info('Нода запущена!')
        self.counter = 0
        self.create_timer(1.0, self.tick)

    def tick(self):
        self.counter += 1
        self.get_logger().info(f'tick #{self.counter}')


def main():
    rclpy.init()
    node = HelloDrone()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
