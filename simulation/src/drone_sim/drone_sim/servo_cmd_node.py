#!/usr/bin/env python3
"""
servo_cmd_node — SG90 servo command translator.

Принимает целевой угол в радианах на ROS2 топике
/drone/sg90/target_angle (std_msgs/Float64), клемпит в [0, π]
и публикует в /drone/sg90/cmd (std_msgs/Float64), который через
ros_gz_bridge уходит в gz.msgs.Double и поступает в
JointPositionController плагин (см. iris_claudedrone/model.sdf).

Назначение узла как промежуточного слоя:
    - изоляция: остальные ноды (sweep, autoscan) шлют только
      target_angle, не зная про gz-bridge;
    - валидация: clamp до пределов SDF joint (0..π рад);
    - точка для расширений: smoothing, rate-limit, телеметрия.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


SG90_MIN_RAD = 0.0
SG90_MAX_RAD = math.pi  # 180°


class ServoCmdNode(Node):

    def __init__(self):
        super().__init__('servo_cmd_node')

        self.declare_parameter('min_rad', SG90_MIN_RAD)
        self.declare_parameter('max_rad', SG90_MAX_RAD)

        self._min = float(self.get_parameter('min_rad').value)
        self._max = float(self.get_parameter('max_rad').value)
        self._last_cmd: float | None = None

        self.pub_cmd = self.create_publisher(
            Float64,
            '/drone/sg90/cmd',
            10,
        )

        self.sub_target = self.create_subscription(
            Float64,
            '/drone/sg90/target_angle',
            self._on_target,
            10,
        )

        self.get_logger().info(
            f'servo_cmd_node started — limits [{self._min:.3f}, {self._max:.3f}] rad'
        )

    def _on_target(self, msg: Float64):
        target = float(msg.data)
        clamped = max(self._min, min(self._max, target))
        if clamped != target:
            self.get_logger().warn(
                f'target {target:.3f} вне [{self._min:.3f}, {self._max:.3f}] — клемп до {clamped:.3f}'
            )

        out = Float64()
        out.data = clamped
        self.pub_cmd.publish(out)
        self._last_cmd = clamped
        self.get_logger().debug(f'/drone/sg90/cmd ← {clamped:.3f} rad')


def main(args=None):
    rclpy.init(args=args)
    node = ServoCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
