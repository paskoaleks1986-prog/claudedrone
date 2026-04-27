#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')

        self.state = State()
        self.tick = 0
        self.phase = 'wait'  # wait → guided → arm → takeoff → hover

        self.create_subscription(State, '/mavros/state', self.state_cb, 10)

        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_srv = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.target = PoseStamped()
        self.target.pose.position.x = -6.5
        self.target.pose.position.y = -3.5
        self.target.pose.position.z = 1.5

        self.create_timer(0.1, self.loop)
        self.get_logger().info('Нода запущена...')

    def state_cb(self, msg):
        self.state = msg

    def loop(self):
        self.tick += 1

        # Ждём подключения
        if not self.state.connected:
            return

        # Фаза 1 — через 3 сек: GUIDED
        if self.phase == 'wait' and self.tick > 30:
            self.get_logger().info('→ GUIDED mode')
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.mode_srv.call_async(req)
            self.phase = 'guided'

        # Фаза 2 — через 2 сек: ARM
        elif self.phase == 'guided' and self.tick > 50:
            self.get_logger().info('→ Arming')
            req = CommandBool.Request()
            req.value = True
            self.arm_srv.call_async(req)
            self.phase = 'arm'

        # Фаза 3 — через 2 сек: TAKEOFF команда (не setpoint!)
        elif self.phase == 'arm' and self.tick > 70:
            if self.state.armed:
                self.get_logger().info('→ Takeoff 1.5m')
                req = CommandTOL.Request()
                req.altitude = 1.5
                self.takeoff_srv.call_async(req)
                self.phase = 'takeoff'
            else:
                self.get_logger().warn('Не вооружился, повтор...')
                self.tick = 55
                self.phase = 'guided'

        # Фаза 4 — через 3 сек после takeoff: начинаем слать setpoint
        elif self.phase == 'takeoff' and self.tick > 100:
            self.get_logger().info('→ Hover mode, sending setpoint')
            self.phase = 'hover'

        # Фаза 5 — hover: только теперь шлём setpoint
        elif self.phase == 'hover':
            self.target.header.stamp = self.get_clock().now().to_msg()
            self.sp_pub.publish(self.target)
            if self.tick % 50 == 0:
                self.get_logger().info(
                    f'Режим: {self.state.mode} | Armed: {self.state.armed}')


def main():
    rclpy.init()
    node = TakeoffNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()