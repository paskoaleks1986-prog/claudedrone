#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.current_state = State()
        self.step = 0  # 0=wait, 1=guided, 2=arm, 3=flying
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')
        [self.target](http://self.target) = PoseStamped()
        [self.target](http://self.target).pose.position.x = -6.5
        [self.target](http://self.target).pose.position.y = -3.5
        [self.target](http://self.target).pose.position.z = 1.5
        # Один таймер 10Hz — управляет всем
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        self.get_logger().info('TakeoffNode started...')
    def state_callback(self, msg):
        self.current_state = msg
    def timer_callback(self):
        self.counter += 1
        # Всегда шлём setpoint
        [self.target](http://self.target).header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish([self.target](http://self.target))
        if not self.current_state.connected:
            return
        # Шаг 1 — через 3 сек после подключения: GUIDED
        if self.step == 0 and self.counter > 30:
            self.get_logger().info('Setting GUIDED mode...')
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.set_mode_[client.call](http://client.call)_async(req)
            self.step = 1
        # Шаг 2 — через 2 сек: ARM
        elif self.step == 1 and self.counter > 50:
            self.get_logger().info('Arming...')
            req = CommandBool.Request()
            req.value = True
            self.arming_[client.call](http://client.call)_async(req)
            self.step = 2
        # Шаг 3 — через 2 сек: проверяем armed
        elif self.step == 2 and self.counter > 70:
            if self.current_state.armed:
                self.get_logger().info('Armed! Flying to 1.5m!')
                self.step = 3
            else:
                self.get_logger().warn('Not armed yet, retrying...')
                req = CommandBool.Request()
                req.value = True
                self.arming_[client.call](http://client.call)_async(req)
        # Шаг 4 — летим (setpoint уже шлётся выше)
        elif self.step == 3:
            if self.counter % 50 == 0:  # лог каждые 5 сек
                self.get_logger().info('Hovering at 1.5m')
def main():
    rclpy.init()
    node = TakeoffNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()