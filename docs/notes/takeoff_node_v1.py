import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')

        # Текущее состояние дрона
        self.current_state = State()
        self.takeoff_done = False

        # Subscriber — слушаем состояние
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)

        # Publisher — шлём setpoint 10 Hz
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10)

        # Service clients
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        # Таймер 10 Hz — слать setpoint
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Целевая точка — взлёт на 1.5м
        self.target = PoseStamped()
        self.target.pose.position.x = -6.5
        self.target.pose.position.y = -3.5
        self.target.pose.position.z = 1.5

        self.get_logger().info('TakeoffNode started, waiting for connection...')

    def state_callback(self, msg):
        self.current_state = msg

    def timer_callback(self):
        # Всегда шлём setpoint — даже до взлёта
        self.target.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.target)

        # Ждём подключения
        if not self.current_state.connected:
            return

        # Один раз выполняем взлёт
        if not self.takeoff_done:
            self.takeoff_done = True
            self.get_logger().info('Connected! Starting takeoff sequence...')
            # Небольшая задержка чтобы setpoint накопился
            self.create_timer(2.0, self.do_takeoff)

    def do_takeoff(self):
        # 1. GUIDED mode
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'GUIDED'
        self.set_mode_client.call_async(mode_req)
        self.get_logger().info('Setting GUIDED mode...')

        # 2. Arm через 1 секунду
        self.create_timer(1.0, self.do_arm)

    def do_arm(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.arming_client.call_async(arm_req)
        self.get_logger().info('Arming...')
        self.get_logger().info('Takeoff to 1.5m!')


def main():
    rclpy.init()
    node = TakeoffNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()