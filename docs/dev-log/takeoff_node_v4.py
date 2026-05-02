#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        # Состояние дрона от MAVROS
        self.state = State()
        # Флаг — прошли ли уже через взлёт
        self.armed_once = False
        # Слушаем состояние дрона
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        # Публикуем цель — куда лететь
        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        # Сервисы управления
        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')
        # Цель — зависнуть на 1.5м над стартовой позицией
        [self.target](http://self.target) = PoseStamped()
        [self.target](http://self.target).pose.position.x = -6.5
        [self.target](http://self.target).pose.position.y = -3.5
        [self.target](http://self.target).pose.position.z = 1.5
        # Счётчик тиков (1 тик = 0.1 сек)
        self.tick = 0
        # Главный цикл 10Hz
        self.create_timer(0.1, self.loop)
        self.get_logger().info('Нода запущена, ждём подключения...')
    def state_cb(self, msg):
        self.state = msg
    def loop(self):
        self.tick += 1
        # Обновляем время и шлём setpoint ВСЕГДА
        # ArduPilot требует минимум 2Hz иначе выходит из GUIDED
        [self.target](http://self.target).header.stamp = self.get_clock().now().to_msg()
        self.sp_pub.publish([self.target](http://self.target))
        # Ждём пока MAVROS подключится к ArduPilot
        if not self.state.connected:
            return
        # Через 3 сек после подключения — переходим в GUIDED
        # (нужно накопить setpoint сначала)
        if self.tick == 30:
            self.get_logger().info('Переключаем в GUIDED...')
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.mode_[srv.call](http://srv.call)_async(req)
        # Через 5 сек — армируем
        if self.tick == 50:
            self.get_logger().info('Армируем...')
            req = CommandBool.Request()
            req.value = True
            self.arm_[srv.call](http://srv.call)_async(req)
        # Через 7 сек — проверяем armed и логируем
        if self.tick == 70:
            if self.state.armed:
                self.get_logger().info('Вооружён! Летим на 1.5м...')
                # setpoint уже шлётся выше — дрон сам поднимется
            else:
                self.get_logger().warn('Не вооружился! Пробуем ещё раз...')
                req = CommandBool.Request()
                req.value = True
                self.arm_[srv.call](http://srv.call)_async(req)
                self.tick = 55  # откатываемся назад
        # Логируем каждые 5 сек после взлёта
        if self.tick > 70 and self.tick % 50 == 0:
            self.get_logger().info(
                f'Режим: {self.state.mode} | '
                f'Вооружён: {self.state.armed}')
def main():
    rclpy.init()
    node = TakeoffNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()