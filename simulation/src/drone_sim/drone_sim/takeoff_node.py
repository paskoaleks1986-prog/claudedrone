#!/usr/bin/env python3
"""takeoff_node — event-driven FSM для ARM+GUIDED+TAKEOFF через MAVROS.

История: v1..v5 (см. docs/dev-log/takeoff_node_v*.py + dev-log/03)
бились о tick-based FSM (mode set + arm в соседних тиках без подтверждения
state). v6 был tick-based — переход в ARM до того как ArduPilot успел
переключиться в GUIDED, поэтому NAV_TAKEOFF возвращал FAILED, дрон
disarm'ился (см. dev-log/07).

v7: state-event-driven. Каждая фаза ждёт подтверждённого перехода
(state.mode == 'GUIDED' / state.armed == True), команды повторяются с
троттлингом 2 с пока ArduPilot не подтвердил. Setpoint публикуется только
в фазе hover, отдельным быстрым таймером (10 Hz).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class TakeoffNode(Node):
    PHASES = ('wait_connect', 'set_mode', 'arming', 'takeoff', 'hover')
    TARGET_ALTITUDE = 2.0    # м, должно влезать в FENCE_ALT_MAX
    CMD_RETRY_S = 2.0        # пауза между ре-сендами одной команды
    TAKEOFF_HOLD_S = 5.0     # сколько ждём набора высоты после NAV_TAKEOFF

    def __init__(self):
        super().__init__('takeoff_node')

        self.state = State()
        self.phase = 'wait_connect'
        self.last_cmd_t = 0.0
        self.takeoff_at = None

        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_srv = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.target = PoseStamped()
        self.target.header.frame_id = 'map'
        self.target.pose.position.x = 0.0
        self.target.pose.position.y = 0.0
        self.target.pose.position.z = self.TARGET_ALTITUDE

        self.create_timer(1.0, self.fsm_loop)
        self.create_timer(0.1, self.hover_loop)

        self.get_logger().info('takeoff_node v7 запущен, ждём FCU...')

    def state_cb(self, msg):
        self.state = msg

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def to_phase(self, new_phase, reason=''):
        msg = f'phase: {self.phase} → {new_phase}'
        if reason:
            msg += f' ({reason})'
        self.get_logger().info(msg)
        self.phase = new_phase
        self.last_cmd_t = 0.0
        self.takeoff_at = None

    def throttled(self):
        return (self.now_s() - self.last_cmd_t) < self.CMD_RETRY_S

    def fsm_loop(self):
        if self.phase == 'wait_connect':
            if self.state.connected:
                self.to_phase('set_mode', 'FCU connected')
            return

        if self.phase == 'set_mode':
            if self.state.mode == 'GUIDED':
                self.to_phase('arming', f'mode={self.state.mode}')
                return
            if self.throttled():
                return
            if not self.mode_srv.service_is_ready():
                self.get_logger().warn('mavros set_mode service не готов')
                return
            self.get_logger().info('→ SetMode GUIDED')
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.mode_srv.call_async(req)
            self.last_cmd_t = self.now_s()
            return

        if self.phase == 'arming':
            if self.state.mode != 'GUIDED':
                self.to_phase('set_mode', f'mode reverted={self.state.mode}')
                return
            if self.state.armed:
                self.to_phase('takeoff', 'armed')
                return
            if self.throttled():
                return
            if not self.arm_srv.service_is_ready():
                self.get_logger().warn('mavros arming service не готов')
                return
            self.get_logger().info('→ Arming')
            req = CommandBool.Request()
            req.value = True
            self.arm_srv.call_async(req)
            self.last_cmd_t = self.now_s()
            return

        if self.phase == 'takeoff':
            if not self.state.armed:
                self.to_phase('arming', 'disarmed во время takeoff')
                return
            if self.takeoff_at is None:
                if not self.takeoff_srv.service_is_ready():
                    self.get_logger().warn('mavros takeoff service не готов')
                    return
                self.get_logger().info(f'→ NAV_TAKEOFF {self.TARGET_ALTITUDE}m')
                req = CommandTOL.Request()
                req.altitude = float(self.TARGET_ALTITUDE)
                self.takeoff_srv.call_async(req)
                self.takeoff_at = self.now_s()
                return
            if self.now_s() - self.takeoff_at > self.TAKEOFF_HOLD_S:
                self.to_phase('hover', 'takeoff hold elapsed')
            return

        if self.phase == 'hover':
            self.get_logger().info(
                f'hover: mode={self.state.mode} armed={self.state.armed}')
            return

    def hover_loop(self):
        if self.phase != 'hover':
            return
        self.target.header.stamp = self.get_clock().now().to_msg()
        self.sp_pub.publish(self.target)


def main():
    rclpy.init()
    node = TakeoffNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
