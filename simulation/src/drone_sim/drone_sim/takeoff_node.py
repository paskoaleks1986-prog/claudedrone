#!/usr/bin/env python3
"""takeoff_node v10 — NAV_TAKEOFF + continuous position-setpoint hold.

История версий:
  v7 — NAV_TAKEOFF based; gap в setpoints вызывал auto-disarm
  v8 — pure position setpoints, no NAV_TAKEOFF (drone не лифтится без GPS)
  v9 — SET_GPS_GLOBAL_ORIGIN, AHRS_OPTIONS 24 (origin set but throttle=0)
  v10 — combine: NAV_TAKEOFF initiates climb + setpoints maintain position
        + DISARM_DELAY=0 в parm prevents timeout disarm. GPS1_TYPE=1 enables
        SITL fake GPS so origin auto-set.

Sequence:
  wait_connect → set_mode (GUIDED) → arming (CommandBool) →
  takeoff (NAV_TAKEOFF + setpoint stream starts) →
  climb (wait z>=1.8 via setpoint hold) →
  hover (stabilize 5s) → released (signal bridge via /takeoff/ready)

Hover_loop активен в phases: takeoff + climb + hover. Setpoint stream
поднимает drone в случае NAV_TAKEOFF issues, и предотвращает disarm.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class TakeoffNode(Node):
    PHASES = ('wait_connect', 'set_mode', 'arming', 'climb', 'hover', 'released')
    TARGET_ALTITUDE = 2.0       # m
    CLIMB_ARRIVAL_M = 1.8       # z >= 1.8 = climb complete (close to 2.0 target)
    CLIMB_TIMEOUT_S = 15.0      # safety: switch hover even if z not reached
    CMD_RETRY_S = 2.0           # service retry interval
    HOVER_STABILIZE_S = 5.0     # hover stable before release to bridge
    ORIGIN_WAIT_S = 1.5         # wait after publishing origin before next phase
    ORIGIN_LAT = 51.0           # dummy origin (no real GPS, any valid coord OK)
    ORIGIN_LON = 0.0
    ORIGIN_ALT = 0.0

    def __init__(self):
        super().__init__('takeoff_node')

        self.state = State()
        self.pose_z = 0.0
        self.phase = 'wait_connect'
        self.last_cmd_t = 0.0
        self.origin_pub_at = None   # phase=set_origin publish time
        self.climb_at = None        # phase=climb start time
        self.hover_at = None
        self.ready_published = False
        self.origin_published = False
        self.takeoff_cmd_sent = False

        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        # odom QoS BEST_EFFORT per memory feedback_mavros_qos_best_effort
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.odom_cb,
            qos_profile_sensor_data)

        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.origin_pub = self.create_publisher(
            GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.ready_pub = self.create_publisher(Bool, '/takeoff/ready', ready_qos)
        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_srv = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.target = PoseStamped()
        self.target.header.frame_id = 'map'
        self.target.pose.position.x = 0.0
        self.target.pose.position.y = 0.0
        self.target.pose.position.z = self.TARGET_ALTITUDE
        self.target.pose.orientation.w = 1.0  # identity yaw

        self.create_timer(1.0, self.fsm_loop)
        self.create_timer(0.1, self.hover_loop)

        self.get_logger().info('takeoff_node v9 запущен, ждём FCU...')

    def state_cb(self, msg):
        self.state = msg

    def odom_cb(self, msg):
        self.pose_z = msg.pose.pose.position.z

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def to_phase(self, new_phase, reason=''):
        msg = f'phase: {self.phase} → {new_phase}'
        if reason:
            msg += f' ({reason})'
        self.get_logger().info(msg)
        self.phase = new_phase
        self.last_cmd_t = 0.0

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
                # Armed → issue NAV_TAKEOFF + start setpoint stream (climb phase).
                self.climb_at = self.now_s()
                self.takeoff_cmd_sent = False
                self.to_phase('climb', 'armed → NAV_TAKEOFF + setpoint stream')
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

        if self.phase == 'climb':
            # Issue NAV_TAKEOFF first (one-shot), then setpoint stream maintains.
            if not self.takeoff_cmd_sent:
                if self.takeoff_srv.service_is_ready():
                    self.get_logger().info(
                        f'→ NAV_TAKEOFF {self.TARGET_ALTITUDE}m')
                    req = CommandTOL.Request()
                    req.altitude = float(self.TARGET_ALTITUDE)
                    self.takeoff_srv.call_async(req)
                    self.takeoff_cmd_sent = True
            if not self.state.armed:
                self.to_phase('arming', 'disarmed во время climb')
                return
            elapsed = self.now_s() - (self.climb_at or self.now_s())
            if self.pose_z >= self.CLIMB_ARRIVAL_M:
                self.to_phase('hover', f'z={self.pose_z:.2f}m ≥ {self.CLIMB_ARRIVAL_M}m')
                return
            if elapsed > self.CLIMB_TIMEOUT_S:
                self.get_logger().warn(
                    f'climb timeout {elapsed:.1f}s, z={self.pose_z:.2f}m < {self.CLIMB_ARRIVAL_M}m '
                    f'— switching hover anyway'
                )
                self.to_phase('hover', 'climb timeout')
                return
            # log progress every ~2s
            if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.5:
                self.get_logger().info(f'climb: z={self.pose_z:.2f}m elapsed={elapsed:.1f}s')
            return

        if self.phase == 'hover':
            if self.hover_at is None:
                self.hover_at = self.now_s()
            elapsed = self.now_s() - self.hover_at
            if elapsed >= self.HOVER_STABILIZE_S and not self.ready_published:
                self.ready_pub.publish(Bool(data=True))
                self.ready_published = True
                self.get_logger().info(
                    f'hover stable ≥{self.HOVER_STABILIZE_S}s (z={self.pose_z:.2f}m) — '
                    f'releasing setpoint control to bridge'
                )
                self.to_phase('released', 'bridge takes over setpoint_position/local')
                return
            self.get_logger().info(
                f'hover: mode={self.state.mode} armed={self.state.armed} '
                f'z={self.pose_z:.2f}m elapsed={elapsed:.1f}s'
            )
            return

        if self.phase == 'released':
            return

    def hover_loop(self):
        # Setpoint stream ТОЛЬКО в hover phase (Aleks @14:00 RCA):
        # publishing во время climb перебивает NAV_TAKEOFF — position controller
        # видит target z=2m vs current z=0.21m, но land_detector says "on ground"
        # → throttle=0. NAV_TAKEOFF делает climb сам, без interference.
        # После z>=1.8 → hover phase → setpoint hold + bridge takes over.
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
