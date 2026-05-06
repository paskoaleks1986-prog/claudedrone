#!/usr/bin/env python3
"""
autoscan_node — независимый триггер sweep'ов.

Не знает и не интересуется текущей позицией сервы (sweep_node при
старте сам уводит серво в 0). После первого initial_delay_s публикует
std_msgs/Empty на /drone/sweep/start и слушает результат на
/drone/sweep/result; через cooldown_s после каждого результата
триггерит следующий цикл.

STOP/RESUME:
    /drone/sweep/stop   (Empty) — остановить цикл. Текущий sweep
                                  дорабатывает до конца, новый не
                                  стартует. После последнего sweep
                                  публикуется STOPPED на /scan/status.
    /drone/sweep/resume (Empty) — снять STOP и сразу триггернуть sweep.

Параметры:
    initial_delay_s — пауза перед первым sweep, default 5 с
                      (чтобы дать gz/bridge/sweep_node стабилизироваться).
    cooldown_s      — пауза между завершением sweep и следующим стартом,
                      default 10 с.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from sensor_msgs.msg import LaserScan


class AutoscanNode(Node):

    def __init__(self):
        super().__init__('autoscan_node')

        self.declare_parameter('initial_delay_s', 5.0)
        self.declare_parameter('cooldown_s', 10.0)

        self._initial_delay = float(self.get_parameter('initial_delay_s').value)
        self._cooldown = float(self.get_parameter('cooldown_s').value)

        self._n_triggered = 0
        self._n_completed = 0
        self._cooldown_timer = None
        self._stopped = False

        self.pub_start = self.create_publisher(Empty, '/drone/sweep/start', 1)
        self.pub_status = self.create_publisher(String, '/scan/status', 10)
        self.sub_result = self.create_subscription(
            LaserScan, '/drone/sweep/result', self._on_result, 1
        )
        self.sub_stop = self.create_subscription(
            Empty, '/drone/sweep/stop', self._on_stop, 1
        )
        self.sub_resume = self.create_subscription(
            Empty, '/drone/sweep/resume', self._on_resume, 1
        )

        self._delay_timer = self.create_timer(self._initial_delay, self._on_delay_done)

        self.get_logger().info(
            f'autoscan_node started — initial_delay={self._initial_delay}s, '
            f'cooldown={self._cooldown}s'
        )

    def _trigger(self):
        self._n_triggered += 1
        self.pub_start.publish(Empty())
        self.get_logger().info(
            f'autoscan: → /drone/sweep/start (trigger #{self._n_triggered})'
        )

    def _on_delay_done(self):
        self._delay_timer.cancel()
        if self._stopped:
            return
        self._trigger()

    def _on_result(self, msg: LaserScan):
        self._n_completed += 1
        finite = sum(1 for r in msg.ranges if math.isfinite(r))
        self.get_logger().info(
            f'autoscan: ← sweep #{self._n_completed} done '
            f'({len(msg.ranges)} samples, {finite} finite); '
            f'{"STOPPED — не триггерю новый sweep" if self._stopped else f"cooldown {self._cooldown}s"}'
        )
        if self._stopped:
            self.pub_status.publish(String(data='STOPPED'))
            return
        if self._cooldown_timer is not None:
            self._cooldown_timer.cancel()
        self._cooldown_timer = self.create_timer(
            self._cooldown, self._on_cooldown_done
        )

    def _on_cooldown_done(self):
        if self._cooldown_timer is not None:
            self._cooldown_timer.cancel()
            self._cooldown_timer = None
        if self._stopped:
            self.pub_status.publish(String(data='STOPPED'))
            return
        self._trigger()

    def _on_stop(self, _msg: Empty):
        if self._stopped:
            return
        self._stopped = True
        self.get_logger().info('autoscan: STOP получен — текущий sweep дорабатывает, новый не стартует')
        # если cooldown уже идёт — отменим timer и сразу публикуем STOPPED
        if self._cooldown_timer is not None:
            self._cooldown_timer.cancel()
            self._cooldown_timer = None
            self.pub_status.publish(String(data='STOPPED'))

    def _on_resume(self, _msg: Empty):
        if not self._stopped:
            return
        self._stopped = False
        self.get_logger().info('autoscan: RESUME — триггерю новый sweep')
        self._trigger()


def main(args=None):
    rclpy.init(args=args)
    node = AutoscanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
