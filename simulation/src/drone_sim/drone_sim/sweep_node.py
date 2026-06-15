#!/usr/bin/env python3
"""
sweep_node — функция скана 0→180° по команде.

При получении std_msgs/Empty на /drone/sweep/start ведёт серво
шагами по step_rad от 0 до π, между шагами ждёт settle_ms (чтобы
TF-Luna при 10 Hz успел выдать свежий замер), и собирает ranges
в один итоговый sensor_msgs/LaserScan на /drone/sweep/result.

Параметры:
    step_rad   — шаг угла, по умолчанию 1° ≈ 0.01745 рад.
    settle_ms  — пауза на каждом шаге, по умолчанию 120 мс
                 (TF-Luna 10 Hz даёт новый сэмпл каждые 100 мс).

Минимальный осмысленный шаг — порядка ширины луча TF-Luna
и углового разрешения SG90, около 1°.

Сам узел только реализует сканирующее движение и сборку. Триггер
(периодический или по запросу) — отдельная нода autoscan (subtask 5).
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Float64, String
from sensor_msgs.msg import LaserScan


SWEEP_MIN_RAD = 0.0
SWEEP_MAX_RAD = math.pi


class SweepNode(Node):

    def __init__(self):
        super().__init__('sweep_node')

        # Ручной (GUI) fine-дефолт: 1°/120мс ≈ 181 шаг ≈ 21.7с/проход — качество картографа.
        self.declare_parameter('step_rad', math.radians(1.0))
        self.declare_parameter('settle_ms', 120)
        # RL fast-дефолт (S3, Aleks 2026-06-14): 5°/60мс ≈ 37 шагов ≈ 2.2с/проход —
        # беспараметрический триггер /drone/sweep/start_fast для дискрет-обёртки SCAN_FAN.
        # Отдельный триггер → не конфликтует с GUI fine-настройками (один и тот же серво).
        self.declare_parameter('fast_step_rad', math.radians(5.0))
        self.declare_parameter('fast_settle_ms', 60)
        # ШАГ 0 = стартовый ПЕРЕГОН серво к началу (после прошлого прохода серво
        # стоит на θ=π; перегон π→0 ~520мс при velocity 6 рад/с). Обычный settle
        # (120/60мс) короче перегона → первые сэмплы снимались ПОКА серво ещё ехал →
        # старт-сторона веера выгнута (RCA 2026-06-15, скрин Aleks). home_settle_ms
        # = дать серво доехать до start до первого сэмпла. Направление-агностично.
        self.declare_parameter('home_settle_ms', 900)

        self._step_rad = float(self.get_parameter('step_rad').value)
        self._settle_ms = int(self.get_parameter('settle_ms').value)
        self._fast_step_rad = float(self.get_parameter('fast_step_rad').value)
        self._fast_settle_ms = int(self.get_parameter('fast_settle_ms').value)
        self._home_settle_ms = int(self.get_parameter('home_settle_ms').value)

        # state machine — «активные» параметры текущего прохода (fine ИЛИ fast)
        self._sweeping = False
        self._step_idx = 0
        self._step_rad_active = self._step_rad
        self._settle_ms_active = self._settle_ms
        self._n_steps = self._calc_n_steps(self._step_rad)
        self._collected: list[float] = []
        self._last_range: Optional[float] = None
        self._last_range_t: Optional[float] = None
        self._step_started_t: Optional[float] = None

        # publishers
        self.pub_target = self.create_publisher(Float64, '/drone/sg90/target_angle', 10)
        self.pub_result = self.create_publisher(LaserScan, '/drone/sweep/result', 10)
        self.pub_progress = self.create_publisher(Float32, '/drone/sweep/progress', 10)
        self.pub_status = self.create_publisher(String, '/scan/status', 10)

        # subscribers
        self.sub_start = self.create_subscription(Empty, '/drone/sweep/start', self._on_start, 1)
        self.sub_start_fast = self.create_subscription(Empty, '/drone/sweep/start_fast', self._on_start_fast, 1)
        self.sub_stop = self.create_subscription(Empty, '/drone/sweep/stop', self._on_stop, 1)
        self.sub_scan = self.create_subscription(LaserScan, '/scan/sweep', self._on_scan, 10)

        # 50 Hz tick — выше, чем TF-Luna 10 Hz, для отзывчивого FSM
        self.create_timer(0.02, self._tick)

        self.get_logger().info(
            f'sweep_node started — step={math.degrees(self._step_rad):.2f}° '
            f'({self._step_rad:.4f} rad), settle={self._settle_ms} ms, '
            f'n_steps={self._n_steps}'
        )

    @staticmethod
    def _calc_n_steps(step_rad: float) -> int:
        return int(math.ceil(SWEEP_MAX_RAD / step_rad)) + 1

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_scan(self, msg: LaserScan):
        if msg.ranges:
            self._last_range = float(msg.ranges[0])
            self._last_range_t = self._now_s()

    def _on_stop(self, _msg: Empty):
        # GUI radio-button (Aleks 2026-06-11): прерывание разового sweep на полпути
        # (mode 2). До этого sweep_node не имел /stop — проход ~21.7с был неотменяем.
        if not self._sweeping:
            return
        self.get_logger().info('sweep: STOP — прерываю проход')
        self._sweeping = False
        self._step_started_t = None
        self.pub_status.publish(String(data='STOPPED'))

    def _on_start(self, _msg: Empty):
        self._begin(self._step_rad, self._settle_ms, 'fine')

    def _on_start_fast(self, _msg: Empty):
        # S3: беспараметрический быстрый проход для RL SCAN_FAN (coarse step/short settle).
        self._begin(self._fast_step_rad, self._fast_settle_ms, 'fast')

    def _begin(self, step_rad: float, settle_ms: int, label: str):
        if self._sweeping:
            self.get_logger().warn('sweep уже идёт — игнорирую новый /start')
            return
        self._step_rad_active = step_rad
        self._settle_ms_active = settle_ms
        self._n_steps = self._calc_n_steps(step_rad)
        self.get_logger().info(
            f'sweep: start ({label}) — {self._n_steps} шагов, '
            f'step={math.degrees(step_rad):.1f}° settle={settle_ms}мс'
        )
        self._sweeping = True
        self._step_idx = 0
        self._collected = []
        self.pub_status.publish(String(data='SCANNING'))
        # шлём цель = 0 и сразу засекаем settle
        self._send_target(0.0)
        self._step_started_t = self._now_s()

    def _send_target(self, angle: float):
        m = Float64()
        m.data = float(angle)
        self.pub_target.publish(m)

    def _tick(self):
        if not self._sweeping:
            return
        if self._step_started_t is None:
            return

        # шаг 0 = большой перегон серво к стартовому углу → ждём дольше (home_settle),
        # иначе старт-сторона веера снимается на едущем серво → выгиб (RCA 2026-06-15).
        settle = self._home_settle_ms if self._step_idx == 0 else self._settle_ms_active
        elapsed_ms = (self._now_s() - self._step_started_t) * 1000.0
        if elapsed_ms < settle:
            return

        # требуем чтобы был хоть один scan-сэмпл, полученный после старта шага
        if self._last_range_t is None or self._last_range_t < self._step_started_t:
            return

        # sample
        self._collected.append(self._last_range if self._last_range is not None else float('inf'))
        self.pub_progress.publish(Float32(data=float(self._step_idx + 1) / float(self._n_steps)))

        self._step_idx += 1
        if self._step_idx >= self._n_steps:
            self._finish_sweep()
            return

        # next angle
        next_angle = min(self._step_idx * self._step_rad_active, SWEEP_MAX_RAD)
        self._send_target(next_angle)
        self._step_started_t = self._now_s()

    def _finish_sweep(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'sg90_arm'
        scan.angle_min = SWEEP_MIN_RAD
        scan.angle_max = SWEEP_MAX_RAD
        scan.angle_increment = self._step_rad_active
        scan.time_increment = self._settle_ms_active / 1000.0
        scan.scan_time = scan.time_increment * self._n_steps
        scan.range_min = 0.2
        scan.range_max = 8.0
        scan.ranges = list(self._collected)
        scan.intensities = []
        self.pub_result.publish(scan)
        self.pub_status.publish(String(data='COMPLETE'))
        self.get_logger().info(
            f'sweep: done — {len(scan.ranges)} samples, '
            f'min={min(scan.ranges):.2f} m, max={max(scan.ranges):.2f} m'
        )
        self._sweeping = False
        self._step_started_t = None


def main(args=None):
    rclpy.init(args=args)
    node = SweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
