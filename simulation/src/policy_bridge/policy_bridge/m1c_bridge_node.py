#!/usr/bin/env python3
"""m1c_bridge_node.py — V4-A1 M1c-инференс-мост (sim стек-инфра, Aleks 2026-06-20:
bridge=моя зона, не interface). Gazebo-сенсоры → obs[17] → M1c sb3-политика →
Twist(vx,vy,yaw_rate) на /drone/cmd_vel_body → manual_fly_node/ActionExecutor ведут
дрон (реюз проверенного velocity-пути, без своей setpoint-механики).

Контракт (ТЗ rl-lab 09:1x):
  obs[17] = m1c_obs_builder (бит-парити КОНСТРУКЦИИ с BlindCorridorEnv._obs).
  action Box(3) [vx,vy,yaw]∈[-1,1] → vx,vy×v_max(0.6) body, yaw×w_max(1.0).
  side=+1(право), band_hi=0.8, фикс-высота (ActionExecutor держит), VL-only, БЕЗ серво/TF.

Запуск (в .venv-policy — там sb3/torch):
  ros2 run policy_bridge m1c_bridge_node --ros-args \
    -p model_path:=<.../experiments/V4A1-M1c-right/model.zip>

⚠ Требует поднятый стек (takeoff + manual_fly_node слушает /drone/cmd_vel_body).
Полётная валидация — на стенде (rl-lab гоняет). obs-КОНСТРУКЦИЯ доказана оффлайн
(help_scripts/m1c_obs_parity_check.py: 500 шагов Δ=0).
"""
from __future__ import annotations
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32MultiArray, String

from policy_bridge.m1c_obs_builder import (ToFMemory, build_obs17,
                                           normalize_vl, yaw_rel_of)

CMD_TOPIC = "/drone/cmd_vel_body"          # == manual_fly_node CMD_TOPIC


class M1cBridge(Node):
    def __init__(self):
        super().__init__("m1c_bridge")
        self.declare_parameter("model_path", "")
        self.declare_parameter("side", "right")          # +1 право / −1 лево
        self.declare_parameter("band_hi_m", 0.8)
        self.declare_parameter("v_max", 0.6)
        self.declare_parameter("w_max", 1.0)
        self.declare_parameter("tof_decay_s", 2.0)
        self.declare_parameter("rate_hz", 10.0)
        # Прогон-A (researchbest gate-вердикт #2, latency limit-cycle): 1-й порядок
        # LPF на cmd_vel для подавления action-осцилляции. 0=off (baseline). τ≈0.1с →
        # alpha=dt/(τ+dt), fc=1/(2πτ)≈1.6Гц. Диагностик БЕЗ ретрейна.
        self.declare_parameter("cmd_lpf_tau_s", 0.0)
        # terminate-stop (опц., greenlit для чистого лога): на FINISH-зону → нулевой
        # Twist (hover), чтобы политика не долбила торец (z-просадка ре-рана #1/#2).
        self.declare_parameter("stop_on_finish", False)
        # полная stop-обвязка терминала (Aleks 2026-06-20): на FINISH ещё и LAND
        # (миссия завершена, не вечный hover) → Bool в /drone/land manual_fly.
        self.declare_parameter("land_on_finish", False)
        mp = self.get_parameter("model_path").value
        if not mp:
            raise SystemExit("m1c_bridge: -p model_path:=<...model.zip> обязателен")
        from stable_baselines3 import PPO        # импорт тут → нода грузится без sb3 для --help
        self.model = PPO.load(mp, device="cpu")
        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.band_hi = float(self.get_parameter("band_hi_m").value)
        self.side_cmd = 1.0 if self.get_parameter("side").value == "right" else -1.0
        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.decay_steps = float(self.get_parameter("tof_decay_s").value) / dt
        self.tof = ToFMemory(self.decay_steps)

        tau = float(self.get_parameter("cmd_lpf_tau_s").value)
        self.lpf_alpha = dt / (tau + dt) if tau > 0.0 else 1.0   # 1.0 = passthrough
        self._cmd_filt = np.zeros(3, dtype=np.float32)           # [vx,vy,yaw] фильтр-состояние
        self.stop_on_finish = bool(self.get_parameter("stop_on_finish").value)
        self.land_on_finish = bool(self.get_parameter("land_on_finish").value)
        self._finished = False
        self._land_sent = False

        self.have_vl = self.have_odom = False
        self.vl_norm = np.ones(6, dtype=np.float32)
        self.yaw = self.yaw_rate = 0.0
        self.start_yaw = None                            # захват на первом тике

        self.create_subscription(Float32MultiArray, "/drone/perimeter",
                                 self._perim_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/mavros/local_position/odom",
                                 self._odom_cb, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.land_pub = self.create_publisher(Bool, "/drone/land", 10)
        if self.stop_on_finish or self.land_on_finish:
            self.create_subscription(String, "/drone/zone_event", self._zone_cb, 10)
        self.create_timer(dt, self._tick)
        lpf = f"LPF α={self.lpf_alpha:.2f}" if self.lpf_alpha < 1.0 else "LPF off"
        self.get_logger().info(
            f"M1cBridge up: model={mp.split('/')[-1]} side={self.side_cmd:+.0f} "
            f"v_max={self.v_max} band={self.band_hi} decay={self.decay_steps}шагов "
            f"{lpf} stop_on_finish={self.stop_on_finish} land_on_finish={self.land_on_finish} "
            f"→ Twist в {CMD_TOPIC}")

    def _perim_cb(self, msg: Float32MultiArray):
        d = list(msg.data)
        if len(d) >= 6:
            self.vl_norm = normalize_vl(d[:6])
            if not self.have_vl:
                self.tof.reset(self.vl_norm)             # инициализация памяти
            self.have_vl = True

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.yaw_rate = msg.twist.twist.angular.z
        self.have_odom = True

    def _zone_cb(self, msg: String):
        import json
        try:
            if str(json.loads(msg.data).get("type", "")).upper() == "FINISH":
                if not self._finished:
                    self._finished = True
                    self.get_logger().info("FINISH-зона → terminate-stop: hover (нулевой Twist)")
                # полная stop-обвязка: stop скоростей (hover) → LAND миссии (один раз)
                if self.land_on_finish and not self._land_sent:
                    self._land_sent = True
                    self.land_pub.publish(Bool(data=True))
                    self.get_logger().info("FINISH-зона → LAND (миссия завершена) /drone/land")
        except (ValueError, TypeError):
            pass

    def _tick(self):
        if not (self.have_vl and self.have_odom):
            return
        # terminate-stop: на FINISH → нулевой Twist (hover), политику не гоняем в торец
        if self._finished:
            self.cmd_pub.publish(Twist())
            return
        if self.start_yaw is None:
            self.start_yaw = self.yaw                     # старт-курс (head-direction якорь)
            self.get_logger().info(f"M1cBridge: start_yaw={self.start_yaw:.3f} — погнали")
        self.tof.update(self.vl_norm)
        obs = build_obs17(self.tof.mem, self.tof.age, self.decay_steps,
                          yaw_rel_of(self.yaw, self.start_yaw), self.yaw_rate,
                          self.w_max, self.side_cmd, self.band_hi)
        a, _ = self.model.predict(obs, deterministic=True)
        a = np.asarray(a, dtype=np.float32).clip(-1, 1)
        raw = np.array([a[0] * self.v_max, a[1] * self.v_max, a[2] * self.w_max],
                       dtype=np.float32)
        # 1-й порядок LPF на cmd_vel (Прогон-A): y += α·(x−y); α=1 → passthrough.
        self._cmd_filt += self.lpf_alpha * (raw - self._cmd_filt)
        t = Twist()
        t.linear.x = float(self._cmd_filt[0])             # vx body
        t.linear.y = float(self._cmd_filt[1])             # vy body
        t.angular.z = float(self._cmd_filt[2])            # yaw_rate
        self.cmd_pub.publish(t)


def main():
    rclpy.init()
    node = M1cBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
