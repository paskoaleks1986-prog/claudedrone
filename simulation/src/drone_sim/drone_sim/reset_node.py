#!/usr/bin/env python3
"""reset_node — reset эпизода обучения: переспавн дрона в позу X + возврат в полёт (TASK-100).

Задание rl-lab по слову Aleks 2026-10-17 (вторая половина боли №1: после краша
эпизод должен перезапускаться БЕЗ ручного вмешательства). Тренинг-луп rl-lab
дёргает reset после каждого краша/эпизода.

ВХОД:  /krot/episode_reset  geometry_msgs/Pose2D (x, y, theta[рад]) — целевая поза спавна
ВЫХОД: /krot/episode_ready  std_msgs/Bool — true = дрон снова в hover на hold_alt,
       можно летать (событие, шлётся один раз на reset); false = reset провалился
       (таймаут фазы), луп решает сам (повторить/алерт).
       /krot/mux_gate       std_msgs/Bool — false на время reset (mux молчит: стрим
       setpoint'ов давит NAV_TAKEOFF — память project_takeoff_before_mux_order),
       true по готовности.

Последовательность (FSM, все тайминги — параметры):
  1. gate=false (mux тихий) → FORCE disarm (CommandLong 400, param2=21196 —
     обычный disarm в полёте AP откажет)
  2. gz set_pose(x, y, z_ground, theta) — телепорт физдвижком (UserCommands;
     вызов `gz service` субпроцессом, partition наследуется из env стека)
  3. ground_settle_s — физика/EKF устаканиваются на земле (SITL fake-GPS: AP
     видит скачок позы на земле, EKF сходится заново)
  4. GUIDED → EKF-settle (guided_settle_s, afe2aa1: арм в несошедшийся EKF =
     AngErr crash) → arm → NAV_TAKEOFF(hold_alt) + ТИШИНА setpoint'ов → climb →
     hover-stab (HoverStabilityMonitor) — гейты takeoff_node v10
  5. gate=true → /krot/episode_ready true

Шрамы учтены: WP_YAW_BEHAVIOR=0 (в indoor.parm), SIM_GPS1_ENABLE=1 (e7ed5cd),
re-takeoff после FORCE disarm на земле — это НЕ пост-LAND кейс (там SITLComm
не садился; телепорт+disarm чистит состояние, живой climb проверяется smoke'ом).

Запуск: ros2 run drone_sim reset_node [--ros-args -p world:=v5_p0_straight]
(world пуст → автодетект по `gz topic -l` /world/<name>/).
"""
from __future__ import annotations

import math
import os
import signal
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose2D, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from drone_sim.takeoff_node import HoverStabilityMonitor


class ResetNode(Node):
    def __init__(self) -> None:
        super().__init__("reset_node")
        p = self.declare_parameter
        p("world", "")                   # имя gz-мира; "" = автодетект
        p("hold_alt_m", 1.0)             # высота возврата в полёт (= hold_alt mux).
                                         # Default 1.0 — кап обучения: >1.5м = краш
                                         # (правило Aleks, приёмка rl-lab B6).
                                         # Перечитывается КАЖДЫЙ reset (см. _reset_cb).
        p("z_ground_m", 0.2)             # z телепорта (как спавн в мирах)
        p("ground_settle_s", 2.5)        # пауза после телепорта (физика+EKF на земле)
        p("guided_settle_s", 8.0)        # EKF-settle до арма (afe2aa1)
        p("climb_arrival_frac", 0.9)
        p("climb_timeout_s", 15.0)
        p("hover_stabilize_s", 3.0)      # короче takeoff_node: reset-цикл частый
        p("hover_z_band_m", 0.5)
        p("hover_max_wait_s", 20.0)
        p("hover_descent_dz_m", -0.15)
        p("hover_descent_window_s", 2.0)
        p("cmd_retry_s", 2.0)
        p("phase_timeout_s", 40.0)       # глобальный таймаут reset'а → ready=false
        gp = lambda n: self.get_parameter(n).value
        self.hold_alt = float(gp("hold_alt_m"))
        self.z_ground = float(gp("z_ground_m"))
        self.ground_settle = float(gp("ground_settle_s"))
        self.settle_s = float(gp("guided_settle_s"))
        self.climb_frac = float(gp("climb_arrival_frac"))
        self.climb_timeout = float(gp("climb_timeout_s"))
        self.cmd_retry = float(gp("cmd_retry_s"))
        self.phase_timeout = float(gp("phase_timeout_s"))
        self._world = str(gp("world"))

        self.phase = "idle"
        self.state = State()
        self.pz = 0.0
        self.target: Pose2D | None = None
        self.last_cmd_t = 0.0
        self.phase_t0 = 0.0
        self.guided_at = None
        self.climb_at = None
        self.takeoff_sent = False
        self.disarm_sent = False
        self.teleported = False
        self.hover_mon = None

        self.create_subscription(State, "/mavros/state", self._state_cb, 10)
        self.create_subscription(Odometry, "/mavros/local_position/odom",
                                 self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Pose2D, "/krot/episode_reset", self._reset_cb, 10)
        self.ready_pub = self.create_publisher(Bool, "/krot/episode_ready", 10)
        self.gate_pub = self.create_publisher(Bool, "/krot/mux_gate", 10)
        self.sp_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)

        self.mode_srv = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_srv = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cmd_srv = self.create_client(CommandLong, "/mavros/cmd/command")
        self.takeoff_srv = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        self.create_timer(0.5, self._fsm)
        self.create_timer(0.1, self._hover_stream)
        self.get_logger().info(
            f"reset_node: /krot/episode_reset(Pose2D) → переспавн+взлёт {self.hold_alt}м "
            f"→ /krot/episode_ready (world={'auto' if not self._world else self._world})")

    # ── callbacks ──
    def _state_cb(self, m): self.state = m

    def _odom_cb(self, m): self.pz = m.pose.pose.position.z

    def _reset_cb(self, m: Pose2D) -> None:
        if self.phase != "idle":
            self.get_logger().warn(f"episode_reset проигнорирован: reset уже идёт (phase={self.phase})")
            return
        # B6 (приёмка rl-lab): hold_alt_m перечитывается на КАЖДОМ reset —
        # `ros2 param set /reset_node hold_alt_m X` действует со следующего цикла.
        self.hold_alt = float(self.get_parameter("hold_alt_m").value)
        # G-0018: safety_guard базового стека молча срывает NAV_TAKEOFF (climb
        # timeout) и перехватывает управление у стены 0.3-0.5м — штатной зоны
        # wall-follow. Ловушку делаем ГРОМКОЙ: тренинг-стек = launch.sh --krot-train.
        if self.count_publishers("/safety/active") > 0:
            self.get_logger().error(
                "safety_guard АКТИВЕН (publisher на /safety/active) — сорвёт NAV_TAKEOFF "
                "и перехватит управление у стены (G-0018). Тренинг-стек поднимать "
                "`launch.sh --krot-train` (guard выкл, паритет с train-env).")
        self.target = m
        self.get_logger().info(
            f"→ RESET эпизода: поза ({m.x:.2f}, {m.y:.2f}, θ={m.theta:.2f}), alt={self.hold_alt}м")
        self.gate_pub.publish(Bool(data=False))          # mux — тишина
        self.disarm_sent = False
        self.teleported = False
        self.takeoff_sent = False
        self._to("disarm")

    # ── helpers ──
    def _now(self): return self.get_clock().now().nanoseconds * 1e-9

    def _to(self, phase, note=""):
        self.get_logger().info(f"reset: {self.phase} → {phase}" + (f" ({note})" if note else ""))
        self.phase = phase
        self.last_cmd_t = 0.0
        self.phase_t0 = self._now()

    def _throttled(self):
        return (self._now() - self.last_cmd_t) < self.cmd_retry

    def _fail(self, why: str) -> None:
        self.get_logger().error(f"reset ПРОВАЛЕН: {why} — episode_ready=false, gate открываю")
        self.gate_pub.publish(Bool(data=True))
        self.ready_pub.publish(Bool(data=False))
        self._to("idle")

    @staticmethod
    def _gz_cli(args: list[str], timeout_s: float) -> str:
        """gz CLI с ЖЁСТКИМ потолком времени. `gz` — ruby-обёртка с детьми; у
        subprocess.run(timeout=) по таймауту убивается только обёртка, а внук держит
        pipe → communicate() ждёт вечно → единственный поток executor'а встаёт
        (подписки видны, callback'и мертвы). Своя группа процессов + killpg."""
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             text=True, start_new_session=True)
        try:
            out, _ = p.communicate(timeout=timeout_s)
            return out or ""
        except subprocess.TimeoutExpired:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            p.communicate()
            raise

    def _gz_world(self) -> str:
        if self._world:
            return self._world
        try:
            out = self._gz_cli(["gz", "topic", "-l"], 10.0)
            for line in out.splitlines():
                if line.startswith("/world/") and "/clock" in line:
                    self._world = line.split("/")[2]
                    self.get_logger().info(f"gz world автодетект: {self._world}")
                    break
        except Exception as e:  # noqa: BLE001 — любая ошибка CLI = fail reset'а, не ноды
            self.get_logger().error(f"gz topic -l: {e}")
        return self._world

    def _gz_set_pose(self, x: float, y: float, yaw: float) -> bool:
        w = self._gz_world()
        if not w:
            return False
        req = (f'name: "iris_claudedrone" position {{ x: {x} y: {y} z: {self.z_ground} }} '
               f'orientation {{ z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)} }}')
        try:
            out = self._gz_cli(
                ["gz", "service", "-s", f"/world/{w}/set_pose",
                 "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "3000", "--req", req], 10.0)
            return "true" in out
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"gz set_pose: {e}")
            return False

    # ── FSM (2 Гц) ──
    def _fsm(self) -> None:
        ph = self.phase
        if ph == "idle":
            return
        if (self._now() - self.phase_t0) > self.phase_timeout:
            self._fail(f"таймаут фазы {ph} ({self.phase_timeout}s)")
            return

        if ph == "disarm":
            if not self.state.armed:
                ok = self._gz_set_pose(self.target.x, self.target.y, self.target.theta)
                if not ok:
                    self._fail("gz set_pose не прошёл")
                    return
                self.teleported = True
                self._to("ground_settle", "телепорт ok")
                return
            if self._throttled() or not self.cmd_srv.service_is_ready():
                return
            # FORCE disarm (MAV_CMD_COMPONENT_ARM_DISARM, param2=21196): в полёте
            # обычный disarm AP откажет — после краша дрон может «лететь»
            req = CommandLong.Request()
            req.command = 400
            req.param1 = 0.0
            req.param2 = 21196.0
            self.cmd_srv.call_async(req)
            self.last_cmd_t = self._now()
            return

        if ph == "ground_settle":
            if (self._now() - self.phase_t0) < self.ground_settle:
                return
            self._to("set_mode", "земля устаканилась")
            return

        if ph == "set_mode":
            if self.state.mode == "GUIDED":
                self.guided_at = self._now()
                self._to("settle_arm")
                return
            if self._throttled() or not self.mode_srv.service_is_ready():
                return
            req = SetMode.Request()
            req.custom_mode = "GUIDED"
            self.mode_srv.call_async(req)
            self.last_cmd_t = self._now()
            return

        if ph == "settle_arm":
            if self.state.mode != "GUIDED":
                self._to("set_mode", f"mode reverted={self.state.mode}")
                return
            if self.state.armed:
                self.climb_at = self._now()
                self.takeoff_sent = False
                self._to("climb", "armed → NAV_TAKEOFF")
                return
            if (self._now() - (self.guided_at or self._now())) < self.settle_s:
                return                                   # EKF-settle (afe2aa1)
            if self._throttled() or not self.arm_srv.service_is_ready():
                return
            req = CommandBool.Request()
            req.value = True
            self.arm_srv.call_async(req)
            self.last_cmd_t = self._now()
            return

        if ph == "climb":
            # ТИШИНА setpoint'ов (gate закрыт, сами тоже молчим) — NAV_TAKEOFF сам
            if not self.takeoff_sent and self.takeoff_srv.service_is_ready():
                req = CommandTOL.Request()
                req.altitude = float(self.hold_alt)
                self.takeoff_srv.call_async(req)
                self.takeoff_sent = True
                self.get_logger().info(f"→ NAV_TAKEOFF {self.hold_alt}м")
            if not self.state.armed:
                self._to("settle_arm", "disarmed во время climb")
                return
            arrival = self.climb_frac * self.hold_alt
            if self.pz >= arrival:
                gp = lambda n: float(self.get_parameter(n).value)
                self.hover_mon = HoverStabilityMonitor(
                    target=self.hold_alt, band=gp("hover_z_band_m"),
                    stabilize_s=gp("hover_stabilize_s"),
                    descent_dz=gp("hover_descent_dz_m"),
                    descent_window_s=gp("hover_descent_window_s"))
                self._to("hover_stab", f"z={self.pz:.2f}")
            elif (self._now() - (self.climb_at or self._now())) > self.climb_timeout:
                self._fail(f"climb timeout (z={self.pz:.2f} < {arrival:.2f})")
            return

        if ph == "hover_stab":
            st = self.hover_mon.update(self._now(), self.pz)
            if self.hover_mon.timed_out(self._now(),
                                        float(self.get_parameter("hover_max_wait_s").value)):
                self._fail(f"hover не стабилизировался (z={self.pz:.2f})")
                return
            if st["stable"]:
                self.gate_pub.publish(Bool(data=True))   # mux снова стримит (odom-latch возьмёт hold)
                self.ready_pub.publish(Bool(data=True))
                self.get_logger().info(f"✅ эпизод готов: hover z={self.pz:.2f} — /krot/episode_ready true")
                self._to("idle")
            return

    def _hover_stream(self) -> None:
        """Позиционный hold в hover_stab (gate закрыт — кто-то должен держать высоту до передачи mux)."""
        if self.phase != "hover_stab":
            return
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.target.x)
        msg.pose.position.y = float(self.target.y)
        msg.pose.position.z = self.hold_alt
        msg.pose.orientation.w = 1.0
        self.sp_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ResetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass        # контекст уже закрыт внешним shutdown'ом (rcl_shutdown already called)


if __name__ == "__main__":
    main()
