#!/usr/bin/env python3
"""manual_fly_node.py — FlightRL-v1 непрерывный velocity-облёт (rl-lab Часть B / Aleks 2026-06-10).

Тонкая обвязка: подписывается на `/drone/cmd_vel_body` (geometry_msgs/Twist, BODY-frame
vx/vy/yaw_rate) и кормит `executor.set_manual_velocity` — ЕДИНЫЙ путь движения новой
архитектуры (нет execute(0-7)/snap/settle). Maintenance-таймер executor стримит команду
как velocity-setpoint каждый тик с safety-клэмпом (B4). Источник Twist — Interface GUI
ИЛИ teleop_keyboard ИЛИ RL-политика.

Поток: взлёт на заданную высоту + hover → слушает Twist (linear.x=vx вперёд+,
linear.y=vy влево+, angular.z=yaw_rate CCW+). Нет команды > 0.5с → тормоз в hover (failsafe).

Высота вверх/вниз (Aleks 2026-06-11): ползунок interface шлёт `/drone/set_altitude`
(std_msgs/Float64, м, [0.5-2.2]) — «выйди на высоту H и держи». ЕДИНЫЙ ручной контроллер
(`executor._publish_manual_flight`) ведёт дрон на target И в hover (AP-position-setpoint),
И в горизонт-полёте (vz=Kp·Δalt). Работает без горизонт-команды (прошлый баг: z-control жил
только в velocity-пути). Статус: `/drone/altitude_locked` (Bool, True пока выходим) +
`/drone/altitude_target` (Float64, clamped target) @5Гц.

Посадка (Aleks 2026-06-11): `/drone/land` (std_msgs/Bool, любой триггер) → LAND mode →
плавный спуск → disarm. ⚠ Повторный взлёт после LAND в SITL ненадёжен — перезапусти стек.

Взлёт «с указанием высоты» (Aleks 2026-06-11): нода по умолчанию НЕ взлетает сама, а ждёт
триггер `/drone/takeoff` (std_msgs/Float64 = высота [0.5-2.2], ≤0=дефолт --alt) — Space у
interface шлёт высоту ползунка, единый владелец взлёта = manual_fly. Флаг `--auto-takeoff` =
взлёт сразу на --alt (standalone smoke без interface).

Предусловие: живой стек (`launch.sh --full --mavros --no-autoscan --gui -w <world>`).
Запуск: `ros2 run policy_bridge manual_fly_node --instance 1 --world lab_room [--auto-takeoff]`
"""
from __future__ import annotations

import argparse
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64

from policy_bridge.action_executor import ALT_MAX_M, ALT_MIN_M, MANUAL_ALT_DEFAULT_M
from policy_bridge.sitl_comm import MavrosSITLComm

SIM = "/data/git/aerosearch/claudedrone-git/simulation"
CMD_TOPIC = "/drone/cmd_vel_body"
ALT_CMD_TOPIC = "/drone/set_altitude"          # interface ползунок → target высота (м)
ALT_LOCKED_TOPIC = "/drone/altitude_locked"    # статус: True пока выходим на высоту
ALT_TARGET_TOPIC = "/drone/altitude_target"    # echo clamped target (м)
TAKEOFF_TOPIC = "/drone/takeoff"               # interface Space → взлёт (Float64 высота, ≤0=дефолт)
LAND_TOPIC = "/drone/land"                     # interface → посадка (std_msgs/Bool, любой триггер)


def main() -> int:
    ap = argparse.ArgumentParser(description="FlightRL-v1 ручной velocity-облёт")
    ap.add_argument("--instance", type=int, default=1, help="SITL_INSTANCE")
    ap.add_argument("--world", default="lab_room")
    ap.add_argument("--session", default="rltrain")
    ap.add_argument("--alt", type=float, default=MANUAL_ALT_DEFAULT_M,
                    help=f"высота взлёта (м), клампится в [{ALT_MIN_M}, {ALT_MAX_M}]")
    ap.add_argument("--room", type=float, default=8.0, help="room dim (для comm, reposition не используется)")
    ap.add_argument("--auto-takeoff", action="store_true",
                    help="взлететь сразу на launch (--alt). Без флага — ждём триггер /drone/takeoff от interface.")
    args = ap.parse_args()
    # Высота взлёта «с указанием» (Aleks 2026-06-11): клампим в диапазон ползунка.
    takeoff_alt = max(ALT_MIN_M, min(ALT_MAX_M, args.alt))

    # Aleks 2026-06-11: РУЧНОЙ режим НЕ перезапускает стек сам. Teleop-нода не должна
    # дёргать launch.sh (раньше watchdog на climb-timeout запускал FULL relaunch
    # `-s rltrain` поверх живого стека → MAVLINK_PORT 5760 already bound → мусор).
    # relaunch/restart=None → hard_reset() деградирует в «soft land+disarm+warn»
    # (sitl_comm.py:747), стек остаётся живым. Если взлёт реально не идёт — это EGL/
    # рендер (фиксится env __EGL_VENDOR_LIBRARY_FILENAMES в launch.sh), не relaunch.
    comm = MavrosSITLComm(
        room_x_m=args.room, room_y_m=args.room, cell_size_m=0.1,
        target_altitude_m=takeoff_alt, sitl_instance=args.instance,
        sitl_restart_cmd=None, relaunch_cmd=None, gz_log_path=None,
        manual_mode=True,   # ручной облёт: глушим RL crash-latch (tilt-термин) — Aleks пилот
    )
    ex = comm.executor_act

    def on_cmd(msg: Twist) -> None:
        ex.set_manual_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    def on_alt(msg: Float64) -> None:
        # Ползунок-высота: «выйди на высоту H и держи» — лочит горизонт до выхода.
        z = ex.set_target_altitude(msg.data)
        comm.node.get_logger().info(f"set_altitude → {z:.2f} м (lock высоты, возврат контроля по достижении)")

    comm.node.create_subscription(Twist, CMD_TOPIC, on_cmd, 10)
    comm.node.create_subscription(Float64, ALT_CMD_TOPIC, on_alt, 10)

    locked_pub = comm.node.create_publisher(Bool, ALT_LOCKED_TOPIC, 10)
    target_pub = comm.node.create_publisher(Float64, ALT_TARGET_TOPIC, 10)

    def pub_alt_status() -> None:
        locked_pub.publish(Bool(data=ex.altitude_locked))
        target_pub.publish(Float64(data=ex.target_altitude_m))

    comm.node.create_timer(0.2, pub_alt_status)  # 5 Гц статус для слайдера interface

    # Взлёт «с указанием высоты» (Aleks 2026-06-11): единый владелец взлёта = manual_fly.
    # По умолчанию ждём триггер /drone/takeoff (Space у interface шлёт высоту ползунка);
    # --auto-takeoff = сразу на --alt (standalone smoke). start_episode блокирующий →
    # запускаем из ГЛАВНОГО потока (не из callback, чтобы не вешать executor-спин).
    tk_lock = threading.Lock()
    tk = {"req": args.auto_takeoff, "alt": takeoff_alt, "done": False, "land": False}

    def on_takeoff(msg: Float64) -> None:
        a = max(ALT_MIN_M, min(ALT_MAX_M, msg.data if msg.data > 0 else takeoff_alt))
        with tk_lock:
            if tk["done"]:
                comm.node.get_logger().warn("takeoff-триггер проигнорирован — уже в воздухе")
                return
            tk["alt"] = a
            tk["req"] = True
        comm.node.get_logger().info(f"takeoff-триггер принят → взлёт на {a:.2f} м")

    def on_land(msg: Bool) -> None:
        # Посадка: глушим maintenance-стрим и садимся (блокирующе → из главного потока).
        with tk_lock:
            if not tk["done"]:
                comm.node.get_logger().warn("land-триггер проигнорирован — ещё на земле")
                return
            tk["land"] = True
        comm.node.get_logger().info("land-триггер принят → посадка")

    comm.node.create_subscription(Float64, TAKEOFF_TOPIC, on_takeoff, 10)
    comm.node.create_subscription(Bool, LAND_TOPIC, on_land, 10)

    if args.auto_takeoff:
        comm.node.get_logger().info(f"manual_fly: auto-takeoff на {takeoff_alt:.2f} м…")
    else:
        comm.node.get_logger().info(
            f"manual_fly: ГОТОВ, жду триггер {TAKEOFF_TOPIC} "
            f"(std_msgs/Float64 высота [{ALT_MIN_M}-{ALT_MAX_M}], ≤0 = дефолт {takeoff_alt:.1f}м)…"
        )
    try:
        while rclpy.ok():
            do_tk = None
            with tk_lock:
                if tk["req"] and not tk["done"]:
                    tk["req"] = False
                    do_tk = tk["alt"]
            if do_tk is not None:
                comm.target_altitude = do_tk      # NAV_TAKEOFF target + climb-arrival
                ex.target_altitude = do_tk        # post-takeoff z-hold setpoint
                comm.node.get_logger().info(f"взлёт на {do_tk:.2f} м…")
                comm.start_episode(randomize_spawn=False)  # takeoff, БЕЗ reposition
                ex.enter_manual_flight()          # ЕДИНЫЙ ручной контроллер (hover+высота+полёт)
                with tk_lock:
                    tk["done"] = True
                comm.node.get_logger().info(
                    f"✅ MANUAL FLY READY → vel: Twist в {CMD_TOPIC} (x=vx,y=vy,z=yaw_rate); "
                    f"высота: Float64 в {ALT_CMD_TOPIC} ([{ALT_MIN_M}-{ALT_MAX_M}]м); "
                    f"посадка: Bool в {LAND_TOPIC}. Ctrl-C = стоп+close."
                )
            do_land = False
            with tk_lock:
                if tk["land"]:
                    tk["land"] = False
                    do_land = True
            if do_land:
                comm.node.get_logger().info("посадка…")
                ex.exit_manual_flight()           # стоп стрима (иначе перебивает LAND)
                comm.land()                       # LAND mode → спуск → disarm (→ _airborne=False)
                # Сброс латча → принимаем новый /drone/takeoff (взлёт/посадка ×N,
                # Aleks 2026-06-14). comm.land() уже сбросил _airborne → следующий
                # start_episode сделает свежий _full_takeoff. Повторный взлёт после
                # LAND в SITL РАБОТАЕТ при живом GPS-фиксе (SIM_GPS1_ENABLE=1 на буте);
                # stand-verify 2026-06-14: re-arm+climb до 1.0м после LAND прошёл.
                with tk_lock:
                    tk["done"] = False
                comm.node.get_logger().info(
                    f"✅ приземлился. Готов к повторному взлёту — шли {TAKEOFF_TOPIC} "
                    f"(Float64 высота)."
                )
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        ex.clear_manual_velocity()
        comm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
