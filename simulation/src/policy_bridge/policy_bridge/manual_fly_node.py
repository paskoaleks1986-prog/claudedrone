#!/usr/bin/env python3
"""manual_fly_node.py — FlightRL-v1 непрерывный velocity-облёт (rl-lab Часть B / Aleks 2026-06-10).

Тонкая обвязка: подписывается на `/drone/cmd_vel_body` (geometry_msgs/Twist, BODY-frame
vx/vy/yaw_rate) и кормит `executor.set_manual_velocity` — ЕДИНЫЙ путь движения новой
архитектуры (нет execute(0-7)/snap/settle). Maintenance-таймер executor стримит команду
как velocity-setpoint каждый тик с safety-клэмпом (B4). Источник Twist — Interface GUI
ИЛИ teleop_keyboard ИЛИ RL-политика.

Поток: взлёт + hover → слушает Twist (linear.x=vx вперёд+, linear.y=vy влево+,
angular.z=yaw_rate CCW+). Нет команды > 0.5с → тормоз в hover (failsafe).

Предусловие: живой стек (`launch.sh --full --mavros --no-autoscan --gui -w <world>`).
Запуск: `ros2 run policy_bridge manual_fly_node --instance 1 --world lab_room`
"""
from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist

from policy_bridge.sitl_comm import MavrosSITLComm, TARGET_ALTITUDE_M

SIM = "/data/git/aerosearch/claudedrone-git/simulation"
CMD_TOPIC = "/drone/cmd_vel_body"


def main() -> int:
    ap = argparse.ArgumentParser(description="FlightRL-v1 ручной velocity-облёт")
    ap.add_argument("--instance", type=int, default=1, help="SITL_INSTANCE")
    ap.add_argument("--world", default="lab_room")
    ap.add_argument("--session", default="rltrain")
    ap.add_argument("--alt", type=float, default=TARGET_ALTITUDE_M)
    ap.add_argument("--room", type=float, default=8.0, help="room dim (для comm, reposition не используется)")
    args = ap.parse_args()

    restart = f"{SIM}/help_scripts/launch.sh --restart-sitl -s {args.session} -w {args.world} -d"
    relaunch = (
        f"{SIM}/help_scripts/launch.sh --full --mavros --no-autoscan "
        f"--no-safety-guard --gui -w {args.world} -s {args.session} -d -log"
    )
    gz_log = f"/data/drone_media/sim/_runtime_logs/{args.session}-gz.log"

    comm = MavrosSITLComm(
        room_x_m=args.room, room_y_m=args.room, cell_size_m=0.1,
        target_altitude_m=args.alt, sitl_instance=args.instance,
        sitl_restart_cmd=restart, relaunch_cmd=relaunch, gz_log_path=gz_log,
    )

    def on_cmd(msg: Twist) -> None:
        comm.executor_act.set_manual_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    comm.node.create_subscription(Twist, CMD_TOPIC, on_cmd, 10)

    comm.node.get_logger().info("manual_fly: взлёт + hover…")
    comm.start_episode(randomize_spawn=False)  # takeoff, БЕЗ reposition
    comm.node.get_logger().info(
        f"✅ MANUAL FLY READY → публикуй geometry_msgs/Twist в {CMD_TOPIC} "
        "(linear.x=vx вперёд, linear.y=vy влево, angular.z=yaw_rate). Ctrl-C = стоп+close."
    )
    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        comm.executor_act.clear_manual_velocity()
        comm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
