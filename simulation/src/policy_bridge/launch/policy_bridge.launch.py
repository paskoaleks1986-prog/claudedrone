"""policy_bridge.launch.py — TASK-059 launch для policy_bridge_node.

Запускает RL policy bridge через dedicated venv `simulation/.venv-policy/`
(SB3 + torch с `--system-site-packages` от ROS2 base).

Параметры выставлены на defaults из sprint plan v2 + TASK-058 action_spec.
Калибруются совместно с Aleks'ом в ТОЧКЕ 5 (TASK-061), здесь не трогать.

Usage:
    source install/setup.bash
    ros2 launch policy_bridge policy_bridge.launch.py
    # или с overrides:
    ros2 launch policy_bridge policy_bridge.launch.py \\
        model_path:=/custom/path/model.zip \\
        free_mask_path:=/path/to/rl_room_empty_6x6/free_mask.png
"""
from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


# Пути зависят от env vars из `.aerosearch_env` / `.env_simulation`.
# Используем os.environ.get для разрешения во время generate_launch_description
# (это runtime launch_ros, не build-time CMake).
AEROSEARCH_ROOT = os.environ.get("AEROSEARCH_ROOT", "/data/git/aerosearch")
RL_LAB_ROOT = os.environ.get("RL_LAB_ROOT", "/data/git/rl-lab")
DRONE_MEDIA_ROOT = os.environ.get("DRONE_MEDIA_ROOT", "/data/drone_media")

SIMULATION_ROOT = f"{AEROSEARCH_ROOT}/claudedrone-git/simulation"
VENV_PYTHON = f"{SIMULATION_ROOT}/.venv-policy/bin/python3"
# v1.5c deploy (2026-06-07): default = ActiveMapping-v1 (md5 7bd62e23).
# SWEEP-02 — явными аргументами model_path + model_family:=sweep02.
DEFAULT_MODEL = f"{RL_LAB_ROOT}/export/activemapping_v1/model.zip"
DEFAULT_ROSBAG_DIR = f"{DRONE_MEDIA_ROOT}/sim/bags/model-to-sim"


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "model_path", default_value=DEFAULT_MODEL,
            description="Путь к SB3 model.zip. Default = ActiveMapping-v1 "
                        "(MaskablePPO, md5 7bd62e23). Для SWEEP-02 передай "
                        "путь + model_family:=sweep02 ЯВНО.",
        ),
        DeclareLaunchArgument(
            "model_family", default_value="activemapping",
            description="sweep02 (PPO Dict obs) | activemapping (MaskablePPO "
                        "Box(21,) + occupancy/frontier + ОБЯЗАТЕЛЬНЫЙ "
                        "action_masks). AM форсирует mode=rl_only.",
        ),
        DeclareLaunchArgument(
            "deterministic", default_value="true",
            description="predict(deterministic=...). AM eval-эталоны rl-lab "
                        "сняты на true; SWEEP-02 исторически летал false.",
        ),
        DeclareLaunchArgument(
            "stuck_escape", default_value="auto",
            description="StuckDetector escape-инъекции: auto (=только sweep02) "
                        "| on | off. Для AM выключено — меряем модель.",
        ),
        DeclareLaunchArgument(
            "room_size", default_value="6.4",
            description="Длина стороны bbox мира в метрах (rl_room_* = 6.4).",
        ),
        DeclareLaunchArgument(
            "cell_size", default_value="0.1",
            description="Размер ячейки grid'а в метрах.",
        ),
        DeclareLaunchArgument(
            "wall_threshold", default_value="0.50",
            description="Action 7 stop distance — VL53L0X-front < этого → stop. "
                        "TASK-059 attempt #1 RCA (2026-05-19): 0.15 м был too tight (drone 0.3 m/s, "
                        "VL53L0X max 2 m), drone hit wall до сенсорного triggering. 0.50 м safe margin.",
        ),
        DeclareLaunchArgument(
            "odom_stale_threshold_s", default_value="1.0",
            description="Если /mavros/local_position/odom callback не приходит дольше — hover_and_wait + skip predict.",
        ),
        DeclareLaunchArgument(
            "safe_box_margin_m", default_value="0.5",
            description="Drone должен оставаться в ±room_size/2+margin. Outside → permanent hover.",
        ),
        DeclareLaunchArgument(
            "linear_speed", default_value="0.30",
            description="Линейная скорость в м/с для actions 0-3 / 7. "
                        "TASK-059 attempt #4 RCA (2026-05-20): 0.15 m/s оказался ниже ArduPilot "
                        "velocity loop deadband — drone едва двигался (0.003 m/s effective). "
                        "Revert к 0.30 m/s + safety_guard (adaptive TOF threshold + IMU watchdog + body→ENU "
                        "transform) предотвращает wall hits. Adaptive speed (4 режима) → attempt #5 если нужно.",
        ),
        DeclareLaunchArgument(
            "angular_speed", default_value="0.26",
            description="Угловая скорость в рад/с (≈15°/с) для actions 4-5.",
        ),
        DeclareLaunchArgument(
            "rate_hz", default_value="10.0",
            description="Bridge loop rate (predict частота).",
        ),
        DeclareLaunchArgument(
            "free_mask_path", default_value="auto",
            description="Путь к per-world free_mask.png для Option A coverage. "
                        "'auto' → resolve по `world_name` из metadata.json. "
                        "'none' → coverage = visited.sum() / 4096 (без free_mask).",
        ),
        DeclareLaunchArgument(
            "rosbag_dir", default_value=DEFAULT_ROSBAG_DIR,
            description="Корень для rosbag2 эпизодов.",
        ),
        DeclareLaunchArgument(
            "max_steps", default_value="3000",
            description="Maximum policy steps per episode (model_card target @3k).",
        ),
        DeclareLaunchArgument(
            "mode", default_value="hybrid",
            description="Bridge mode: hybrid (wall_follow→RL), wall_follow_only, rl_only. "
                        "TASK-062 Path B HYBRID default.",
        ),
        DeclareLaunchArgument(
            "wall_distance", default_value="0.95",
            description="WallFollower target distance к стене (м). attempt #21 RCA "
                        "2026-05-20: (1) bridge's obs_builder clips к 1.2m → WF "
                        "thresholds в [0, 1.2]; (2) safety_guard threshold = 0.80m "
                        "паркует drone у 0.80m → WF wall_distance ДОЛЖЕН быть >0.80, "
                        "иначе APPROACH→FOLLOW не triggers (tug-of-war). 0.95 = выше "
                        "safety, ниже cap.",
        ),
        DeclareLaunchArgument(
            "perimeter_laps", default_value="1",
            description="Сколько кругов wall_follow перед switch к RL phase.",
        ),
        # v2 fix (2026-06-06): obs_builder по умолчанию слушал "/joint_state",
        # но ros_gz_bridge публикует /world/<world>/model/iris_claudedrone/joint_state
        # (см. drone.launch.py gz_bridge args). Override сюда никогда не передавался →
        # servo_angle в obs был вечным 0.0 — модель не знала, куда смотрит
        # sweep-дальномер (distances[6]). Default резолвим из DEFAULT_WORLD тем же
        # механизмом, что world в drone.launch.py.
        DeclareLaunchArgument(
            "joint_state_topic",
            default_value=(
                f"/world/{os.environ.get('DEFAULT_WORLD', 'indoor_room')}"
                "/model/iris_claudedrone/joint_state"
            ),
            description="JointState topic от ros_gz_bridge для servo_angle obs.",
        ),
    ]

    # ExecuteProcess вместо Node т.к. требуется dedicated isolated venv python
    # (numpy 2.x для model.zip cloudpickle совместимости, matplotlib 3.x для SB3 logger).
    # PYTHONPATH собран явно — venv-policy сам по себе isolated (no system-site-packages),
    # rclpy / std_msgs / sensor_msgs тянутся через /opt/ros/jazzy, custom extractors через
    # $RL_LAB_ROOT (DroneCombinedExtractor — кастомный feature extractor SB3 policy).
    ros_jazzy_pp = "/opt/ros/jazzy/lib/python3.12/site-packages"
    pb_install_pp = f"{SIMULATION_ROOT}/install/policy_bridge/lib/python3.12/site-packages"

    bridge_proc = ExecuteProcess(
        cmd=[
            VENV_PYTHON, "-u",
            "-m", "policy_bridge.policy_bridge_node",
            "--ros-args",
            "-p", ["model_path:=", LaunchConfiguration("model_path")],
            "-p", ["model_family:=", LaunchConfiguration("model_family")],
            "-p", ["deterministic:=", LaunchConfiguration("deterministic")],
            "-p", ["stuck_escape:=", LaunchConfiguration("stuck_escape")],
            "-p", ["room_size:=", LaunchConfiguration("room_size")],
            "-p", ["cell_size:=", LaunchConfiguration("cell_size")],
            "-p", ["wall_threshold:=", LaunchConfiguration("wall_threshold")],
            "-p", ["linear_speed:=", LaunchConfiguration("linear_speed")],
            "-p", ["angular_speed:=", LaunchConfiguration("angular_speed")],
            "-p", ["rate_hz:=", LaunchConfiguration("rate_hz")],
            "-p", ["free_mask_path:=", LaunchConfiguration("free_mask_path")],
            "-p", ["rosbag_dir:=", LaunchConfiguration("rosbag_dir")],
            "-p", ["max_steps:=", LaunchConfiguration("max_steps")],
            "-p", ["odom_stale_threshold_s:=", LaunchConfiguration("odom_stale_threshold_s")],
            "-p", ["safe_box_margin_m:=", LaunchConfiguration("safe_box_margin_m")],
            "-p", ["mode:=", LaunchConfiguration("mode")],
            "-p", ["wall_distance:=", LaunchConfiguration("wall_distance")],
            "-p", ["perimeter_laps:=", LaunchConfiguration("perimeter_laps")],
            "-p", ["joint_state_topic:=", LaunchConfiguration("joint_state_topic")],
        ],
        output="screen",
        emulate_tty=True,
        additional_env={
            # ROS2 jazzy первым → rclpy + msgs.
            # rl-lab второй → custom extractors.DroneCombinedExtractor.
            # policy_bridge install для самой ноды.
            "PYTHONPATH": f"{ros_jazzy_pp}:{RL_LAB_ROOT}:{pb_install_pp}",
        },
    )

    return LaunchDescription([*args, bridge_proc])
