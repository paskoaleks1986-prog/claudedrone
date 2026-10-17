"""krot_bridges.launch.py — v5-krot Этап-1 мосты поверх живого стека.

Запускать ПОВЕРХ launch.sh (gz+SITL+mavros+gz_bridge уже подняты). Поднимает:
    tof_ring_node     — /drone/vl53l0x/ch0..5 → /krot/tof_ring (obs-фид RL)
                        + /krot/tof_ring_raw (физ. 2.0м, картограф/GUI — TASK-097)
    wall_gt_node      — поза(odom)+arena → /krot/wall_gt (privileged GT критик+reward)
    yaw_slave_node    — wall_gt t̂ → /krot/yaw_cmd (нос слейв)
    vel_setpoint_mux  — /krot/cmd_vel_world(RL)+yaw → /mavros/setpoint_raw/local
    contacts_bridge   — gz /drone/crash_contacts → ROS (crash-ring, TASK-099)
    collision_node    — contacts → /krot/collision Bool (краш-детект обучения)

Arena-дескриптор резолвится по DEFAULT_WORLD (или аргумент arena:=<path>).
⚠ GZ_PARTITION обязан совпадать со стеком (launch.sh ставит его из .env_simulation,
обычно 'sim') — иначе contacts-мост и reset_node (gz service set_pose) НЕ видят
gz-транспорт: reset падает «gz set_pose не прошёл» (грабля e2e 2026-10-17).
Пример:
    GZ_PARTITION=sim DEFAULT_WORLD=v5_p0_straight ros2 launch drone_sim krot_bridges.launch.py
    ros2 launch drone_sim krot_bridges.launch.py follow_dir:=1
"""
import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _resolve_arena(pkg, world):
    direct = os.path.join(pkg, 'worlds', 'v5_krot', world, f'{world}.arena.yaml')
    if os.path.exists(direct):
        return direct
    hits = glob.glob(os.path.join(pkg, 'worlds', '**', f'{world}.arena.yaml'), recursive=True)
    return hits[0] if hits else ''


def generate_launch_description():
    pkg = get_package_share_directory('drone_sim')
    world = os.environ.get('DEFAULT_WORLD', 'v5_p0_straight')

    arena_arg = DeclareLaunchArgument(
        'arena', default_value=_resolve_arena(pkg, world),
        description='Путь к <world>.arena.yaml (GT-дескриптор для wall_gt_node)')
    follow_dir_arg = DeclareLaunchArgument(
        'follow_dir', default_value='1', description='нос-слейв направление вдоль стены (+1/-1)')
    hold_alt_arg = DeclareLaunchArgument(
        'hold_alt', default_value='1.0',
        description='высота hold (м): mux + reset_node (NAV_TAKEOFF). Default 1.0 — '
                    'кап обучения >1.5м = краш (правило Aleks, приёмка rl-lab B6); '
                    'reset_node перечитывает свой hold_alt_m на каждом цикле (param set работает).')
    latch_alt_arg = DeclareLaunchArgument(
        'latch_alt', default_value='true',
        description='true=mux латчит alt из odom (был на земле→держит землю). '
                    'false=держит hold_alt → mux САМ взлетает на hold_alt в GUIDED+armed '
                    '(airborne closed-loop, БЕЗ takeoff_node — не запускать оба, конфликт setpoint).')
    record_arg = DeclareLaunchArgument(
        'record', default_value='true',
        description='true=flight_recorder пишет gt/odom/cmd/tof/events per эпизод '
                    '(TASK-104; выключение НЕ рекомендуется — трекинг = дефолт каждого '
                    'рана, директива Aleks 2026-06-09)')
    run_id_arg = DeclareLaunchArgument(
        'run_id', default_value='',
        description='каталог рана рекордера (<sprint>/<model>/<ep> по формату rl-lab); '
                    '""=временная метка')
    tof_noise_arg = DeclareLaunchArgument(
        'tof_noise_frac', default_value='0.0',
        description='ToF-DR: σ доля (research 0.03). 0=off')
    tof_pdrop_arg = DeclareLaunchArgument(
        'tof_p_dropout', default_value='0.0',
        description='ToF-DR: p дропаута→sentinel (research 0.03, DR→0.3). 0=off')
    record = LaunchConfiguration('record')
    run_id = LaunchConfiguration('run_id')
    arena = LaunchConfiguration('arena')
    follow_dir = LaunchConfiguration('follow_dir')
    hold_alt = LaunchConfiguration('hold_alt')
    latch_alt = LaunchConfiguration('latch_alt')
    tof_noise = LaunchConfiguration('tof_noise_frac')
    tof_pdrop = LaunchConfiguration('tof_p_dropout')

    return LaunchDescription([
        arena_arg, follow_dir_arg, hold_alt_arg, latch_alt_arg, tof_noise_arg, tof_pdrop_arg,
        record_arg, run_id_arg,
        Node(package='drone_sim', executable='tof_ring_node',
             name='tof_ring_node', output='screen',
             parameters=[{'noise_sigma_frac': ParameterValue(tof_noise, value_type=float),
                          'p_dropout': ParameterValue(tof_pdrop, value_type=float)}]),
        Node(package='drone_sim', executable='wall_gt_node',
             name='wall_gt_node', output='screen',
             parameters=[{'arena': arena}]),
        Node(package='drone_sim', executable='yaw_slave_node',
             name='yaw_slave_node', output='screen',
             parameters=[{'follow_dir': ParameterValue(follow_dir, value_type=int)}]),
        Node(package='drone_sim', executable='vel_setpoint_mux',
             name='vel_setpoint_mux', output='screen',
             parameters=[{'hold_alt': ParameterValue(hold_alt, value_type=float),
                          'latch_alt_from_odom': ParameterValue(latch_alt, value_type=bool)}]),
        # crash-детект (TASK-099): gz contact-sensor → ROS → /krot/collision
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='crash_contacts_bridge', output='screen',
             arguments=['/drone/crash_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts']),
        Node(package='drone_sim', executable='collision_node',
             name='collision_node', output='screen'),
        # reset эпизода (TASK-100): /krot/episode_reset → переспавн+взлёт → /krot/episode_ready
        Node(package='drone_sim', executable='reset_node',
             name='reset_node', output='screen',
             parameters=[{'world': os.environ.get('DEFAULT_WORLD', ''),
                          'hold_alt_m': ParameterValue(hold_alt, value_type=float)}]),
        # полётные логи (TASK-104): gt/odom/cmd/tof/events per эпизод, t = sim-time
        # (/clock бриджится базовым стеком drone.launch; топики /krot/* без header —
        # штамп на приёме). Границы эпизодов = /krot/episode_reset.
        Node(package='drone_sim', executable='flight_recorder',
             name='flight_recorder', output='screen',
             condition=IfCondition(record),
             parameters=[{'use_sim_time': True,
                          'world': os.environ.get('DEFAULT_WORLD', ''),
                          'run_id': ParameterValue(run_id, value_type=str),
                          'hold_alt_m': ParameterValue(hold_alt, value_type=float)}]),
    ])
