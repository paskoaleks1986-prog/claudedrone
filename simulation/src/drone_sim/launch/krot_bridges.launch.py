"""krot_bridges.launch.py — v5-krot Этап-1 мосты (4 ноды) поверх живого стека.

Запускать ПОВЕРХ launch.sh (gz+SITL+mavros+gz_bridge уже подняты). Поднимает:
    tof_ring_node     — /drone/vl53l0x/ch0..5 → /krot/tof_ring (obs-фид RL)
    wall_gt_node      — поза(odom)+arena → /krot/wall_gt (privileged GT критик+reward)
    yaw_slave_node    — wall_gt t̂ → /krot/yaw_cmd (нос слейв)
    vel_setpoint_mux  — /krot/cmd_vel_world(RL)+yaw → /mavros/setpoint_raw/local

Arena-дескриптор резолвится по DEFAULT_WORLD (или аргумент arena:=<path>).
Пример:
    DEFAULT_WORLD=p0_straight_wall ros2 launch drone_sim krot_bridges.launch.py
    ros2 launch drone_sim krot_bridges.launch.py follow_dir:=1
"""
import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    world = os.environ.get('DEFAULT_WORLD', 'p0_straight_wall')

    arena_arg = DeclareLaunchArgument(
        'arena', default_value=_resolve_arena(pkg, world),
        description='Путь к <world>.arena.yaml (GT-дескриптор для wall_gt_node)')
    follow_dir_arg = DeclareLaunchArgument(
        'follow_dir', default_value='1', description='нос-слейв направление вдоль стены (+1/-1)')
    hold_alt_arg = DeclareLaunchArgument(
        'hold_alt', default_value='2.0', description='высота hold (м) для vel_setpoint_mux fallback')
    arena = LaunchConfiguration('arena')
    follow_dir = LaunchConfiguration('follow_dir')
    hold_alt = LaunchConfiguration('hold_alt')

    return LaunchDescription([
        arena_arg, follow_dir_arg, hold_alt_arg,
        Node(package='drone_sim', executable='tof_ring_node',
             name='tof_ring_node', output='screen'),
        Node(package='drone_sim', executable='wall_gt_node',
             name='wall_gt_node', output='screen',
             parameters=[{'arena': arena}]),
        Node(package='drone_sim', executable='yaw_slave_node',
             name='yaw_slave_node', output='screen',
             parameters=[{'follow_dir': ParameterValue(follow_dir, value_type=int)}]),
        Node(package='drone_sim', executable='vel_setpoint_mux',
             name='vel_setpoint_mux', output='screen',
             parameters=[{'hold_alt': ParameterValue(hold_alt, value_type=float)}]),
    ])
