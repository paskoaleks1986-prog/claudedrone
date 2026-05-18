"""sweep_only.launch.py — standalone smoke launch для sweep_storage_node.

Не запускает Gazebo / gz_bridge / mission_fsm. Только sweep_storage сам по себе
для unit-style проверки rclpy boot + topic discovery + param load.

Полноценный smoke с реальным TF-Luna и servo идёт через stack.launch.py
(добавится отдельным change'м после первичного TASK-047 close).

Аргументы (LaunchArgument):
    cycle_period_s — длина one full triangular sweep cycle (default 9.0)
    output_mode    — laserscan | pointcloud2 (default laserscan)
    dump_npz       — true | false (default true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cycle_period_arg = DeclareLaunchArgument(
        "cycle_period_s", default_value="9.0",
        description="Full triangular sweep cycle duration in seconds",
    )
    output_mode_arg = DeclareLaunchArgument(
        "output_mode", default_value="laserscan",
        description="Output message type: laserscan | pointcloud2",
    )
    dump_npz_arg = DeclareLaunchArgument(
        "dump_npz", default_value="true",
        description="Dump NPZ per cycle in $RESEARCHBEST_ROOT/output_data/TASK-047/",
    )

    sweep_storage_node = Node(
        package="researchbest_drone",
        executable="sweep_storage",
        name="sweep_storage_node",
        output="screen",
        parameters=[{
            "cycle_period_s": LaunchConfiguration("cycle_period_s"),
            "output_mode": LaunchConfiguration("output_mode"),
            "dump_npz": LaunchConfiguration("dump_npz"),
        }],
    )

    return LaunchDescription([
        cycle_period_arg,
        output_mode_arg,
        dump_npz_arg,
        sweep_storage_node,
    ])
