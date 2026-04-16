from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Нода публикатор — мок датчиков
        Node(
            package='drone_sim',
            executable='hello',
            name='hello_drone',
            output='screen'
        ),

        # Нода монитор — CRITICAL/HIGH/LOW
        Node(
            package='drone_sim',
            executable='sensor_monitor',
            name='sensor_monitor',
            output='screen'
        ),

    ])
