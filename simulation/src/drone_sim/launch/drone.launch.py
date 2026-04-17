import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('drone_sim')

    # Путь к миру
    world = os.path.join(pkg, 'worlds', 'indoor_room.sdf')

    # Путь к моделям
    models = os.path.join(pkg, 'models')

    return LaunchDescription([

        # Запускаем Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            additional_env={
                'GZ_SIM_RESOURCE_PATH': models
            },
            output='screen'
        ),

        # Нода публикатор — мок датчиков
        Node(
            package='drone_sim',
            executable='hello',
            name='hello_drone',
            output='screen'
        ),

        # Нода монитор
        Node(
            package='drone_sim',
            executable='sensor_monitor',
            name='sensor_monitor',
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                '/world/indoor_room/pose/info'
                '@geometry_msgs/msg/PoseArray'
                '[gz.msgs.Pose_V',
                '/clock'
                '@rosgraph_msgs/msg/Clock'
                '[gz.msgs.Clock',
            ],
            output='screen'
        ),

    ])