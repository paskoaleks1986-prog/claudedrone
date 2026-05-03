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
        # Нода монитор
        Node(
            package='drone_sim',
            executable='sensor_monitor',
            name='sensor_monitor',
            output='screen'
        ),
        # SG90 servo command node — клемп target_angle → /drone/sg90/cmd
        Node(
            package='drone_sim',
            executable='servo_cmd',
            name='servo_cmd_node',
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                # Позиция дрона
                '/world/indoor_room/pose/info'
                '@geometry_msgs/msg/PoseArray'
                '[gz.msgs.Pose_V',

                # Часы симуляции
                '/clock'
                '@rosgraph_msgs/msg/Clock'
                '[gz.msgs.Clock',

                # TF Luna вниз
                '/drone/tf_luna_down'
                '@sensor_msgs/msg/LaserScan'
                '[gz.msgs.LaserScan',

                # TF Luna sweep (на sg90_arm, движется с сервой)
                '/scan/sweep'
                '@sensor_msgs/msg/LaserScan'
                '[gz.msgs.LaserScan',

                # SG90 servo команда (ROS2 → gz, double в радианах)
                '/drone/sg90/cmd'
                '@std_msgs/msg/Float64'
                ']gz.msgs.Double',

                # VL53L0X x6
                '/drone/vl53l0x/ch0'
                '@sensor_msgs/msg/LaserScan'
                '[gz.msgs.LaserScan',

                '/drone/vl53l0x/ch1'
                '@sensor_msgs/msg/LaserScan'
                '[gz.msgs.LaserScan',

                '/drone/vl53l0x/ch2'
                '@sensor_msgs/msg/LaserScan'
                '[gz.msgs.LaserScan',

                '/drone/vl53l0x/ch3'
                '@sensor_msgs/msg/LaserScan'
                '[gz.msgs.LaserScan',

                '/drone/vl53l0x/ch4'
                '@sensor_msgs/msg/LaserScan'
                '[gz.msgs.LaserScan',

                '/drone/vl53l0x/ch5'
                '@sensor_msgs/msg/LaserScan'
                '[gz.msgs.LaserScan',
            ],
            output='screen'
        ),

    ])