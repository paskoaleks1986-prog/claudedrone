import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('drone_sim')

    # Путь к миру
    world = os.path.join(pkg, 'worlds', 'indoor_room.sdf')

    # Путь к моделям
    models = os.path.join(pkg, 'models')

    # TASK-054 opt-in: переключение legacy sweep_node ↔ researchbest sweep_storage_node.
    # default=false → legacy поведение. true → sweep_storage_node с param-override под
    # совместимые legacy topic/frame (target_angle → servo_cmd → cmd; frame_id=sg90_arm).
    sweep_storage_arg = DeclareLaunchArgument(
        'sweep_storage',
        default_value='false',
        description='Use sweep_storage_node (researchbest TASK-047) вместо legacy sweep_node',
    )
    sweep_storage = LaunchConfiguration('sweep_storage')

    return LaunchDescription([

        sweep_storage_arg,

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
        # Sweep (legacy TASK-002) — step-by-step 0→π, /drone/sweep/start → /drone/sweep/result.
        # Запускается когда sweep_storage:=false (default).
        Node(
            package='drone_sim',
            executable='sweep',
            name='sweep_node',
            output='screen',
            condition=UnlessCondition(sweep_storage),
        ),
        # Sweep storage (researchbest TASK-047 / TASK-054 opt-in) — triangular sweep,
        # NPZ dump, configurable output. Параметры переопределены под legacy:
        #   cmd_topic=/drone/sg90/target_angle (через servo_cmd → /drone/sg90/cmd)
        #   frame_id=sg90_arm                  (совместимо с TF/legacy LaserScan)
        # Запускается когда sweep_storage:=true (вместо legacy sweep_node).
        Node(
            package='drone_sim',
            executable='sweep_storage',
            name='sweep_storage_node',
            output='screen',
            parameters=[{
                'cmd_topic': '/drone/sg90/target_angle',
                'frame_id': 'sg90_arm',
                'dump_dir': '/tmp/sweep_dumps/sim',
            }],
            condition=IfCondition(sweep_storage),
        ),
        # Autoscan — независимый триггер sweep'ов с cooldown
        Node(
            package='drone_sim',
            executable='autoscan',
            name='autoscan_node',
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