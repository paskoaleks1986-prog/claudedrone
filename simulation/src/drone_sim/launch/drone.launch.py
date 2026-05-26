import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('drone_sim')

    # World name — env DEFAULT_WORLD (из .env_simulation) или 'indoor_room' fallback.
    # При запуске launch.sh -w <name> устанавливает DEFAULT_WORLD перед ros2 launch.
    # Используется и для SDF path, и для gz_bridge `/world/<name>/...` topics.
    world_name = os.environ.get('DEFAULT_WORLD', 'indoor_room')
    world = os.path.join(pkg, 'worlds', f'{world_name}.sdf')

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

    launch_gz_arg = DeclareLaunchArgument(
        'launch_gz',
        default_value='true',
        description='Запускать ли Gazebo внутри этого launch файла (set false если gz уже стартован extern launch.sh -gz)',
    )
    launch_gz = LaunchConfiguration('launch_gz')

    return LaunchDescription([

        sweep_storage_arg,
        launch_gz_arg,

        # Запускаем Gazebo (только если launch_gz:=true — backward compat для standalone)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            additional_env={
                'GZ_SIM_RESOURCE_PATH': models
            },
            output='screen',
            condition=IfCondition(launch_gz),
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
        # Safety guard — TOF-уровневая аварийная остановка (TASK-059 attempt #4).
        # Independent reactive layer ниже policy_bridge: если ANY VL53L0X/sweep
        # читает < 0.5 м, publish zero Twist на /mavros/setpoint_velocity/cmd_vel_unstamped
        # на 50 Hz (выше bridge inner loop 20 Hz) → bridge'овы non-zero Twist'ы
        # overwritten last-write-wins → drone hovers in place до уезда из safety zone.
        Node(
            package='drone_sim',
            executable='safety_guard',
            name='safety_guard',
            output='screen',
            parameters=[{
                'stop_threshold': 0.5,
                'check_rate_hz': 50.0,
            }],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                # Позиция дрона
                f'/world/{world_name}/pose/info'
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

                # Joint state — для bridge ObsBuilder (servo_angle = sg90_joint pos / π).
                # gz публикует gz.msgs.Model, мы маппим в sensor_msgs/JointState.
                # Topic в ROS2 = f'/world/{world_name}/model/iris_claudedrone/joint_state',
                # policy_bridge_node принимает param joint_state_topic для override.
                f'/world/{world_name}/model/iris_claudedrone/joint_state'
                '@sensor_msgs/msg/JointState'
                '[gz.msgs.Model',
            ],
            output='screen'
        ),

    ])