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

    # v2 Block 2 (2026-06-06): для RL-ранов autoscan:=false ОБЯЗАТЕЛЕН.
    # autoscan каждые cooldown_s триггерит sweep_node, который гоняет серву
    # 0→π — а в тренировке серву двигает ТОЛЬКО action 6 шагами 30°.
    # Параллельные sweep'ы делают servo_angle/distances[6] в obs бессмысленными.
    # Aleks 2026-06-15: ДЕФОЛТ false = скан ТОЛЬКО по кнопке /drone/sweep/start (GUI
    # мануал). true = авто-цикл (opt-in, --autoscan). RL-раны и так false.
    autoscan_arg = DeclareLaunchArgument(
        'autoscan',
        default_value='false',
        description='Автотриггер sweep циклов. ДЕФОЛТ false = скан по кнопке (GUI мануал). '
                    'true = авто-цикл (opt-in). RL-раны: false (серва — у action 6).',
    )
    autoscan = LaunchConfiguration('autoscan')

    # SITL-RL fine-tune (Aleks 2026-06-09): safety_guard:=false для train.
    # safety_guard — deployment-слой, которого НЕТ в train-env (drone_map_env):
    # он аборт­ит ротации (в train mask[4,5] всегда True) → livelock, и паузит
    # setpoint-стрим → дрон проседает/заваливается на reposition (ран TASK-RL-
    # SITL-FT-1 умер: tilt 61.8°, re-arm fail). Защита от стен в fine-tune уже
    # parity-консистентна: action_mask (env) + gate_blocks (sitl_comm). Деплой =
    # true (untrusted policy). RL train передаёт false (launch.sh --no-safety-guard).
    safety_guard_arg = DeclareLaunchArgument(
        'safety_guard',
        default_value='true',
        description='Sensor-level аварийный стоп. false для RL fine-tune (паритет '
                    'с train-env; защита = action_mask+gate_blocks). Деплой = true.',
    )
    safety_guard_cfg = LaunchConfiguration('safety_guard')

    # scan_points (C1): /scan/record для interface scan_store (точки веера/precise по
    # ФАКТ. углу джойнта). Aleks 2026-06-15: interface рисует веер ИЗ /scan/record, а
    # не из /drone/sweep/result (там командные углы → выгиб). default true для GUI/мануала;
    # RL может выключить (точки не нужны, серва у action 6).
    scan_points_arg = DeclareLaunchArgument(
        'scan_points',
        default_value='true',
        description='C1 нода /scan/record (точки по факт. углу). false для RL-ранов.',
    )
    scan_points_cfg = LaunchConfiguration('scan_points')

    return LaunchDescription([

        sweep_storage_arg,
        launch_gz_arg,
        autoscan_arg,
        safety_guard_arg,
        scan_points_arg,

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
        # DS->AP forwarder (Стенд 2026-06-08): сырые VL53/TF-Luna LaserScan →
        # sensor_msgs/Range на /mavros/* → mavros distance_sensor → MAVLink
        # DISTANCE_SENSOR → FC. Нужен пока в indoor.parm активны PRX1/RNGFND
        # (иначе AP: PreArm No Data). Harmless если параметры закомментированы.
        Node(
            package='drone_sim',
            executable='distance_sensor_forwarder',
            name='distance_sensor_forwarder',
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
        # Autoscan — независимый триггер sweep'ов с cooldown.
        # Отключаем для RL-ранов (autoscan:=false): серва принадлежит action 6.
        Node(
            package='drone_sim',
            executable='autoscan',
            name='autoscan_node',
            output='screen',
            condition=IfCondition(autoscan),
        ),
        # scan_points (C1) — /scan/record: точки веера/precise в world по ФАКТ. углу
        # джойнта (joint_state) + per-ray поза. interface рисует веер отсюда.
        Node(
            package='drone_sim',
            executable='scan_points',
            name='scan_points_node',
            output='screen',
            parameters=[{'world': world_name, 'use_sim_time': True}],
            condition=IfCondition(scan_points_cfg),
        ),
        # Safety guard — TOF-уровневая аварийная остановка (TASK-059 attempt #4).
        # Independent reactive layer ниже policy_bridge: если ANY VL53L0X/sweep
        # читает < 0.5 м, publish zero Twist на /mavros/setpoint_velocity/cmd_vel_unstamped
        # на 50 Hz (выше bridge inner loop 20 Hz) → bridge'овы non-zero Twist'ы
        # overwritten last-write-wins → drone hovers in place до уезда из safety zone.
        # v2 fix (2026-06-06): параметр назывался 'stop_threshold' — нода такого
        # не объявляет (safety_guard.py: 'stop_threshold_floor'), значение молча
        # игнорировалось и реально действовал дефолт 0.8 м.
        # v2 run E (2026-06-07): floor 0.8 → 0.5 (кольцо 0.8м = 44% комнаты).
        # v2 run F (решение Aleks 08:26): 0.5 → 0.4 — ⚠ SIM-ONLY. Wall effect
        # в Gazebo без спецплагина не моделируется; для реального железа
        # минимальный клиренс считается ЗАНОВО по диаметру пропа и diagonal
        # frame. Ниже 0.4 не идти: near-wall states за пределами надёжного
        # переноса модели без дообучения. Cap покрытия при 0.4 ≈ 0.76.
        # v2 run F checklist #1 (2026-06-07): 0.4 → 0.45 — на floor 0.4 guard
        # стрелял 11 раз/18 шагов по oblique vl[5]≈0.395-0.400 (action7
        # tug-of-war на границе). Когерентный сет: gate 0.55, wt 0.70.
        # v2 run H (решение Aleks после вердикта G): 0.45 → 0.40 НАЗАД.
        # Ран G (carrot, 0 таймаутов): 135/135 триггеров в полосе
        # 0.437-0.450 — margin (0.45) == floor (0.45), дрон паркуется на
        # линии триггера, guard 10.1 с/мин. Фикс: буфер margin−floor=0.05
        # (дефицит oblique 0.004-0.013, запас ×4); margin/gate/wt НЕ трогать
        # (0.45/0.55/0.70). Cap покрытия возвращается к ~0.76.
        Node(
            package='drone_sim',
            executable='safety_guard',
            name='safety_guard',
            output='screen',
            parameters=[{
                'stop_threshold_floor': 0.40,
                'check_rate_hz': 50.0,
            }],
            condition=IfCondition(safety_guard_cfg),
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