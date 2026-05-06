# ROS2 + Gazebo Integration — Dev Log

> **What this is:** raw development notes from setting up the ROS2 Jazzy workspace, building the first nodes, creating the indoor world, and integrating sensors with Gazebo. Written as I worked through each step.
>
> **Why it's here:** like the first dev log, this is an honest record of the engineering process — including the venv-vs-colcon conflict that took me a while to figure out, and the SDF schema reference I built up while learning Gazebo's world format.
>
> **For polished setup instructions** see [`docs/SETUP.md`](../SETUP.md).

**Date:** early – mid April 2026
**Topics covered:**
- ROS2 workspace creation and `drone_sim` package
- First Python nodes (hello, sensor_monitor)
- venv vs colcon conflict and `COLCON_IGNORE` workaround
- Mock sensor publishers
- Launch file creation
- Gazebo Harmonic install and indoor world creation
- SDF schema notes (lights, materials, sensors, models)
- Custom drone model with sensor mounts
- ros_gz_bridge for sensor data

---

NOTES:
WARN: venv и ROS2 конфликтуют
- colcon build → только с активным venv
- ros2 run/launch → только БЕЗ venv (deactivate)
- touch venv/COLCON_IGNORE → чтобы colcon не видел venv как пакет
#########################################################################
commit b896c539c5ad39e85a7a496a2bba81af0c27d4ce (HEAD -> main)
1. run
source /opt/ros/jazzy/setup.bash
2. check
printenv ROS_DISTRO
>>>jazzy
3. create proj
ros2 pkg create dron_sim --build-type ament_cmake --dependencies rclpy std_msgs sensor_msgs
4. make proj tree
/simulation/src$ tree
.
└── drone_sim
    ├── CMakeLists.txt
    ├── config
    ├── drone_sim
    │   └── __init__.py
    ├── include
    │   └── drone_sim
    ├── launch
    ├── models
    ├── package.xml
    ├── src
    └── worlds
5. Create test Node. here some code
nano <repo-root>/simulation/src/drone_sim/drone_sim/hello.py

6. Build node
cd <repo-root>/simulation

# Собрать всё
colcon build

# Собрать с симлинками — не надо пересобирать при изменении Python файлов
colcon build --symlink-install

# Собрать только один пакет
colcon build --packages-select drone_sim

После сборки всегда:
bashsource install/setup.bash

we can run our Test node
claudedrone/simulation$ ros2 run drone_sim hello
to see our nodes  
claudedrone/simulation$ ros2 node list
see node info 
claudedrone/simulation$ ros2 node info /hello_drone
########################################################
commit 812baa9ff8349971cfaf753897a94b5d091cf2f1 (HEAD -> main)

1 
we change hello.py node
example 
class HelloDrone(Node):

    def __init__(self):
    	# nodes name
        super().__init__('hello_drone')

        # Публикуем данные VL53L0X x6 — мок
        self.pub_perimeter = self.create_publisher(
            Float32MultiArray, # type of data
            '/drone/perimeter', # name of param
            10 # save in store if not reaeded
        )
    def tick()

One moment about ROS2 to build nodes I use env for all pkgs
But to run nodes we need deactivate venv because ROS2 not friendly for it

2
we rebuild prj and run
3
see topics
ros2 topic list
4
Теперь читаем данные из топика:
bashros2 topic echo /drone/altitud
5
ros2 topic hz /drone/perimeter
Должен увидеть ~1Hz — потому что наш таймер срабатывает раз в секунду.
average rate: 1.000        min: 1.000s max: 1.000s std dev: 0.00044s window: 2 
average rate: 1.000        min: 1.000s max: 1.000s std dev: 0.00035s window: 4 
#################################################################################
commit 13c7d8ae16746ad8e1148533a9bb3b539c9c519d (HEAD -> main)

WARN!!!
# Игнорируем venv для colcon
touch <repo-root>/simulation/venv/COLCON_IGNORE

1. CReate new node sensor_monitor.py
2. add new node to CMakeLists.txt after # Регистрируем ноды
install(PROGRAMS
  drone_sim/sensor_monitor.py
  RENAME sensor_monitor
  DESTINATION lib/${PROJECT_NAME}
)
3. для тестов поставим один крит парам
hello.py
perimeter.data = [1.2, 0.25, 1.5, 2.0, 1.1, 0.9]
#################################################################################
commit 363e74c84c4a3b64a64f07fbb28e895b81b10aff (HEAD -> main)

1 create launcher
simulation/src/drone_sim/launch/drone.launch.py

2 build
3.run
ros2 launch drone_sim drone.launch.py
4. usualy launcher with no text color in terminal
you can set it
RCUTILS_COLORIZED_OUTPUT=1 
#################################################################################
commit 915d761842c6f2b79e8d66d3b2404282f9164fed (HEAD -> main)

1 check Gazebo
gz sim --version
2 check ros2 pkgs for gazebo
ros2 pkg list | grep gz
3 install pkg
sudo apt install ros-jazzy-ros-gz
Это установит сразу все нужные пакеты:

ros_gz_bridge — мост между ROS2 и Gazebo
ros_gz_sim — запуск Gazebo из ROS2
ros_gz_interfaces — общие типы сообщений


4 run simulation 
gz sim empty.sdf

5 create room
newFile <repo-root>/simulation/src/drone_sim/worlds/indoor_room.sdf

6 add light
<!-- Основной свет сверху -->
<light type="directional" name="sun">
  <cast_shadows>false</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>1.0 1.0 1.0 1</diffuse>
  <specular>0.5 0.5 0.5 1</specular>
  <direction>0.1 0.1 -1</direction>
  <attenuation>
    <range>20</range>
    <constant>0.5</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
</light>

<!-- Дополнительный свет -->
<light type="point" name="light2">
  <pose>0 0 2.0 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>10</range>
    <constant>0.5</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
</light>

7 walls color
<!-- Пол — бежевый -->
<ambient>0.85 0.82 0.72 1</ambient>
<diffuse>0.85 0.82 0.72 1</diffuse>

8 check the room
gz sim <repo-root>/simulation/src/drone_sim/worlds/indoor_room.sdf

9 DESCRIPTION
SDF — главные компоненты

<world> — контейнер всего
xml<world name="indoor_room">
  <!-- всё внутри -->
</world>
Это корневой элемент. Один файл = один мир. Имя используется в ROS2 для идентификации.

<physics> — физический движок
xml<physics name="1ms" type="ignored">
  <max_step_size>0.001</max_step_size>   <!-- шаг симуляции 1мс -->
  <real_time_factor>1.0</real_time_factor> <!-- 1.0 = реальное время -->
</physics>
real_time_factor — важный параметр:

1.0 — симуляция идёт в реальном времени
2.0 — в два раза быстрее реального времени
0.5 — в два раза медленнее (для отладки)


<plugin> — подключаемые модули
xml<plugin filename="gz-sim-physics-system"
        name="gz::sim::systems::Physics"/>
Плагины добавляют функциональность миру. Три обязательных для нас:
ПлагинЗачемPhysicsфизика — гравитация, столкновенияSceneBroadcasterтранслирует сцену в GUIUserCommandsможно двигать объекты мышкой в GUI

<light> — освещение
xml<light type="directional" name="sun">
  <diffuse>1.0 1.0 1.0 1</diffuse>   <!-- R G B Alpha -->
  <specular>0.5 0.5 0.5 1</specular>
  <direction>0.1 0.1 -1</direction>  <!-- направление лучей -->
</light>
Три типа света:

directional — как солнце, параллельные лучи, нет позиции
point — лампочка, светит во все стороны, есть позиция
spot — фонарик, конус света


<model> — любой объект в сцене
xml<model name="wall_north">
  <static>true</static>        <!-- не двигается физикой -->
  <pose>0 2.0 1.25 0 0 0</pose> <!-- X Y Z Roll Pitch Yaw -->
  <link name="link">
    <!-- collision + visual -->
  </link>
</model>
<pose> — шесть чисел:

первые три: позиция X Y Z в метрах
последние три: поворот Roll Pitch Yaw в радианах


<link> — физическое тело внутри model
xml<link name="link">
  <collision name="c">...</collision>  <!-- невидимая физическая форма -->
  <visual name="v">...</visual>        <!-- видимая форма -->
  <inertial>...</inertial>             <!-- масса и инерция — для динамических объектов -->
</link>
collision и visual могут отличаться — часто упрощают collision для скорости физики.

<geometry> — форма объекта
xml<!-- Куб -->
<geometry><box><size>5 4 0.05</size></box></geometry>

<!-- Сфера -->
<geometry><sphere><radius>0.5</radius></sphere></geometry>

<!-- Цилиндр -->
<geometry><cylinder><radius>0.1</radius><length>0.5</length></cylinder></geometry>

<!-- Внешняя 3D модель -->
<geometry><mesh><uri>model://mymodel/mesh.dae</uri></mesh></geometry>

<material> — цвет и материал
xml<material>
  <ambient>0.6 0.8 0.9 1</ambient>   <!-- цвет в тени -->
  <diffuse>0.6 0.8 0.9 1</diffuse>   <!-- основной цвет -->
  <specular>0.1 0.1 0.1 1</specular> <!-- блики -->
</material>
Все цвета в формате R G B Alpha где каждое число от 0 до 1.

<sensor> — датчики (нам нужно для дрона)
xml<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0 0 0 0</pose>
  <topic>/drone/scan</topic>      <!-- топик ROS2 -->
  <update_rate>10</update_rate>   <!-- Hz -->
  <lidar>
    <range>
      <min>0.1</min>
      <max>8.0</max>
    </range>
  </lidar>
</sensor>
Датчики ставятся внутри <link>. Данные идут в ROS2 топики через ros_gz_bridge.

<include> — подключить внешнюю модель
xml<include>
  <uri>model://claudedrone</uri>   <!-- ищет в GZ_SIM_RESOURCE_PATH -->
  <pose>0 0 0.3 0 0 0</pose>
</include>
Вместо описания модели прямо в world файле — подключаем отдельный файл. Так будем делать с дроном.

Главное что нужно помнить:
world
  └── model          — объект
        └── link     — физическое тело
              ├── collision  — физика
              ├── visual     — графика
              └── sensor     — датчики


10 drone model
mkdir -p <repo-root>/simulation/src/drone_sim/models/claudedrone

this model passport
newFile <repo-root>/simulation/src/drone_sim/models/claudedrone/model.config

model
newFile <repo-root>/simulation/src/drone_sim/models/claudedrone/model.sdf

11 add drone to the room
indoor_room.sdf
    <!-- Дрон — стартовая позиция в центре -->
    <include>
      <uri>model://claudedrone</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>

12 run
export GZ_SIM_RESOURCE_PATH=<repo-root>/simulation/src/drone_sim/models
gz sim <repo-root>/simulation/src/drone_sim/worlds/indoor_room.sdf
#################################################################################
commit 452f2a082a5f12d756ae7be52917125b34b98c5d (HEAD -> main, origin/main, origin/HEAD)
START SIMUILATION
1.
cd <repo-root>/simulation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch drone_sim drone.launch.py

2. check topic lists
GAZEBO
gz topic -l
ROS2
source /opt/ros/jazzy/setup.bash
ros2 topic list

3. set communication ROS2<-->GZ
/world/indoor_room/pose/info — позиция дрона

Запускаем bridge для этого топика:
ros2 run ros_gz_bridge parameter_bridge \
  /world/indoor_room/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V
Разбор команды:

parameter_bridge — программа моста
/world/indoor_room/pose/info — топик в Gazebo
@ — разделитель
geometry_msgs/msg/PoseArray — тип в ROS2
[ — направление: из Gazebo в ROS2
gz.msgs.Pose_V — тип в Gazebo

4. can see toppic in ROS2 list
ros2 topic echo /world/indoor_room/pose/info

5. Теперь добавим bridge в launch файл чтобы он запускался автоматически

nano <repo-root>/simulation/src/drone_sim/launch/drone.launch.py

# Bridge — Gazebo <-> ROS2
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

now everything set up in launcher
#################################################################################

1. add sensors to Drone
subl <repo-root>/simulation/src/drone_sim/models/claudedrone/model.sdf

2. add it our simulation
subl <repo-root>/simulation/src/drone_sim/worlds/indoor_room.sdf

<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
3. test
#################################################################################





