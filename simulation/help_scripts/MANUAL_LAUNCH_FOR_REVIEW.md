# Manual launch — review mode

> Для Aleks'а. Запуск стека **без `launch.sh`** — каждый компонент в своём терминале,
> со всеми env vars и параметрами руками. Цель — понять что куда подключается,
> что слушает что публикует, какие ноды можно outright игнорировать.

**Минимальный стек:** Gazebo + ArduPilot SITL + ros_gz_bridge (clock) + drone_sim
ноды (sensor_monitor, servo_cmd, sweep, gz_bridge внутри drone.launch.py) + MAVROS +
takeoff_node + policy_bridge (опционально).

**Ветка:** `dbg/aleks-env-takeoff-bridg`. Содержит только то что нужно для bare-minimum
взлёта + bridge. Никаких rosbag/screenshot/launch.sh скриптов.

---

## 0. Pre-flight

### 0.1 Env

```bash
# Главный env (Aerosearch root, paths, AEROSEARCH_ROOT, WS_DIR, DRONE_MEDIA_ROOT)
source /data/git/aerosearch/.aerosearch_env

# Sim env (WS_DIR override, ARDUPILOT_DIR, WORLD_DIR, PARAMS_DIR)
set -a; source /data/git/aerosearch/.env_simulation; set +a

# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Проверка
echo "AEROSEARCH_ROOT=$AEROSEARCH_ROOT"
echo "WS_DIR=$WS_DIR"                     # /data/git/aerosearch/claudedrone-git/simulation
echo "ARDUPILOT_DIR=$ARDUPILOT_DIR"       # /data/ardupilot
echo "WORLD_DIR=$WORLD_DIR"
echo "ROS_DISTRO=$ROS_DISTRO"             # jazzy
```

### 0.2 SITL eeprom wipe (КРИТИЧНО)

ArduCopter persistит params в `eeprom.bin`. Если не вытереть — `indoor.parm` игнорируется,
старые значения остаются. **Делать ВСЕГДА перед launch если меняли .parm.**

```bash
rm -f /data/ardupilot/ArduCopter/{eeprom.bin,mav.parm,mav.tlog}
```

### 0.3 Build colcon workspace

```bash
cd "$WS_DIR"
colcon build --symlink-install
source install/setup.bash
```

После build — keep этот терминал открытым (он будет parent для каждого нового pane,
наследует `install/setup.bash`).

### 0.4 Multi-instance isolation env (всегда экспортить во ВСЕ терминалы)

```bash
export SITL_INSTANCE=0
export MAVLINK_PORT=$((5760 + 10 * SITL_INSTANCE))   # = 5760
export GZ_PARTITION=sim
export ROS_DOMAIN_ID=0
```

> Если нужны два SITL одновременно (на D2) — бамп `SITL_INSTANCE=1`, тогда
> `MAVLINK_PORT=5770`, `GZ_PARTITION=sim2`, `ROS_DOMAIN_ID=1`.

---

## 1. Терминал 1 — Gazebo

**Цель:** запустить world `rl_room_empty_6x6` с `iris_claudedrone` моделью.

```bash
source /data/git/aerosearch/.aerosearch_env
set -a; source /data/git/aerosearch/.env_simulation; set +a
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
export GZ_PARTITION=sim
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/drone_sim/models"

# Запуск — НЕ-headless (с GUI) для review:
gz sim "$WS_DIR/src/drone_sim/worlds/rl_room_empty_6x6.sdf" -r

# Если нужно headless:
# gz sim "$WS_DIR/src/drone_sim/worlds/rl_room_empty_6x6.sdf" -r -s
```

**Флаги:**
- `-r` — autorun (физика запущена сразу, не paused)
- `-s` — server only (headless, без GUI)
- world path — абсолютный. **Без `-r`** Gazebo стартует с физикой на паузе — drone не свалится, но и SITL handshake не пройдёт.

**Что должно происходить:**
- Откроется Gazebo окно, world rl_room_empty_6x6, iris drone в центре
- Лог: `[Msg] Loading SDF file ...`
- ⚠ Если видишь `libEGL warning: pci id 10de:2f04, driver (null)` — NVIDIA EGL degraded,
  нужен reboot D2 (см. memory про Gazebo kill cycles).

**Что мониторить (в отдельном терминале):**
```bash
gz topic -l | head -20                        # список топиков world'а
gz topic -e -t /world/rl_room_empty_6x6/dynamic_pose/info  # позиция drone'а
gz model --list                                # модели в world
```

---

## 2. Терминал 2 — ArduPilot SITL

**Цель:** ArduCopter 4.5 в SITL mode, JSON FDM режим (связка с Gazebo через `127.0.0.1:9002`).

```bash
source /data/git/aerosearch/.aerosearch_env
set -a; source /data/git/aerosearch/.env_simulation; set +a
export SITL_INSTANCE=0
cd "$ARDUPILOT_DIR/ArduCopter"

sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    -I 0 \
    --add-param-file="$WS_DIR/config/ardupilot/indoor.parm"
```

**Параметры:**
- `-v ArduCopter` — vehicle тип
- `-f gazebo-iris` — frame (использует iris airframe)
- `--model JSON` — внешний FDM через JSON socket (связь с ArduPilotPlugin в Gazebo)
- `-I 0` — instance number (MAVLink ports = `5760 + 10*I`)
- `--add-param-file` — указатель на indoor.parm (после wipe eeprom)

**Что должно происходить:**
1. Compile (если не cached): несколько секунд
2. `INFO Started model JSON` + `Connected to ArduPilot controller @ 127.0.0.1:51607`
3. MAVProxy console (текстовый): `Waiting for heartbeat from 127.0.0.1:14550`
4. Через 5-10s: `HEARTBEAT MAV_TYPE_QUADROTOR`, `EKF3 IMU0 initial yaw alignment complete`,
   `GPS 1: detected as u-blox at 230400 baud`, `Origin set to <coords>`.

⚠ **Без heartbeat (просто Waiting forever)** — typical signs:
- ArduPilot не подключился к Gazebo (`Connected to ArduPilot controller ...` нет в Gazebo логе)
- порт `5760` уже занят (предыдущий SITL не убит) — `ss -ltn | grep 5760`
- eeprom не wipe'нут, params crash loop (см. memory про Cyrillic comments в `.parm`)

**Команды в MAVProxy console (полезное):**
```
mode GUIDED            # переключить режим
arm throttle           # arm vehicle
takeoff 2              # NAV_TAKEOFF 2m
status                 # full state
param show GPS1_TYPE   # проверить param
graph ALT              # ASCII график altitude
```

**Что мониторить (в отдельном терминале — на этом этапе ничего ROS2 ещё не работает):**
```bash
ss -ltn | grep -E "576[0-9]|11000|9002"   # порты SITL должны быть LISTEN
```

---

## 3. Терминал 3 — ros_gz_bridge (clock)

**Цель:** sync ROS2 `/clock` с Gazebo `/world/.../clock`. Без этого ROS2 ноды не получают time.

```bash
source /data/git/aerosearch/.aerosearch_env
set -a; source /data/git/aerosearch/.env_simulation; set +a
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
export GZ_PARTITION=sim
export ROS_DOMAIN_ID=0

ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

**Параметр format:** `<ros_topic>@<ros_type>[<gz_type>` — `[` означает gz→ros direction.

**Что мониторить:**
```bash
ros2 topic echo /clock --once          # должно показать sec/nanosec
ros2 topic hz /clock                    # ~1000 Hz при -r autorun
```

---

## 4. Терминал 4 — drone_sim ROS2 stack

**Цель:** запустить sensor_monitor + servo_cmd + sweep + gz_bridge для VL53L0X/IMU/altitude
topics. Это весь drone-side, кроме MAVROS и policy.

```bash
source /data/git/aerosearch/.aerosearch_env
set -a; source /data/git/aerosearch/.env_simulation; set +a
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
export GZ_PARTITION=sim
export ROS_DOMAIN_ID=0
export DEFAULT_WORLD=rl_room_empty_6x6           # критично — gz_bridge подключается к /world/$DEFAULT_WORLD/...

ros2 launch drone_sim drone.launch.py launch_gz:=false
```

**Параметр `launch_gz:=false` — КРИТИЧНО.** Без него drone.launch.py попытается запустить
свой собственный `gz sim` через ExecuteProcess → второй Gazebo → SITL talks to wrong world
(это был root cause #1 из attempt #1).

**Что должно подняться (ноды):**
- `/sensor_monitor` — читает gz LaserScan/Altitude, публикует `/drone/perimeter`, `/drone/altitude`
- `/servo_cmd_node` — `/drone/sg90/target_angle` → `/drone/sg90/cmd` (Float64)
- `/sweep_node` — legacy sweep (TF-Luna swivel)
- `/gz_bridge` — ros_gz_bridge для sensor topics (внутри launch файла)

**Если хочешь использовать researchbest sweep_storage_node:**
```bash
ros2 launch drone_sim drone.launch.py launch_gz:=false sweep_storage:=true
```

**Что мониторить:**
```bash
ros2 node list                                   # должно быть 4 ноды + ros_gz_bridge
ros2 topic list | grep -E "drone|world"
ros2 topic echo /drone/perimeter --once          # [d0,d1,d2,d3,d4,d5] floats — VL53L0X 6 каналов
ros2 topic echo /drone/altitude --once           # TF-Luna вниз
ros2 topic hz /drone/perimeter                   # ~10 Hz steady
```

---

## 5. Терминал 5 — MAVROS

**Цель:** мост MAVLink (SITL UDP) → ROS2 (`/mavros/*` topics).

```bash
source /data/git/aerosearch/.aerosearch_env
set -a; source /data/git/aerosearch/.env_simulation; set +a
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
export ROS_DOMAIN_ID=0
export SITL_INSTANCE=0

FCU_REMOTE=$((14550 + 10 * SITL_INSTANCE))   # = 14550
FCU_LOCAL=$((14555 + 10 * SITL_INSTANCE))    # = 14555

ros2 launch mavros apm.launch fcu_url:="udp://:$FCU_REMOTE@$FCU_LOCAL"
```

**Что должно подняться:**
- `/mavros` node + `/mavros/state`, `/mavros/local_position/odom`, `/mavros/local_position/pose`,
  `/mavros/imu/data`, `/mavros/setpoint_velocity/cmd_vel_unstamped`, etc.
- Connected к SITL: `[INFO] FCU: Built ... ArduCopter V4.5 ...`

⚠ **QoS gotcha (memory `feedback_mavros_qos_best_effort`):** mavros publish'ит **BEST_EFFORT**
для всех sensor topics (`/mavros/local_position/*`, `/mavros/imu/*`). Любой subscriber с
default RELIABLE получит ZERO messages silently. **Всегда `qos_profile_sensor_data` для
mavros subs.** policy_bridge уже исправлен — но если пишешь свой sub в ad-hoc — помни.

**Что мониторить:**
```bash
ros2 topic info --verbose /mavros/local_position/odom    # должно показать Reliability: BEST_EFFORT
ros2 topic echo /mavros/state                            # connected: true, armed: false, mode: STABILIZE
ros2 topic echo /mavros/local_position/pose --once
```

---

## 6. Терминал 6 — Takeoff

**Цель:** arm + GUIDED + NAV_TAKEOFF 2.0m + hover. Node v10 event-driven (FSM).

```bash
source /data/git/aerosearch/.aerosearch_env
set -a; source /data/git/aerosearch/.env_simulation; set +a
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
export ROS_DOMAIN_ID=0

ros2 run drone_sim takeoff
```

**FSM:** FCU_CONNECT → SET_MODE_GUIDED → ARM → NAV_TAKEOFF → STREAM_SETPOINTS → HOVER

**Что должно происходить (логи в этом терминале):**
- `[FCU] connected` — после ~2s
- `[mode] GUIDED ack`
- `[arm] ack=true`
- `[takeoff] NAV_TAKEOFF z=2.0 ack`
- `[stream] streaming hover setpoints`
- `[hover] ready` — drone в воздухе на z=2m

⚠ **Если drone armed + GUIDED + NAV_TAKEOFF accepted, но `/mavros/local_position/pose.z`
остаётся ~0.243m или ~0.0** — это **handshake AP↔Gazebo broken**. Plugin loaded ✓ but
`/model/.../rotor_0_joint/cmd_force = 0 publishers`. Recovery: `Ctrl+C` весь стек,
**reboot D2** (не просто kill — EGL degraded после kill cycles). См. memory
`feedback_gazebo_kill_cycles_break_egl`.

**Что мониторить:**
```bash
ros2 topic echo /mavros/local_position/pose --field pose.position.z    # должно расти от 0 до 2.0
ros2 topic echo /mavros/state --field armed                            # true
gz topic -e -t /world/rl_room_empty_6x6/dynamic_pose/info | head -50
```

---

## 7. Терминал 7 — policy_bridge (опционально)

**Цель:** запустить PPO inference bridge. **Только после того как drone в hover @ z=2m.**

### 7.1 Setup venv (один раз)

`policy_bridge.launch.py` use'ет dedicated `simulation/.venv-policy/`. Если его нет:

```bash
cd "$WS_DIR"
python3 -m venv .venv-policy   # БЕЗ --system-site-packages! см. memory feedback_venv_isolated_for_sb3
source .venv-policy/bin/activate
pip install stable-baselines3 torch numpy pillow
deactivate
```

> rclpy идёт **не через venv**, а через PYTHONPATH от `/opt/ros/jazzy` + `install/setup.bash`.
> venv нужен ИЗОЛИРОВАННЫЙ — system numpy 1.26 ломает SB3 model (pickled numpy 2.x).

### 7.2 Запуск

```bash
source /data/git/aerosearch/.aerosearch_env
set -a; source /data/git/aerosearch/.env_simulation; set +a
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
export ROS_DOMAIN_ID=0
export RL_LAB_ROOT=/data/git/rl-lab          # для default model.zip

# Минимум — RL only, без wall_follower (для review одного слоя за раз):
ros2 launch policy_bridge policy_bridge.launch.py mode:=rl_only

# Или wall_follow_only — debug только perimeter:
# ros2 launch policy_bridge policy_bridge.launch.py mode:=wall_follow_only

# Или hybrid (default — TASK-062):
# ros2 launch policy_bridge policy_bridge.launch.py
```

**Полный список launch params (из policy_bridge.launch.py):**

| Param | Default | Что |
|---|---|---|
| `model_path` | `$RL_LAB_ROOT/export/sweep02/model.zip` | SB3 PPO model |
| `room_size` | `6.4` | м, bbox мира |
| `cell_size` | `0.1` | м, grid cell |
| `wall_threshold` | `0.50` | м, action 7 stop dist (attempt #1 RCA fix) |
| `odom_stale_threshold_s` | `1.0` | hover если odom stale |
| `safe_box_margin_m` | `0.5` | м, permanent hover если drone outside |
| `linear_speed` | `0.30` | м/с (attempt #4 RCA — 0.15 был ниже AP deadband) |
| `angular_speed` | `0.26` | rad/с (~15°/с) |
| `rate_hz` | `10.0` | bridge tick |
| `free_mask_path` | `auto` | auto / explicit / `none` |
| `rosbag_dir` | `$DRONE_MEDIA_ROOT/sim/bags/model-to-sim` | rosbag2 root |
| `max_steps` | `3000` | per episode |
| `mode` | `hybrid` | `hybrid` / `wall_follow_only` / `rl_only` |
| `wall_distance` | `0.95` | м, WF target distance (attempt #21 — vs safety_guard 0.80) |
| `perimeter_laps` | `1` | WF circuits before switch на RL |

**Что должно происходить:**
- `Bridge ready` после ~3s (model load, subs setup)
- `step 0 · action <N> · coverage 0.000 · pose (x,y,z=2.0)`
- coverage растёт по мере того как drone движется в visited cells

**Что мониторить:**
```bash
ros2 topic echo /policy_bridge/coverage --once
ros2 topic echo /policy_bridge/phase                # hybrid mode: WALL_FOLLOW → RL_EXPLORE
ros2 topic hz /policy_bridge/coverage
```

---

## 8. Опционально — safety_guard

Independent ROS2 node — brake'ит drone если any VL53L0X < 0.80m. Pub в
`/mavros/setpoint_velocity/cmd_vel_unstamped` приоритетно.

```bash
source /data/git/aerosearch/.aerosearch_env
set -a; source /data/git/aerosearch/.env_simulation; set +a
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

ros2 run drone_sim safety_guard
```

⚠ **Конфликт с wall_follower:** safety_guard brake'ит при < 0.80m, WF хочет drone на 0.95m
(после attempt #21 fix). До #21 WF хотел 0.60m → tug-of-war. Это коллизия двух подсистем
которая стоит обсуждения (см. dev-log 20 Open item #2).

---

## 9. Cleanup — после review

**Никогда не используй `kill -9` для launch parent'ов** (memory `feedback_smoke_cleanup`).
ROS2 child node'ы становятся orphan'ами и держат topics, следующий launch получит
duplicate publishers warning или Plugin loaded twice.

```bash
# Каждый терминал: Ctrl+C один раз, ждать ~3s graceful shutdown.

# Если что-то осталось живое:
pgrep -af "ros2|gz sim|sim_vehicle|ardupilot|policy_bridge|mavros" | head
# Целиться SIGTERM в parent, не SIGKILL.

# SITL eeprom — оставить или wipe (см. §0.2 для next launch).
```

---

## Ноды и топики которые стоит смотреть всегда

| Что | Команда | Зачем |
|---|---|---|
| ROS2 nodes | `ros2 node list` | Все ноды в `ROS_DOMAIN_ID=0` |
| ROS2 topics | `ros2 topic list \| sort` | Полный список |
| Topic QoS | `ros2 topic info --verbose <name>` | Reliability/Durability — критично для mavros subs |
| Topic hz | `ros2 topic hz <name>` | Не упало ли publishing |
| MAVROS state | `ros2 topic echo /mavros/state` | connected/armed/mode |
| Drone position | `ros2 topic echo /mavros/local_position/pose` | ENU pose |
| Gazebo entities | `gz model --list` | Что в world физически |
| Gazebo topics | `gz topic -l` | Что publish'ит physics |
| Gazebo dynamic pose | `gz topic -e -t /world/<name>/dynamic_pose/info` | Real drone position в world (vs MAVROS reports) |

**Сравнение real vs reported pose** — если они расходятся, EKF lying (attempt #14c-style bug):
- Real position: `gz topic -e -t /world/rl_room_empty_6x6/dynamic_pose/info` → координаты iris_claudedrone
- MAVROS reported: `ros2 topic echo /mavros/local_position/pose.pose.position`
- Если real_z = 0.243m а MAVROS reports z = 2.0m — handshake broken, drone не лифт физически.

---

## Что НЕ запускать на этой ветке

- ❌ `launch.sh` — нет на ветке намеренно, review только manual
- ❌ `rosbag_record_episode.sh` — нет, ad-hoc `ros2 bag record` если нужно
- ❌ `screenshot_world.sh` — нет
- ❌ Не запускать одновременно `wall_follow_only` + `rl_only` — это разные modes одного bridge'а

## Что отсутствует на ветке (vs WIP `c53c4cc`)

Намеренно не cherry-pick'нуто чтобы review был фокусированный:
- `help_scripts/launch.sh` (с auto-флагами)
- `help_scripts/rosbag_record_episode.sh`
- `help_scripts/screenshot_world.sh`
- `simulation/.gitignore` patch (3 строки — `.venv-policy/` пр.)

Если что-то понадобится — `git checkout c53c4cc -- <path>` на этой ветке.
