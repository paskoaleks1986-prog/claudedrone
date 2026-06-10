# Interface ↔ Simulation — контракт ROS2-топиков

**Назначение:** топики, которые агент **Interface** читает (телеметрия/сенсоры/статус) и
пишет (команды движения/режим), с типами, QoS и частотами. Сверено live по запущенному
стеку (`ros2 topic info -v`) + SDF `update_rate` + код. ROS2 **Jazzy**, ArduPilot SITL
Copter 4.8-dev, `ROS_DOMAIN_ID=0` (для изолированного instance — свой, см. §4).

> ⚠ **QoS — критично.** BEST_EFFORT-паблишеры НЕ дойдут до RELIABLE-подписчика (0 сообщений
> молча). Подписывайся на sensor/MAVROS-стримы через `rclpy.qos.qos_profile_sensor_data`
> (BEST_EFFORT) там, где указано BEST_EFFORT. См. [[feedback_mavros_qos_best_effort]].

---

## 1. READ — телеметрия и статус (Interface подписывается)

| Топик | Тип | QoS (Reliability / Durability) | Частота | Назначение |
|---|---|---|---|---|
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | **BEST_EFFORT** / VOLATILE | ~10–30 Hz (MAVROS LOCAL_POSITION_NED) | Позиция+ориентация дрона (EKF), скорости |
| `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | **BEST_EFFORT** / VOLATILE | ~10–30 Hz | Альт. поза (только pose) |
| `/mavros/state` | `mavros_msgs/msg/State` | RELIABLE / **TRANSIENT_LOCAL** | ~1 Hz (heartbeat) | `connected`, `armed`, `mode` (GUIDED/BRAKE/LAND…) |
| `/drone/vl53l0x/ch0` … `ch5` | `sensor_msgs/msg/LaserScan` | RELIABLE / VOLATILE | 10 Hz | 6× сырой ToF (см. §1.1) |
| `/drone/perimeter` | `sensor_msgs/msg/LaserScan` | RELIABLE / VOLATILE | 10 Hz | Агрегированный perimeter (6 каналов) |
| `/drone/tf_luna_down` | `sensor_msgs/msg/LaserScan` | RELIABLE / VOLATILE | 10 Hz | Высотомер вниз (TF-Luna) |
| `/scan/sweep` | `sensor_msgs/msg/LaserScan` | RELIABLE / VOLATILE | 10 Hz (во время sweep) | TF-Luna на servo (sweep, ch6) |
| `/drone/sweep/result` | `sensor_msgs/msg/LaserScan` | RELIABLE / VOLATILE | по завершении sweep | Итог скана направления |

### 1.1 VL53L0X ToF — семантика
- **Тип `LaserScan`**, дистанция = `ranges[0]` (одиночный луч, `samples=1`). Метры.
- **Канал = heading-ОТНОСИТЕЛЬНЫЙ угол:** `ch0` +0° (вперёд по курсу), `ch1` +60°, `ch2`
  +120°, `ch3` +180° (зад), `ch4` +240°, `ch5` +300°. При yaw дрона [0] всегда вдоль курса.
- **`inf` = нет препятствия** (свободно), НЕ ошибка. Кап на `range_max` (~2.0м), не дропать
  callback. См. [[feedback_sensor_monitor_inf_handling]].
- **Mount-радиус:** сенсор на `VL_MOUNT_RADIUS_M=0.1м` от центра → дистанция до стены ОТ
  ЦЕНТРА = `ranges[0] + 0.1`.
- `range_max ≈ 2.0м` (за пределом → `inf`, дрон «слеп» дальше 2м — для UI рисуй как «открыто»).

### 1.2 Карта / occupancy (если нужен display карты)
| Топик | Тип | Источник |
|---|---|---|
| `/rl_policy/occupancy_grid` | `nav_msgs/msg/OccupancyGrid` | policy_bridge_node (INFERENCE, не train) |
| `/rl_policy/visited_grid` | `nav_msgs/msg/OccupancyGrid` | policy_bridge_node |
| `/rl_policy/mapped_ratio` | `std_msgs/msg/Float32` | 0..1, прогресс покрытия |

⚠ `/rl_policy/*` публикует **INFERENCE-нода**, НЕ train. occupancy `rl_room_*`: resolution
0.1, dims `64×64`, origin SW `(−room/2, −room/2)`, LUT UNKNOWN −1 / FREE 0 / OCCUPIED 100.

---

## 2. WRITE — команды (Interface публикует)

| Топик / сервис | Тип | QoS | Назначение |
|---|---|---|---|
| `/drone/sg90/cmd` | `std_msgs/msg/Float64` | RELIABLE / VOLATILE | Угол servo (рад, после bridge), on-demand |
| `/drone/sweep/start` | `std_msgs/msg/Empty` | RELIABLE | Запуск sweep-скана |
| `/mavros/setpoint_raw/local` | `mavros_msgs/msg/PositionTarget` | **BEST_EFFORT** / VOLATILE | Низкоуровневая команда движения (см. §2.1) |
| `/mavros/set_mode` (srv) | `mavros_msgs/srv/SetMode` | — | Режим: GUIDED / BRAKE / LAND |
| `/mavros/cmd/arming` (srv) | `mavros_msgs/srv/CommandBool` | — | ARM / DISARM |

### 2.1 `/mavros/setpoint_raw/local` — PositionTarget
- `coordinate_frame`: `FRAME_LOCAL_NED` (1).
- **`type_mask` ОБЯЗАТЕЛЕН с явным yaw** — иначе ArduPilot крутит yaw вдоль velocity → tumble.
  - position+yaw: маска позволяет `position` + `yaw` (как `RAW_TYPE_MASK_POS_YAW`).
  - velocity+yaw: `velocity` + `yaw` (как `RAW_TYPE_MASK_VEL_YAW`).
- ⚠ **Конфликт с maintenance-стримом.** Симуляция гонит свой setpoint-стрим 10 Hz
  (`action_executor._publish_maintenance`). Если Interface шлёт сюда напрямую — стримы дерутся.
  **Рекомендация:** Interface НЕ пишет `setpoint_raw/local` напрямую, а вызывает примитивы
  executor через тонкую `manual_control_node` (bridge). См. §3.

---

## 3. Рекомендуемый путь команд — через executor (не сырой setpoint)

Вместо прямого `setpoint_raw` — `manual_control_node` (bridge Interface) поверх публичного
API `ActionExecutor` (`policy_bridge/action_executor.py`):

| Команда Interface | Метод executor | Примечание |
|---|---|---|
| Поворот на **любой** угол | `rotate_by_deg(delta_deg)` / `snap_to_yaw(yaw_rad)` | БЕЗ 15°-решётки (произвольный угол) |
| Лететь до стены | `execute(7)` | forward_until_collision + safety-слой |
| Полный стоп / hover | `set_safety_hold(True)` → hover-hold | держит позу |
| Старт движения | `execute(7)` / goto | translating gate |

⚠ RL-действия `execute(0..5)` — **сетка** (шаги 0.1м, повороты 15°), для RL-parity. Для
ручного «направление+полёт» используем `snap_to_yaw`/`rotate_by_deg` + `execute(7)`, НЕ 0..5.

### 3.1 `/goto_waypoint` (спек, гейт снимается по согласованию)
- **service** `/goto_waypoint`, srv: req `float64 x, float64 y` (map ENU), опц. `float64 yaw`
  (NaN=держать), опц. `float64 z` (NaN=держать alt); resp `bool success, string msg`.
- **Валидация — bridge** (in-map + не-в-препятствии по `free_mask`/occupancy), НЕ Interface.
- Преемптит maintenance штатно (новый target через `_set_target`, не emergency).

---

## 4. Изолированный instance (параллельно с sim-стеком на D2)

Чтобы стек Interface не дрался с RL/safety-стеком на одной RTX 5070
([[project_d2_multi_instance_isolation]]):
```
SITL_INSTANCE=2  GZ_PARTITION=ifc  ROS_DOMAIN_ID=2  MAVLINK_PORT=5780
```
(`MAVLINK_PORT = 5760 + 10·SITL_INSTANCE`). Live-bringup (НЕ train):
```
help_scripts/launch.sh --full --mavros --no-autoscan --gui -w <world> -s ifc -log
```
(без `--no-safety-guard` → поднимается `/safety/active` Bool latched — страховка при ручном).

---

## 5. Ключевые параметры стека (контекст)
`WP_YAW_BEHAVIOR=0` (AP не дерётся auto-yaw с нашим явным), `ATC_ANGLE_MAX=25°` (макс крен),
`GPS1_TYPE=1` (fake-GPS SITL — BRAKE/LAND работают), `GUID_OPTIONS=0`. Высота цель 3.0м.

_Сверено: 2026-06-10, ветка feature/pre-train-preparation. При изменении стека — обновить._
