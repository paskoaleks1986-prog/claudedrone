# Interface ↔ Simulation — контракт ROS2-топиков (continuous-velocity архитектура)

> 🖊 **ВЛАДЕЛЕЦ контракта/спеки — rl-lab** (директива Aleks 2026-06-10, мандат «слушать
> rl-lab как Aleks»). **Authoritative источник:** `rl-lab/docs/new_env_spec.md` (v2,
> obs/action/reward) + `rl-lab/docs/training_principles_and_sim_contract.md` (Часть B = sim).
> Этот файл — **sim-side companion** (топик-имена/QoS/частоты, что я реализую). Числа RL —
> из их спеки: action **Box(3)** vx/vy ×0.5 м/с (body) + yaw_rate ×1.0 рад/с, 10 Гц; VL53
> obs-clip **1.2 м**; TF-Luna 12×15° (0–180°), sentinel UNKNOWN **2.0**. Parity-формула
> (сенсор→сектор, decay freshness) — пишет rl-lab (B8), подставлю под неё.

**Назначение:** топики, которые **bridge** публикует (для RL-политики И Interface) и
**executor** принимает (от политики И Interface). Архитектура «направление + полёт»:
ПОЛИТИКА И ИНТЕРФЕЙС говорят с дроном через **один непрерывный velocity-эндпоинт**, а не
через дискретные действия. Safety-слой **клэмпит** скорость (уменьшает), не блокирует.

> ROS2 **Jazzy**, ArduPilot SITL Copter 4.8-dev. Для изолированного instance — свой
> `ROS_DOMAIN_ID` (см. §4). **QoS критично:** BEST_EFFORT-паблишер НЕ дойдёт до RELIABLE-
> подписчика (0 сообщений молча) — см. [[feedback_mavros_qos_best_effort]].
>
> ⚠ **Статус:** контракт описывает ЦЕЛЕВУЮ архитектуру (RL-spec). Помечено `[ЕСТЬ]` —
> уже публикуется; `[НОВОЕ]` — bridge должен реализовать (rename/новый стрим/эндпоинт).

---

## 1. BRIDGE ПУБЛИКУЕТ → политика + Interface

| Топик | Тип | QoS | Частота | Статус | Назначение |
|---|---|---|---|---|---|
| `/mavros/vl53_ch0` … `vl53_ch5` | `std_msgs/msg/Float32` | RELIABLE / VOLATILE | 10 Hz | `[НОВОЕ]` rename | 6 сырых ToF, **метры** (см. §1.1) |
| `/drone/tfluna_sectors` | `drone_sim/msg/TFLunaSectors` | RELIABLE / VOLATILE | 10 Hz | `[НОВОЕ]` | TF-Luna угловое кольцо 12 секторов (см. §1.2) |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | **BEST_EFFORT** / VOLATILE | ~10–30 Hz | `[ЕСТЬ]` | velocity (twist) + heading (pose orientation) |

### 1.1 VL53 ToF (`/mavros/vl53_ch0..ch5`)
- **6 отдельных топиков**, каждый — одно расстояние в **метрах** (`std_msgs/Float32`,
  `data` = дистанция).
- **Канал = heading-ОТНОСИТЕЛЬНЫЙ угол:** `ch0` +0° (вперёд по курсу), `ch1` +60°, `ch2`
  +120°, `ch3` +180° (зад), `ch4` +240°, `ch5` +300°. [0] всегда вдоль текущего курса.
- **`inf` (или `range_max`) = свободно**, НЕ ошибка. `range_max ≈ 2.0м` (дальше дрон «слеп»).
- **Mount-радиус:** сенсор на **0.1м** от центра → дистанция до стены ОТ ЦЕНТРА = `data + 0.1`.
- ⚠ Rename из текущего `/drone/vl53l0x/chN` (`sensor_msgs/LaserScan`) → bridge перепубликует
  `ranges[0]` как `Float32` под `/mavros/vl53_chN`.

### 1.2 TF-Luna угловое кольцо (`/drone/tfluna_sectors`)
- TF-Luna на servo **сканирует 0–180°** → агрегируется в **12 секторов** (по 15°).
- Каждый сектор: `dist_m` (последнее измерение, метры) + `freshness_steps` (сколько шагов
  назад сектор последний раз измерен — счётчик устаревания; sweep обновляет по одному сектору).
- Публикуется **всё кольцо 10 Hz** (last-known + возраст по каждому сектору).
- Предлагаемый msg `drone_sim/msg/TFLunaSectors`:
  ```
  std_msgs/Header header
  float32[12] dist_m            # дистанция по сектору, метры (inf=свободно)
  int32[12]   freshness_steps   # 0=измерен в этот шаг, растёт пока servo не вернётся
  ```
  Fallback без нового msg: `std_msgs/Float32MultiArray` (24 значения: 12 dist + 12 freshness).

### 1.3 odom
- `twist.twist.linear` (vx,vy,vz world ENU) + `twist.twist.angular.z` (yaw_rate);
  `pose.pose.orientation` → heading. EKF-источник. **BEST_EFFORT** (подписка через
  `qos_profile_sensor_data`).

---

## 2. EXECUTOR ПРИНИМАЕТ ← политика + Interface

**ОДИН эндпоинт. Никаких дискретных действий / snap / settle.**

| Топик | Тип | QoS | Частота | Статус |
|---|---|---|---|---|
| `/drone/cmd_vel_body` | `geometry_msgs/msg/Twist` | RELIABLE / VOLATILE | 10 Hz | `[НОВОЕ]` |

- **Continuous velocity в BODY frame:**
  - `linear.x` = vx (вперёд+, м/с)
  - `linear.y` = vy (влево+, м/с)
  - `angular.z` = yaw_rate (CCW+, рад/с)
  - (`linear.z` = vz опционально; по умолчанию 0 — высоту держит executor)
- **10 Hz** поток (как maintenance-стрим GUIDED; пропуск > N тиков → executor тормозит в 0).
- Один и тот же эндпоинт для политики и для Interface (ручное управление = тот же Twist).

### 2.1 Safety-слой — КЛЭМП, не блок
- Перед выдачей в `/mavros/setpoint_raw/local` safety проверяет proximity (ToF + tfluna):
  - если вектор скорости ведёт в препятствие (proximity-constraint нарушен) →
    **уменьшает** компоненту/масштаб вектора (вплоть до 0 по опасной оси), НЕ отбрасывает
    команду и НЕ переключает режим.
  - чем ближе стена — тем сильнее клэмп (плавно, не порог). Перпендикулярный/отворачивающий
    компонент скорости НЕ режется (можно уходить от стены).
- Результат: дрон **не может влететь в стену**, но всегда отзывчив на команду (летит вдоль
  стены, тормозит к ней, свободно уходит). Политика/Interface видят реальную (склэмпленную)
  скорость в odom — parity по факту движения.

### 2.2 Высота — ползунок (Aleks 2026-06-11)

Высота управляется **дискретной командой** (НЕ live-follow): Interface даёт горизонтальный
ползунок рядом со взлётом, **минималка 0.5 м, максималка 2.2 м**. Пилот ставит ползунок →
Interface шлёт целевую высоту **на commit** (отпускание/отрисовка, не на каждое движение
мыши). Система **лочит контроль высоты**: на время вертикального выхода горизонт+yaw из
`/drone/cmd_vel_body` заморожены (дрон идёт ровно вверх/вниз), по достижении (`|z−H|≤0.08 м`)
**отдаёт контроль** обратно. Взлёт тоже «с указанием высоты» — поднимается на текущий target.

**Interface → executor:**

| Топик | Тип | QoS | Когда | Статус |
|---|---|---|---|---|
| `/drone/set_altitude` | `std_msgs/msg/Float64` | RELIABLE / VOLATILE | на commit ползунка | `[НОВОЕ]` |

- `data` = целевая высота, м (ENU z над точкой взлёта). Executor **клампит в [0.5, 2.2]** (защита).
- Дискретно: одно сообщение = одна команда «выйди на H и держи». Спамить на каждый пиксель НЕ надо.

**Executor → Interface (статус ползунка):**

| Топик | Тип | QoS | Частота | Статус |
|---|---|---|---|---|
| `/drone/altitude_locked` | `std_msgs/msg/Bool` | RELIABLE / VOLATILE | 5 Hz | `[НОВОЕ]` |
| `/drone/altitude_target` | `std_msgs/msg/Float64` | RELIABLE / VOLATILE | 5 Hz | `[НОВОЕ]` |

- `altitude_locked=True` → идёт вертикальный выход, **горизонт-управление заблокировано**
  (Interface может показать «высота меняется…», подсветить ползунок). `False` → контроль возвращён.
- `altitude_target` = clamped target (echo) — Interface синхронизирует позицию ползунка.
- Текущую фактическую высоту Interface берёт из `/mavros/local_position/odom` (pose.z).

**Взлёт «с указанием высоты» (Space → высота ползунка) — РЕШЕНО (твой вариант 4):**
единый владелец взлёта = `manual_fly_node`. По умолчанию он НЕ делает auto-takeoff, а ждёт
триггер. Твой Space публикует высоту ползунка:

| Топик | Тип | Когда | Статус |
|---|---|---|---|
| `/drone/takeoff` | `std_msgs/msg/Float64` | Space-arm у Interface | `[НОВОЕ]` |

- `data` = высота взлёта, м (клампится в [0.5, 2.2]); `data ≤ 0` → дефолт `--alt` ноды (1.0 м).
- manual_fly: GUIDED → EKF-settle → arm → NAV_TAKEOFF на эту высоту → climb → hover → READY.
- **Climb-arrival переписан** под низкие высоты: climb done при `z ≥ 0.9·target` (было хардкод
  1.8 м — ломал высоты <1.8). Для целей ≥2.0 м поведение прежнее (1.8 м).
- Повторный `/drone/takeoff` в воздухе игнорируется (один взлёт). Высоту в полёте меняй
  через `/drone/set_altitude` (см. выше).
- ⚠ Прислать `/drone/takeoff` ПОСЛЕ того как стек/нода поднялись (`ros2 node list` видит
  `manual_fly`). Standalone-тест ноды без Interface: флаг `--auto-takeoff`.

---

## 3. Frame / единицы
- BODY frame: x вперёд, y влево, z вверх (REP-103). vx/vy м/с, yaw_rate рад/с.
- ToF/tfluna дистанции — метры, от сенсора (+0.1м mount до центра).
- odom — world ENU (mavros); executor конвертит cmd body→world для setpoint.

## 4. Изолированный instance (параллельно на D2)
[[project_d2_multi_instance_isolation]] — стек Interface не дерётся с RL-стеком на RTX 5070:
```
SITL_INSTANCE=2  GZ_PARTITION=ifc  ROS_DOMAIN_ID=2  MAVLINK_PORT=5780
```
(`MAVLINK_PORT = 5760 + 10·SITL_INSTANCE`). Live-bringup:
`help_scripts/launch.sh --full --mavros --no-autoscan --gui -w <world> -s ifc -log`.

## 5. Параметры стека (контекст)
`WP_YAW_BEHAVIOR=0`, `ATC_ANGLE_MAX=25°`, `GPS1_TYPE=1` (fake-GPS SITL), `GUID_OPTIONS=0`,
высота-цель 3.0м.

## 6. НЕ в этом контракте (старая архитектура)
Дискретный `execute(0..7)`, 15°-снапы поворота, `snap_to_yaw`, `_settle`, cell-step
трансляции, action-mask — **исключены**. Новая архитектура = только continuous velocity (§2).
RL-env Discrete(8) при необходимости остаётся отдельно для legacy-parity, но Interface на
него НЕ завязан.

_Сверено/спроектировано: 2026-06-10, ветка feature/pre-train-preparation. `[ЕСТЬ]` — live;
`[НОВОЕ]` — bridge реализует под новую архитектуру._
