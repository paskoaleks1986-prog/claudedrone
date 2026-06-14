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

---

## 7. TF-Luna sweep — режимы серво (GUI radio-button, Aleks 2026-06-11)

Цепочка железа: `…/sg90/target_angle → servo_cmd_node (клемп 0..π) → /drone/sg90/cmd
→ ros_gz_bridge → JointPositionController (model.sdf) → серво едет → TF-Luna на arm → /scan/sweep`.
TF-Luna `/scan/sweep` (LaserScan, 10 Гц) **всегда** выдаёт ОДИН луч в текущем угле серво —
«sweep» создаётся движением серво, а его задаёт отдельная нода.

**VL53 ×6 + TF-Luna down — потоковые ВСЕГДА** (gz_bridge, не зависят от режима sweep):
`/mavros/vl53_ch0..5`, `/drone/vl53l0x/ch0..5`, `/drone/tf_luna_down` (LaserScan 10 Гц).

### Топики (типы подтверждены live 2026-06-11, стек session `sim`)
| топик | тип | напр. | смысл |
|---|---|---|---|
| `/drone/sweep/start` | `std_msgs/Empty` | GUI→ | триггер разового прохода (mode 2 / 4) |
| `/drone/sweep/stop` | `std_msgs/Empty` | GUI→ | стоп цикла (autoscan / sweep_storage; sweep_node НЕ слушает) |
| `/drone/sweep/resume` | `std_msgs/Empty` | GUI→ | снять стоп (только autoscan) |
| `/drone/sg90/target_angle` | `std_msgs/Float64` | GUI→ | прямой угол серво 0..π рад (mode 5) |
| `/drone/sg90/cmd` | `std_msgs/Float64` | внутр. | выход servo_cmd → gz (GUI сюда НЕ пишет) |
| `/scan/sweep` | `sensor_msgs/LaserScan` | →GUI | TF-Luna одиночный луч @10 Гц (ranges[0]) |
| `/drone/sweep/result` | `sensor_msgs/LaserScan` | →GUI | агрегированный проход (mode 2/3/4) |
| `/drone/sweep/progress` | `std_msgs/Float32` | →GUI | прогресс прохода 0..1 (mode 2) |
| `/scan/status` | `std_msgs/String` | →GUI | `SCANNING` / `COMPLETE` / `STOPPED` |

### Режимы радиокнопки
| # | режим | как включить | runtime-переключаемо? | нода |
|---|---|---|---|---|
| 1 | **Статичный вперёд (90°)** | `Float64(π/2)` → `/drone/sg90/target_angle` (или ничего — дефолт после взлёта) | ✅ да | servo_cmd_node |
| 2 | **Разовый sweep** | `Empty` → `/drone/sweep/start` | ✅ да (нода уже жива) | sweep_node |
| 3 | **Autoscan** (авто-цикл) | relaunch `autoscan:=true`, ЛИБО GUI сам шлёт start по таймеру | ⚠ нода — нет; эмуляция — ✅ | autoscan_node |
| 4 | **Непрерывный треугольный + NPZ** | relaunch `sweep_storage:=true` (заменяет sweep_node) | ❌ нужен restart | sweep_storage_node |
| 5 | **Прямое управление серво** | ползунок → `Float64(0..π)` → `/drone/sg90/target_angle` | ✅ да | servo_cmd_node |

### Параметры режимов (дефолты)
- **mode 2 (sweep_node):** step 1° (`step_rad`), settle 120 мс (`settle_ms`) → **~181 шаг ≈ 21.7 с/проход**. ⚠ для живого облёта медленно — можно step 3–5°, settle 60 мс.
- **mode 3 (autoscan_node):** `initial_delay_s=5.0`, `cooldown_s=10.0`. Стоп/резюм рантайм: `/drone/sweep/stop`÷`/resume`.
- **mode 4 (sweep_storage_node):** `cycle_period_s=9.0` (триангл 0→π→0), `cmd_rate_hz=50`, `output_mode=laserscan|pointcloud2`, `n_bins=90`, NPZ-дамп в `$RESEARCHBEST_ROOT/output_data/TASK-047/`.

### ⚠ Грабли для GUI
1. **Режимы 3 и 4 НЕ переключаются радиокнопкой на лету** — это launch-флаги (нужен рестарт стека). 1/2/5 — чистые pub'ы, мгновенно.
2. **Mode 3 лучше эмулировать в GUI/bridge** без рестарта: `sweep_node` уже жив → GUI шлёт `Empty`→`/drone/sweep/start` по таймеру (start, ждать `COMPLETE` на `/scan/status`, через cooldown повторить). Даёт autoscan-поведение на текущем стеке.
3. **Разовый sweep (mode 2) НЕ прерывается** — у `sweep_node` нет `/stop`. Переключение с mode 2 в середине прохода (~22 с) не остановит серво до конца. Учитывать в UI (disable радио пока `SCANNING`, или ждать `COMPLETE`).
4. **Mode 4 (sweep_storage) конфликтует с sweep_node** — оба слушают `/drone/sweep/start` и водят серво. Запускать ТОЛЬКО один (launch это и делает: sweep_storage ВМЕСТО sweep_node).
5. **Владение серво:** в ручном `manual_fly` executor ставит серво в 90° лишь ОДНАЖДЫ на взлёте, далее не трогает → серво свободно для GUI/sweep. Конфликта нет, **пока GUI не шлёт target_angle и sweep одновременно** (радиокнопка = один источник).
6. **Mount-offset:** дистанция до стены от ЦЕНТРА дрона = `ranges[0] + 0.1 м`.

_Live-проверка 2026-06-11 (стек session `sim`, `--no-autoscan`): подняты `servo_cmd_node`, `sweep_node`,
`distance_sensor_forwarder`; `autoscan_node`/`sweep_storage_node` — НЕ запущены. Типы топиков сверены `ros2 topic type`._

### 7.1 Live-включение mode 3/4 БЕЗ рестарта стека (simulation 2026-06-11, ветка `v3`)

ROS2-ноды можно добавлять в живой граф. Под радиокнопку добавлены параметры
(дефолты сохраняют прежнее поведение, симлинк-инсталл → правки живут без rebuild):
- `sweep_node`: **новый sub `/drone/sweep/stop` (Empty)** — mode 2 теперь прерываемый (раньше проход ~21.7с был неотменяем). ⚠ Оживёт только после рестарта sweep_node (нода уже крутится со старым кодом).
- `autoscan_node`: **param `autostart` (bool, default true)** — `false` → нода поднимается в STOPPED, цикл стартует по `/drone/sweep/resume`, пауза `/drone/sweep/stop`.
- `sweep_storage_node`: **param `loop` (bool, default false)** — `true` → после каждого триангл-цикла стартует следующий (непрерывный mode 4) до `/…/stop`. Топики уже параметризуемы (start/stop/output/status/cmd).

**Mode 3 (autoscan) — spawn idle в живой стек, без рестарта:**
```
ros2 run drone_sim autoscan --ros-args -p autostart:=false -p cooldown_s:=10.0
```
GUI: выбрал mode 3 → `Empty`→`/drone/sweep/resume` (autoscan гонит `/drone/sweep/start` → существующий sweep_node метёт циклом). Снял → `Empty`→`/drone/sweep/stop`. Статус — `/scan/status`.

**Mode 4 (sweep_storage) — spawn на ОТДЕЛЬНЫХ топиках (не дерётся с sweep_node за серво):**
```
ros2 run drone_sim sweep_storage --ros-args \
  -p loop:=true -p cycle_period_s:=9.0 \
  -p cmd_topic:=/drone/sg90/target_angle \
  -p start_topic:=/drone/sweep/tri/start \
  -p stop_topic:=/drone/sweep/tri/stop \
  -p output_topic_laserscan:=/drone/sweep/tri/result \
  -p status_topic:=/scan/tri/status \
  -p frame_id:=sg90_arm
```
GUI: выбрал mode 4 → `Empty`→`/drone/sweep/tri/start` (непрерывный триангл). Снял → `Empty`→`/drone/sweep/tri/stop` (парковка серво в 0). Результат `/drone/sweep/tri/result` (LaserScan, n_bins=90), статус `/scan/tri/status`. cmd идёт через `target_angle`→servo_cmd (клемп 0..π).

⚠ **Один источник угла за раз** — радиокнопка гарантирует, что активен ровно один режим; sweep_node(`/drone/sweep/start`), autoscan и sweep_storage(`/drone/sweep/tri/*`) разведены по топикам, но физически серво одно. При смене режима GUI должен сначала остановить текущий (stop старого), потом включить новый.

⚠ **Не протестировано на стенде** (sim не поднимает стек сам). py_compile OK; проверить при ближайшем bringup. Альтернатива spawn'у — могу добавить флаг `sweep_modes:=true` в `drone.launch.py` (предзагрузка всех провайдеров idle одним стеком) — скажи, если так удобнее.

---

## 8. ТРЕК-1 — детерминизм shared velocity + watchdog (RL дискрет-обёртка A.1, S1-S3)

> Запрос rl-lab (HANDOFF 2026-06-14 13:45) под `navigator_rl_node` / `Discrete(11)`: дискрет-слой
> строит rl-lab поверх ТОЙ ЖЕ velocity-ноды; ручной полёт interface остаётся континуальным (Aleks).
> Этот раздел **фиксирует контракт по факту кода** (`v3`, `policy_bridge/action_executor.py` +
> `manual_fly_node.py`). Инфраструктура S1-S3 **уже существует** — раздел сверяет детерминизм/частоты,
> не вводит новый код. ⚠ Один пункт «на ревью» (см. S1, легаси velocity-путь) — runtime не трогаю
> без go (правило «никаких правок на лету»).

### 8.1 S1 — shared velocity-нода: source-agnostic + детерминизм + watchdog

**Source-agnostic ✅.** Единый вход `/drone/cmd_vel_body` (`geometry_msgs/Twist`, BODY: `linear.x`=vx
вперёд+, `linear.y`=vy влево+, `angular.z`=yaw_rate CCW+). Подписчик — `manual_fly_node`
(`set_manual_velocity` → executor). Любой источник взаимозаменяем ВЫШЕ контроллера: ручной GUI
(`/teleop/velocity`→`cmd_vel_body`), `teleop_keyboard`, RL `navigator_rl_node` — пишут в один топик.
На стек — ОДИН executor (`manual_fly_node`); арбитраж «кто за рулём» (manual↔RL mux + приоритет
BRAKE/`/emergency`) — фланг interface (control_api), не sim.

**Клэмпы (детерминированная сатурация, НЕ сглаживание).** `set_manual_velocity` клампит:
`vx,vy ∈ [−0.5, +0.5] м/с` (`V_MAX_MS=0.5`), `yaw_rate ∈ [−1.0, +1.0] рад/с` (`W_MAX_RAD_S=1.0`).
RL-константы амплитуд ДОЛЖНЫ лежать в этих границах — за ними команда молча насыщается. Это чистая
сатурация (idempotent), не ramp.

**Детерминизм отклика (S1.б) — путь `manual_flight`.** После взлёта `manual_fly_node` зовёт
`enter_manual_flight()` → executor в режиме `_manual_flight=True`. Активный путь `_publish_manual_flight`:
- удержанный вектор публикуется **СЫРЫМ** — `velocity.x=vx`, `velocity.y=vy`, `yaw_rate` напрямую.
  **Нет ramp, нет сглаживания, нет safety-клэмпа** (Aleks 2026-06-11 «как на пульте» — вся safety
  из ручного пути вырезана);
- `coordinate_frame = FRAME_BODY_OFFSET_NED` → **единственный** поворот body→world делает САМ AP по
  своей оценке курса. ⚠ RL/источник **НЕ вращает vx,vy руками** (двойное вращение = баг 538b16d,
  фикс 11f0971);
- `vz` — НЕ из vx/vy: P-регулятор высоты `vz = clamp(Kp·(target_alt − z), ±0.5)`, `Kp=MANUAL_Z_KP=0.8`,
  ведёт к `target_altitude` (см. S2);
- ⇒ отклик на удержанный `(vx,vy,yaw_rate)` детерминирован на уровне setpoint; форму придаёт лишь
  внутренний velocity-PID ArduPilot (часть «планта», одинаков для всех источников/прогонов).

⚠ **Развилка двух velocity-путей (важно для parity, S1.б).** В executor ДВА пути:
| путь | когда активен | поведение |
|---|---|---|
| `_publish_manual_flight` | `_manual_flight=True` (после взлёта через `manual_fly_node`) | **чистый** BODY_OFFSET_NED, без клэмпа/ramp ← **детерминированный путь для RL** |
| `_publish_manual_velocity` | `_manual_vel` задан при `_manual_flight=False` (легаси FlightRL-v1) | safety-клэмп B4 (`_clamp_velocity_body`) + body→world поворот РУКАМИ (FRAME_LOCAL_NED) — скрытый клэмп + старый double-rotation footgun |

⇒ Для паритета RL-обёртка обязана гонять команды **через тот же `manual_fly_node` в режиме
`manual_flight`** (т.е. publish в `/drone/cmd_vel_body`), а НЕ поднимать второй executor, который
сядет на легаси-путь. На живом стеке легаси-путь фактически мёртв (manual_fly всегда входит в
`manual_flight`). **Рекомендация «на ревью»:** удалить/загейтить `_publish_manual_velocity`, чтобы
ambiguity исчезла физически — отдельным коммитом по go Aleks, не на лету.
**Поправка к §2.1:** safety-клэмп из §2.1 описывает ЛЕГАСИ-путь; живой `manual_flight`-путь клэмпа
НЕ имеет. Это СОВПАДАЕТ с train-env RL (нет скрытого клэмпа; `safety_guard` — отдельная нода,
OFF на fine-tune, [[project_safety_guard_off_for_rl_finetune]]).

**Watchdog / частота OFFBOARD-стрима (S1.в):**
- **Частота setpoint-стрима = `MAINTAIN_RATE_HZ = 10.0 Гц`** — внутренний maintenance-таймер executor
  сам стримит setpoint в `/mavros/setpoint_raw/local` @10 Гц (обязательно для ArduPilot GUIDED). RL
  **НЕ обязан** стримить 10 Гц сам: он задаёт вектор, стримит executor.
- **Таймаут свежести команды = `MANUAL_VEL_TIMEOUT_S = 0.5 с`.** Опубликованный `cmd_vel_body`
  «свежий» 0.5 с; нет новой команды дольше → failsafe: POSITION-hold hover (тормоз в 0).
- ⇒ RL `action_hold` = 3 тика @10 Гц = **0.3 с < 0.5 с ✅** — одна команда держится все 3 тика без
  срабатывания failsafe. Для удержания дольше — RL пере-публикует ДО истечения 0.5 с. Рекоменд:
  RL шлёт ≥2 Гц (запас), идеально 10 Гц (чёткие переходы между действиями).

### 8.2 S2 — UP/DOWN = ±Δ к `set_altitude` (position-step, ПОДТВЕРЖДЕНО)

`/drone/set_altitude` (`std_msgs/Float64`, м ENU над точкой взлёта) — детерминированный position
set-point (см. §2.2). `set_target_altitude` клампит в **[`ALT_MIN_M=0.5`, `ALT_MAX_M=2.2`]**, ставит
`target_altitude`. Контроллер держит x,y и ведёт z (в hover — AP-position-контроллер; в полёте —
`vz=Kp·Δz`, ±0.5). Возврат контроля по `|z−target| ≤ ALT_ARRIVE_TOL_M = 0.08` (`altitude_locked=False`).
- **RL дискрет ±Δ:** прочитать текущий target (echo `/drone/altitude_target` @5 Гц), опубликовать
  `target±Δ` (re-clamp [0.5,2.2]). **Величина Δ — выбор rl-lab (parity-таблица), sim её не фиксирует.**
- `vz` НЕ велосити-вход — `manual_fly` игнорит `linear.z` Twist; вертикаль только через `set_altitude`.
  Подтверждаю: устраивает (как и просил rl-lab).

### 8.3 S3 — скан-нода: беспараметрический дефолт-триггер (ПОДТВЕРЖДЕНО — уже есть)

- **SCAN_FAN (action 9):**
  - **RL fast (S3, готово 2026-06-14) → `/drone/sweep/start_fast` (`std_msgs/Empty`)** —
    беспараметрический **быстрый** проход: coarse `fast_step_rad=5°` / `fast_settle_ms=60`
    ≈ **37 шагов ≈ ~3.7 с/проход** (≈6× быстрее fine). Это дефолт-триггер RL SCAN_FAN.
  - **Ручной GUI fine → `/drone/sweep/start` (`std_msgs/Empty`)** — `step 1°/settle 120 мс`
    ≈ 181 шаг ≈ ~21.7 с/проход (качество картографа). **Не трогается** RL-триггером — отдельные
    топики, один и тот же серво, но конфигурации независимы.
  - Оба метут `SWEEP_MIN_RAD=0.0 .. SWEEP_MAX_RAD=π` (полный веер). Результат → `/drone/sweep/result`
    (`LaserScan`, `angle_increment` = активный step), статус `/scan/status` (`SCANNING`→`COMPLETE`).
  - ⚠ TF-Luna 10 Гц → settle<100мс упирается в freshness-guard (≈100мс/шаг минимум); реальный
    выигрыш fast-режима — от МЕНЬШЕГО числа шагов, не от settle.
- **SCAN_PRECISE (action 10) → фикс-выстрел по носу servo=π/2:** publish `Float64(π/2)` →
  `/drone/sg90/target_angle` (servo_cmd_node клампит [0,π]). Дистанция — `ranges[0]` с `/scan/sweep`
  (TF-Luna в текущем угле серво) + 0.1 м mount-offset до центра. Беспараметрично ✅.
  - ⚠ **Открыто:** нет атомарной ноды «навёл→устаканил→снял один результат» для precise (sweep_node
    делает полный проход). Два варианта детерминированной одиночной отдачи: **(a)** RL шлёт π/2, ждёт
    settle (~120-200 мс), читает свежий `/scan/sweep` — без новой ноды; **(b)** добавить мини-триггер
    (`Empty`), который наводит серво π/2, устаканивает и эмитит один `/drone/sweep/result` с одним bin
    (единый формат сообщения). Рекоменд **(a)** сейчас; **(b)** — если rl-lab нужен единый shape
    результата. Решаем в parity-раунде.
- `bearing` каждой точки + `servo_range` результата → стыкуется с **C1** (точки-сторона, отдельный трек).

_Сверено по коду `v3` (`action_executor.py`, `manual_fly_node.py`, `sweep_node.py`, `servo_cmd_node.py`),
2026-06-14. Runtime-поведение НЕ изменено (только документация). Реализация любых правок (RL-дефолт
скана, чистка легаси velocity-пути) — отдельными коммитами по go Aleks._
