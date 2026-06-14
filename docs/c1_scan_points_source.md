# C1 — источник реальных точек скана (sim-сторона, gen_0)

> **Статус:** DRAFT 2026-06-14 (simulation-сторона). Закрывает открытый **C1** из
> `rl-lab/docs/point_scan_generation_contract.md §7` и `interface/src/GUI_README.md §3.4`.
> Авторитет схемы Scan/Dot/`from_pose`/`bearing` — rl-lab `point_scan_generation_contract.md`
> (этот док — КАК sim её наполняет с живого стека). Реализация ноды — по go Aleks
> (правило «никаких правок на лету»; геометрия серво требует stand-verify).

## 0. Что отдаёт sim (C1)

С живого стека (Gazebo + SITL + sweep) sim производит **gen_0 `scan`-записи** — надмножество
текущего sweep, в формате rl-lab `scan_store/flight_log`:
1. точки скана → world `{x,y,z}` (вычислены в ДРЕЙФЕ, по odom-позе);
2. `from_pose` **двойной** — `odom` (что думал дрон) + `gt` (Gazebo-факт);
3. `bearing` на **каждой** точке (body-frame угол луча) + `servo_range`;
4. `origin` (`fan`/`precise`), `t` (sim-time), стабильные `id`.

## 1. Источники данных на стеке

| данные | топик / источник | тип | смысл |
|---|---|---|---|
| лучи fan-скана | `/drone/sweep/result` | `sensor_msgs/LaserScan` | агрегированный проход: `ranges[i]` @ угол `angle_min+i·angle_increment` |
| (альт.) дамп | `sweep_storage` NPZ | `(theta_target, range_m, t_rel, …)` | то же, но с явным servo-θ на сэмпл |
| луч precise | `/scan/sweep` | `sensor_msgs/LaserScan` | одиночный луч TF-Luna в текущем угле серво (`ranges[0]`) |
| **odom-поза** (дрейф) | `/mavros/local_position/odom` | `nav_msgs/Odometry` | EKF-оценка (дрейфит) — `from_pose.odom` |
| **gt-поза** (факт) | `/world/{world}/pose/info` | `geometry_msgs/PoseArray` (gz Pose_V) | Gazebo-истина; дрон = модель `iris_claudedrone` — `from_pose.gt` |
| servo θ | `/drone/sg90/cmd` (echo) / NPZ theta | `Float64` рад | угол серво на момент сэмпла |

⚠ `/world/{world}/pose/info` — `PoseArray` со ВСЕМИ моделями+саб-линками БЕЗ имён
(stand-verify: 73 поз на base_stand). gt-дрон = запись с **3D-минимумом расстояния до
odom (с Z!)** — корень дрона на z≈odom.z, саб-линки (sg90/ротеры) на XY≈0 но иных Z →
XY-match неоднозначен, 3D разводит. gt=null в real-world (там этого топика нет).

## 2. Геометрия: (servo θ, range) → world-точка

Выведено из `iris_claudedrone/model.sdf` (⚠ знак/zero **верифицировать на стенде** перед
закрытием — правило: SDF/sim-таск без Gazebo-смоука не закрывается):
- `sg90_arm` базовая поза `0 0 0.146 0 0 −1.5708` (yaw −π/2), `sg90_joint` ось `Z`, θ∈[0,π];
  сенсор `tf_luna_sweep` смотрит вдоль +x руки, поза в руке `0.025 0 0.005`.
- ⇒ **body-frame угол луча:**
  ```
  bearing_body = θ_servo − π/2          # REP-103 body: x=нос, y=влево, CCW+
  ```
  Проверка: θ=0 → −π/2 (право), θ=π/2 → 0 (нос ✅ = SCAN_PRECISE), θ=π → +π/2 (лево).
  `servo_range [0,π]` ⇒ body `[−π/2, +π/2]` (передняя полусфера, право→нос→лево).
- **mount:** сенсор +`MOUNT_FWD=0.025` м вдоль луча от центра дрона (XY), высота
  +`MOUNT_Z≈0.151` м (0.146 рука + 0.005 сенсор). Свип в гориз. плоскости (ось Z) → z точки
  постоянна = `z_pose + MOUNT_Z`.
- **world-точка (вид в дрейфе, по `from_pose.odom`):**
  ```
  φ_world = yaw_odom + bearing_body
  x = x_odom + (MOUNT_FWD + range)·cos(φ_world)
  y = y_odom + (MOUNT_FWD + range)·sin(φ_world)
  z = z_odom + MOUNT_Z
  ```
- **«Истинную» точку НЕ дублируем** (rl-lab D6): выводится из `from_pose.gt + bearing + range`
  тем же преобразованием с `gt`-позой. `range` восстановим из gen_0:
  `range = hypot(x−x_odom, y−y_odom) − MOUNT_FWD` — поэтому отдельное поле `range` не храним
  (как в схеме rl-lab; если картографу удобнее явное — добавлю по запросу).

## 3. Формат `scan`-записи (наполнение схемы rl-lab)

Sim эмитит одну `scan`-запись на проход (fan) / выстрел (precise):
```json
{"rec":"scan","id":"s12","t":13.0,"origin":"fan","servo_range":[0.0,3.1416],
 "from_pose":{"odom":{"x":2.55,"y":2.05,"z":1.80,"yaw":-0.18},
              "gt":  {"x":2.61,"y":2.09,"z":1.80,"yaw":-0.18}},
 "dots":[
   {"id":"s12d0","scan_id":"s12","t":13.0,"gen":0,"x":-1.20,"y":2.80,"z":1.95,
    "origin":"fan","bearing":-1.05,"trust":true,"cat":null,"source":"sensor"}
 ]}
```
- `bearing` = `bearing_body` (рад, body-frame: 0=нос). Для precise — один dot, `bearing≈0`,
  `servo_range=[0,0]` (или один угол).
- `source:"sensor"` для всех gen_0 от живого стека. (`source:"gt"` — отдельный привилегированный
  слой картографа из SDF, НЕ из C1-сенсора; см. rl-lab §6 D3.)
- `trust` (C4) — sim ставит дефолт `true`; правило trust (out-of-range/inf/настенный clip)
  предлагаю позже (открыто у rl-lab C4). inf/out-of-range лучи (range_min/max sweep_storage:
  0.2/8.0) → точку НЕ создаём (нет препятствия), не trust=false.
- `t` — sim-time от старта прогона (из `/clock`), монотонно; `meta.t0` = реал-тайм.

## 4. Per-scan vs per-point поза

- **fan (дрон висит, скан ~доли сек):** одна `from_pose` на скан (D6 default). Дрон в hover
  на время прохода → odom/gt берём один раз на старте прохода (или среднее).
- **continuous (дрон движется во время свипа):** дублируем `from_pose` в каждый Dot (D6).
  Для нашего sweep_storage continuous-режима (mode 4) — поза на каждый сэмпл по его `t_rel`
  (интерполяция odom/gt к времени сэмпла). Реализуемо; решим, нужно ли (зависит от того,
  будет ли RL сканировать в движении).

## 5. Реализация (по go Aleks)

Предлагаемая нода `scan_points_node` (drone_sim) ИЛИ расширение `sweep_storage_node`:
- подписка: `/drone/sweep/result` (или sweep-сэмплы) + `/mavros/local_position/odom` +
  `/world/{world}/pose/info` + servo-θ;
- на каждый завершённый проход/выстрел: собрать позы (odom+gt на момент скана), посчитать
  world-точки (§2), собрать `scan`-запись (§3);
- sink: эмит в общий поток `flight_log.jsonl` (C2, надмножество) — durable аппенд +
  (опц.) Redis live-канал, как договорено в rl-lab §4b. Writer формата = сторона
  interface (классы Store), sim отдаёт уже посчитанные `scan`-записи (ROS2-топик или
  прямой аппенд — согласуем с interface).

✅ **Gazebo stand-verify ПРОЙДЕН (2026-06-14, base_stand_12x12, свипы на земле + hover 1.2м):**
(1) `bearing_body` верен — нос=восток (θ=π/2→0), право=южн.стена(−6), лево=ГЛУХАЯ +Y центр-комнаты(1.5);
(2) world-точки ложатся на реальную геометрию (центр-комната ~1.5 + внешние стены ~6 через проёмы);
(3) `z` точек = odom.z+0.151 точно; gt-дрон находится 3D-матчем на земле и в воздухе;
(4) Δ(gt,odom)=0.9см в стабильном hover (dual-pose снимается; реальный дрейф — в динамическом полёте, TODO).
Визуал: `$DRONE_MEDIA_ROOT/sim/c1-stand-verify-2026-06-14/` (top-down PNG + mp4).

## 6. Открытые вопросы (к rl-lab / Aleks)

1. **Sink-механизм:** sim публикует `scan`-записи в ROS2-топик (interface-writer слушает и
   пишет `flight_log.jsonl`), ИЛИ sim сам аппендит в файл? Чей процесс владеет файлом на
   живом стеке (interface control_api держит писатель — логично отдать ему топик).
2. **range explicit?** хранить `range` полем или восстановление из x,y (см. §2) ок?
3. **trust-правило (C4):** дефолт `true` + drop inf/out-of-range — достаточно для gen_0?
4. **continuous per-point поза** (§4) — нужен ли RL-скан в движении в A.1, или только hover-скан?
