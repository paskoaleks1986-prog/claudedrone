# SIM ↔ RL ↔ INTERFACE — Data contract v5-krot Этап-1 (Sim-сторона)

> **Ветка:** `V5_A1` · **Автор:** simulation · **Дата:** 2026-06-24 · **Канон:** `v5/planing/v5_krot_pilot_stage1_surface_contract_v2.md` (researchbest) + ответ research HANDOFF 12:4x.
> **Что это:** что Sim ПУБЛИКУЕТ и ПОТРЕБЛЯЕТ для пилота-этапа-1 (wall-follow). researchbest владеет контрактом ДАННЫХ (поля), **имена топиков — proposal Sim, финал согласует interface** (зона interface). RL/interface читают это, чтобы подключиться.

---

## 0. Архитектура мостов (развязка носа от политики)

```
 gz VL53×6 (/drone/vl53l0x/ch0..5, LaserScan)
        │
        ▼  tof_ring_node
   /krot/tof_ring  ──────────────────────────────▶  RL: строит obs[0:6] tof + [6:12] hit
                                                     │
 RL-ПИЛОТ (политика, obs→action) ──▶ /krot/cmd_vel_world (2D МИР-скорость) ─┐
                                                                            │
 gz GT-поза (odom≈GT на SITL) ─▶ wall_gt_node ─▶ /krot/wall_gt[12] ─┬──────┼─▶ vel_setpoint_mux ─▶ /mavros/setpoint_raw/local
   (privileged: d_perp,t̂,n̂,validity,min_obs,поза,vel)              │      │
                                                                    └─▶ yaw_slave_node ─▶ /krot/yaw_cmd ─┘
                                                                        (нос слейв к касательной)
   /krot/wall_gt ──────────────────────────────────────────────────▶  RL-критик (privileged) + reward тейк-4
   /krot/wall_gt, /krot/tof_ring ──────────────────────────────────▶  interface (визуал standoff/нормаль/min_obs)
```

**Ключ:** RL-пилот публикует ТОЛЬКО чистую 2D мир-скорость. Нос ведёт `yaw_slave_node` (слейв к
касательной из GT). `vel_setpoint_mux` сводит их в один `PositionTarget`. Так RL **структурно не может**
рулить носом (контракт §3: yaw вне политики; убирает краб на корню).

---

## 1. Топики (proposal-имена — согласовать с interface)

| Топик | Тип | Publisher | Subscribers | Назначение |
|---|---|---|---|---|
| `/drone/vl53l0x/ch0..5` | `sensor_msgs/LaserScan` | gz (модель) | tof_ring_node | сырьё 6×ToF |
| `/krot/tof_ring` | `std_msgs/Float32MultiArray`[6] | **tof_ring_node** | RL, interface | 6 лучей, канон-порядок, метры, клип 1.2 |
| `/krot/wall_gt` | `std_msgs/Float32MultiArray`[12] | **wall_gt_node** | RL (критик+reward), interface | privileged GT-геометрия стены |
| `/krot/cmd_vel_world` | `geometry_msgs/Twist` | **RL-пилот** | vel_setpoint_mux | 2D мир-скорость (linear.x/y), м/с |
| `/krot/yaw_cmd` | `std_msgs/Float64` | **yaw_slave_node** | vel_setpoint_mux | абс. yaw носа (мир, рад) |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | **vel_setpoint_mux** | ArduPilot | финальный setpoint |
| §5 сигналы (`standoff_err`/`LOST`/`GRAZE`) | interface owns | **RL-пилот** | interface | косметика ГУИ (НЕ Sim) |

> ⚠️ `/krot/*` — рабочие proposal-имена Sim. interface: подтверди/переименуй, я подстрою (все имена — ROS-параметры нод, не хардкод).

---

## 2. `/krot/tof_ring` — obs-фид (Float32MultiArray, 6 float)

Порядок = `sensor_angles_deg` из `drone_geometry.yaml` = **[0,60,120,180,240,300]°**, index0 = +x (нос), CCW.
Единицы — **метры**, клип `[tof_min=0.03, tof_clip=1.2]`. no-hit (inf / >clip) → **1.2** (sentinel «чисто до предела»).

**RL строит из этого** (контракт §2):
- `obs[0:6]  tof[6]` = `tof_ring / 1.2` → [0,1] (no-return → 1.0)
- `obs[6:12] hit[6]` = `1.0 if tof_ring < 1.2 else 0.0`

PARITY: RL ОБЯЗАН читать лучи в этом порядке. Раскладка зашита в модели `iris_claudedrone` (vl53l0x_0..5).

---

## 3. `/krot/wall_gt` — privileged GT (Float32MultiArray, 12 float)

Считается **аналитически** из `*.arena.yaml` + истинной позы, БЕЗ сенсор-шума (шум живёт в obs/DR).
Источник reward тейк-4 (контракт §4) и асимметричного критика.

| idx | поле | смысл |
|---|---|---|
| 0,1,2 | x, y, yaw | истинная поза (мир) |
| 3,4 | vx, vy | истинная скорость (мир) |
| 5 | **d_perp** | дист до ближайшей followable-стены (м, кадр мира; абс — дрон внутри арены) |
| 6,7 | **t_x, t_y** | касательная (unit, мир) |
| 8,9 | **n_x, n_y** | нормаль (unit, мир, стена→дрон) |
| 10 | **validity** | 1.0 если стена в `r_usable` (1.2м) иначе 0.0 |
| 11 | **min_obstacle_dist** | до ЛЮБОГО препятствия (центр дрона) — collision/no-graze floor |

**Reward тейк-4 (RL считает из этого):** `v_t = v·t̂`, `v_n = v·n̂`; `v_t* = follow_dir·v_cruise`;
`v_n* = clip(−Kp·(d_perp − d*))`; члены −w_t|v_t−v_t*| −w_n|v_n−v_n*| −w_d|d_perp−d*| + hard no-graze при `min_obstacle_dist < d_graze`. Веса/Kp — на P0 (контракт §8 seed).

**Поза:** текущий источник `/mavros/local_position/odom` (на SITL fake-GPS odom≈GT ~1см, memory
`project_sitl_fakegps_near_zero_drift`). Для строгого GT — бриджить gz `/world/<w>/pose/info` (param `pose_topic`).
**d_perp side:** v1 = ближайшая стена (side-agnostic); командуемую сторону применяет RL. Уточним при P1.

---

## 4. `drone_geometry.yaml` (config/, владелец Sim)

Единый источник физики, **пороги ФОРМУЛАМИ** (контракт §1). RL читает тот же файл (`drone_sim/geometry.py` = удобный loader, или сырой yaml).

ЗАМЕРЫ по `iris_claudedrone` (2026-06-24): `motor_arm=0.256`, `sensor_ring_r=0.10`, `prop_radius=0.063`[REAL].
Формулы → `prop_tip=0.319`, `prop_tip_beam=0.219`, `d_graze=0.279`, `d_star_min=0.329`, target `d*=0.35`.

🔴 **РАСХОЖДЕНИЕ vs контракт-заглушки (0.17/0.09 → 0.233/0.203/0.253):** реальная iris БОЛЬШЕ →
полоса жёстче (d*_min 0.25→0.33, запас target→GRAZE 0.147→0.071). Решение за researchbest
(принять на iris / поднять target / geometry-DR). Реальный дрон МЕНЬШЕ → запас вернётся.
**Своп на железо = править 3 строки yaml, код/тренинг не трогать.**

---

## 5. Арены (`worlds/v5_krot/<name>/`, владелец Sim)

| Файл | Содержимое |
|---|---|
| `<name>.sdf` | gz-мир (без `<gui>` = свободная камера), спавн `iris_claudedrone` |
| `<name>.arena.yaml` | GT-дескриптор для wall_gt_node: `followable[]`, `obstacles[]`, `r_usable` |
| `<name>.occupancy.npz` | карта: `occupancy`(uint8,1=wall), `resolution`(0.1), `origin`(SW), `shape` |

- **P0 `p0_straight_wall`** — прямая стена x=2 длина 8м; спавн (1.65,−3.5) standoff 0.35 нос +Y; финиш=прошёл длину.
- **P2 `p2_circus`** — замкнутый круг R=4.5 (полигон 36 сегм); спавн у стены; финиш=обход (угол-бюджет 2π).
- **P1** = мир P0 + эпизод-параметры (рамп d*[0.25..0.55] + рандом командуемого носа) — отдельный SDF не нужен.
- Генератор: `help_scripts/gen_worlds_v5.py`. P3 волна / P4 зигзаг / P5 зубцы — следующий заход (по curriculum).

`*.arena.yaml` формат стены: `{type: segment, p1:[x,y], p2:[x,y]}` или `{type: circle, center:[x,y], radius:R, inside:true}`.

---

## 6. Разделение обязанностей

| | Sim (этот контракт) | RL-lab | interface |
|---|---|---|---|
| sensors | tof_ring (канон 6×ToF) | строит obs[31] | визуал лучей |
| GT | wall_gt (d_perp/t̂/n̂/validity/min_obs/поза/vel) | критик+reward тейк-4 | визуал standoff/нормаль |
| action | vel_setpoint_mux (склейка→MAVROS) | политика → /krot/cmd_vel_world | — |
| нос | yaw_slave_node (слейв к касат.) | НЕ трогает | визуал курса |
| геометрия | drone_geometry.yaml + замеры | читает, считает пороги | — |
| арены | SDF+occupancy+GT-дескриптор | тренинг P0→ | визуал карты |
| §5 сигналы | — (даём GT) | публикует standoff_err/LOST/GRAZE | рисует лампочки/шкалу |
| takeoff/стек | launch+takeoff+kill | запуск прогонов* | viewer-bridge |

\* кто драйвит стек/прогоны на стенде — уточнить с Aleks (в v4 был спор очереди sim↔rl).

---

## 7. Что СДЕЛАНО (V5_A1) и что PENDING

**Готово (собрано/верифицировано без стенда):** фильтрация v4 (`refactor` 3a3d44e); geometry.yaml+loader; 4 ноды
(`feat` 9276955, colcon build OK, формулы/импорт верифицированы); арены P0/P2 (`feat` bf48a23, gz sdf
структурно чист, GT-математика d_perp=0.35/n̂/t̂ верна).

**PENDING (нужен стенд gz+SITL — Sim НЕ поднимает без слова Aleks):**
- runtime-smoke: gz up → entities/топики → tof_ring/wall_gt публикуют → vel_setpoint_mux армит и держит standoff;
- obs-parity 1 кадр sim↔rl (как в v4 — RL даёт dump);
- live-валидация полёта P0 (oracle warm-start, дрон держит d*=0.35 вдоль стены).

**Открытые вопросы (→ research/interface):**
1. researchbest: принять полосу на iris (target 0.35 vs d*_min 0.329) или скорректировать?
2. interface: финал имён топиков `/krot/*`; §5-сигналы публикует RL-пилот — взять `/krot/wall_gt`+tof для визуала?
3. RL: контракт `/krot/cmd_vel_world` (Twist world) ок? obs[31] строишь из tof_ring+wall_gt+EKF — нужен ли тебе ещё какой sim-фид?
4. occupancy.npz формат (npz: occupancy/resolution/origin/shape) — подходит RL/interface?
