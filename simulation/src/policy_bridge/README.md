# policy_bridge

SWEEP-02 RL policy bridge — ROS2 ноды, которые соединяют Gazebo-сенсоры с
обученной PPO-политикой и переводят дискретные действия в MAVROS velocity
команды.

**Sprint:** model-to-sim-bridge Phase 1 (TASK-059).
**Status:** Phase 0 scaffold (2026-05-19).

## Источник модели

**PROD (v2 promote, Aleks 2026-06-08):**
`$RL_LAB_ROOT/export/activemapping_v1_v2/model.zip` — ActiveMapping-v1-v2
(MaskablePPO), md5 `40673767c00b121b27e92fdce9f4c077`, canonical seed2.
Alignment primary-гейт взят: no-travel **0%** в SITL (parity 0/44 bit-exact).
Дефолты launch: `model_family:=activemapping`, `v2_sensor_mask:=true`,
`min_frontier_cluster_cells:=3`, `wall_stop_cells:=6`.

Легаси (явными аргументами `model_path` + `model_family`):
- N6-v1 `export/activemapping_v1/model.zip` (md5 `1d7d7013`, no-travel 6.5%)
- SWEEP-02 `export/sweep02/model.zip` (md5 `49f459fd…`, `model_family:=sweep02`)

Specs interface: `obs_spec.md`, `action_spec.md`, `model_card.md` рядом с моделью.

## Архитектура

```
Gazebo → /drone/perimeter (Float32MultiArray[6] raw m)
       → /drone/altitude  (Float32 m)        ← sensor_monitor агрегатор
       → /mavros/local_position/odom         (nav_msgs/Odometry)
                                ↓
                       policy_bridge_node (10 Hz, Phase 1)
                       ├── ObsBuilder       — normalize → Dict obs
                       ├── VisitedGridBuilder — +room_size/2 offset
                       ├── PPO.predict()
                       └── ActionExecutor   — asymmetric timing
                                ↓
                       /mavros/setpoint_velocity/cmd_vel_unstamped
                       /rl_policy/{action, visited_grid, coverage}
```

## Python окружение

SB3 + torch установлены в **dedicated venv** `simulation/.venv-policy/` с
`--system-site-packages` (наследует ROS2 deps). Активируется wrapper'ом
в launch файле (см. `launch/policy_bridge.launch.py`).

## Конвенции

- **bbox миров:** 6.4 × 6.4 m, `cell_size = 0.1 m`, grid 64 × 64.
- **VisitedGridBuilder:** `iy = int((y + 3.2) / 6.4 * 64)`, `ix = int((x + 3.2) / 6.4 * 64)`.
- **VL53L0X norm:** `dist_m / 1.2` (max range 1.2 m).
- **TF-Luna norm:** `dist_m / 6.4` (max range 6.4 m).
- **Failure default:** `hover_and_wait`, **не land**.

См. `obs_spec.md` / `action_spec.md` / `model_card.md` от rl-lab для точного формата.

## Build

```bash
cd $AEROSEARCH_ROOT/claudedrone-git/simulation
colcon build --symlink-install --packages-select policy_bridge
source install/setup.bash
```

## Phase 0 субтаски (TASK-059 0a..0e)

- ✅ 0a. package scaffold
- ⬜ 0b. policy_bridge.launch.py с defaults
- ⬜ 0c. venv-policy + SB3 install
- ⬜ 0d. rosbag2 helper → `$DRONE_MEDIA_ROOT/sim/bags/model-to-sim/`
- ⬜ 0e. SG90 joint в `iris_claudedrone/model.sdf` (отдельно от пакета, но в scope TASK-059)
