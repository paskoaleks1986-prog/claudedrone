# policy_bridge

SWEEP-02 RL policy bridge — ROS2 ноды, которые соединяют Gazebo-сенсоры с
обученной PPO-политикой и переводят дискретные действия в MAVROS velocity
команды.

**Sprint:** model-to-sim-bridge Phase 1 (TASK-059).
**Status:** Phase 0 scaffold (2026-05-19).

## Источник модели

`$RL_LAB_ROOT/export/sweep02/model.zip` (canonical SWEEP-02 seed22).
md5 `49f459fd018b0fe484ecbe51a4b0e217`. Загружается `PPO.load("model.zip",
device="cpu")` из stable-baselines3.

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
