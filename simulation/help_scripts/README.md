# help_scripts

Build and launch helpers for the `simulation` ROS2 workspace.

## Requirements

- bash, tmux
- ROS 2 Jazzy
- Gazebo Harmonic (`gz sim`)
- ArduPilot SITL (`sim_vehicle.py`) and `ardupilot_gazebo`
- Optional: MAVProxy (for `--manual`), `watch` (for `--monitor`)

## Setup

The env file lives **outside the repo**, one level above:

```
/home/aleks/git/proj/aerosearch/.env_simulation     # NOT committed
/home/aleks/git/proj/aerosearch/git/simulation/help_scripts/.env_simulation.example  # committed
```

Copy and edit:

```bash
cp git/simulation/help_scripts/.env_simulation.example .env_simulation
$EDITOR .env_simulation
```

To use a different env file: `ENV_FILE=/path/to/env ./build.sh`.

## Flags

### build.sh

| Flag         | Description                                    |
|--------------|------------------------------------------------|
| (none)       | `colcon build --symlink-install`               |
| `-p NAME`    | build only that package (`--packages-select`)  |
| `--clean`    | wipe `build/ install/ log/` first              |
| `--no-sym`   | drop `--symlink-install`                       |
| `-v`         | verbose colcon output                          |
| `-h`         | help                                           |

### launch.sh

Components — pick any combination:

| Flag       | Pane                               |
|------------|------------------------------------|
| `-gz`      | Gazebo                             |
| `-sitl`    | ArduPilot SITL (`sim_vehicle.py`)  |
| `-bridge`  | `ros_gz_bridge` clock bridge       |
| `-ros`     | ROS2 nodes (`drone.launch.py`)     |

Modifiers:

| Flag           | Effect                                              |
|----------------|-----------------------------------------------------|
| `-r`           | Gazebo autorun (`-r`)                               |
| `-d`           | detached tmux (do not attach)                       |
| `-s NAME`      | tmux session name (default `sim`)                   |
| `-w NAME`      | world name (without `.sdf`)                         |
| `--ask-world`  | interactive list of `worlds/`                       |
| `-p FILE`      | specific `.parm` file                               |
| `--ask-params` | interactive list of `params/`                       |
| `-log`         | tee each pane to `$LOG_DIR/<session>-<pane>.log`    |
| `--headless`   | Gazebo without GUI (`-s`)                           |
| `--manual`     | extra pane: MAVProxy on udp:127.0.0.1:14550         |
| `--monitor`    | extra pane: `watch ros2 topic list`                 |
| `--auto NAME`  | extra pane: `ros2 run drone_sim NAME`               |

Presets:

| Flag       | Expands to                              |
|------------|-----------------------------------------|
| `--layout` | `-gz`                                   |
| `--sim`    | `-gz -r -sitl -bridge`                  |
| `--full`   | `-gz -r -sitl -bridge -ros`             |

## Examples

```bash
# 1. Full workspace rebuild from scratch
./build.sh --clean

# 2. Rebuild a single package
./build.sh -p drone_sim

# 3. Just Gazebo with the default world
./launch.sh -gz -r

# 4. Full stack (Gazebo + SITL + bridge + ROS2), attach to tmux
./launch.sh --full

# 5. Headless full stack with logging, detached
./launch.sh --full --headless -log -d -s ci

# 6. Sim + manual MAVProxy pane, custom world picked interactively
./launch.sh --sim --manual --ask-world
```

## Notes

- The scripts read everything from `.env_simulation` — no hardcoded paths.
- Re-running `launch.sh` with the same `-s` name kills the previous session.
- `LOG_DIR` is auto-created if missing.
