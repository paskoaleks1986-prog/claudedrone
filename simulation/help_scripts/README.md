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
/home/aleks/git/proj/aerosearch/claudedrone-git/simulation/help_scripts/.env_simulation.example  # committed
```

Copy and edit:

```bash
cp claudedrone-git/simulation/help_scripts/.env_simulation.example .env_simulation
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

### capture.sh

Mode (pick one):

| Flag             | Effect                                                      |
|------------------|-------------------------------------------------------------|
| `--video`        | record a target window via `ffmpeg` + `x11grab` + `xdotool` |
| `--screenshots`  | loop over tmux panes, screenshot whole screen each tick     |

Common:

| Flag       | Effect                                                                |
|------------|-----------------------------------------------------------------------|
| `-o DIR`   | output directory (default: `$MEDIA_DIR` from env)                     |

`--video` only:

| Flag         | Effect                                          |
|--------------|-------------------------------------------------|
| `-t SECONDS` | recording length (default 15)                   |
| `-w NAME`    | window name to focus (default `Gazebo`)         |

`--screenshots` only:

| Flag         | Effect                                                  |
|--------------|---------------------------------------------------------|
| `-i SECONDS` | interval between shots (default 3)                      |
| `-n COUNT`   | total shots (default 0 = run until Ctrl+C)              |
| `-s NAME`    | tmux session to cycle through (default `sim`)           |

`capture.sh` is a separate script (not a `launch.sh` flag) because the
two have different lifecycles: launch brings the stack up once and
exits; capture is a long-running observer that may be started, stopped
and restarted independently while the stack runs.

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

# 7. Record 15s of the Gazebo window
./capture.sh --video

# 8. 10 screenshots, 3s apart, while cycling through tmux panes
./capture.sh --screenshots -i 3 -n 10
```

## Notes

- The scripts read everything from `.env_simulation` — no hardcoded paths.
- Re-running `launch.sh` with the same `-s` name kills the previous session.
- `LOG_DIR` and `MEDIA_DIR` are auto-created if missing.
- `capture.sh --video` requires `xdotool` and `ffmpeg`; `--screenshots`
  uses `scrot` if present, otherwise falls back to ImageMagick `import`.
