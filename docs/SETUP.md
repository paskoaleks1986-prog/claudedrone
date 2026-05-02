# Setup Guide

This guide walks through the complete setup needed to run the claudedrone simulation locally — from a clean Ubuntu 24.04 install to a flying drone in Gazebo.

**Total time:** ~60-90 minutes for a clean setup, depending on download speeds.

For raw notes from the original integration journey, see [`dev-log/01-ardupilot-sitl-setup.md`](dev-log/01-ardupilot-sitl-setup.md) and [`dev-log/02-ros2-gazebo-integration.md`](dev-log/02-ros2-gazebo-integration.md).

---

## Prerequisites

- **OS:** Ubuntu 24.04 LTS (Noble Numbat)
- **GPU (recommended):** NVIDIA with proprietary drivers, or any modern GPU with OpenGL 3.3+
- **Disk space:** ~15 GB free (ROS2 + Gazebo + ArduPilot)
- **RAM:** 8 GB minimum, 16 GB recommended

---

## Step 1 — Install ROS2 Jazzy

Follow the [official ROS2 Jazzy install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). The short version:

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
```

Add ROS2 to your shell:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
printenv ROS_DISTRO
# Expected: jazzy
```

## Step 2 — Install Gazebo Harmonic and ros_gz_bridge

```bash
sudo apt install -y ros-jazzy-ros-gz
```

This installs Gazebo Harmonic, `ros_gz_bridge`, `ros_gz_sim`, and `ros_gz_interfaces` together.

Verify:

```bash
gz sim --version
ros2 pkg list | grep gz
```

## Step 3 — Install ArduPilot SITL

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Restart shell to pick up PATH changes
source ~/.bashrc
```

Add SITL tools to PATH (if not already):

```bash
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
source ~/.bashrc
which sim_vehicle.py
```

First build:

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
# This is a long initial build — go grab coffee
```

## Step 4 — Install ardupilot_gazebo plugin

```bash
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Add plugin paths:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

## Step 5 — Clone and build claudedrone

```bash
cd ~
git clone https://github.com/alekspasko/claudedrone.git
cd claudedrone/simulation

# Make sure ROS2 is sourced
source /opt/ros/jazzy/setup.bash

# Build
colcon build --symlink-install

# Source the local workspace
source install/setup.bash
```

> **Note about Python virtualenvs:** if you use a `venv` inside `simulation/`, mark it as ignored by colcon:
> ```bash
> touch simulation/venv/COLCON_IGNORE
> ```
> Otherwise `colcon` will try to build the venv as a package and fail.

Add the project models path (needed so Gazebo finds `iris_claudedrone`):

```bash
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/claudedrone/simulation/src/drone_sim/models:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

## Step 6 — Run the simulation

You'll need three terminals.

**Terminal 1 — Gazebo with the indoor world:**

```bash
cd ~/claudedrone/simulation
source /opt/ros/jazzy/setup.bash
source install/setup.bash

gz sim src/drone_sim/worlds/indoor_room.sdf -r
```

The `-r` flag auto-starts the simulation (skips the manual play button).

**Terminal 2 — ArduPilot SITL connected to Gazebo:**

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
  --add-param-file=$HOME/claudedrone/simulation/config/ardupilot/indoor.parm
```

The `--add-param-file` flag loads our indoor parameter overrides (GPS disabled, optical flow as position source, slow speeds).

**Terminal 3 — ROS2 launch (sensor monitor + ros_gz_bridge):**

```bash
cd ~/claudedrone/simulation
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch drone_sim drone.launch.py
```

## Step 7 — First flight

In the SITL console (Terminal 2), arm and take off:

```
mode GUIDED
arm throttle
takeoff 2
```

The drone should arm, lift off, and hover at 2 meters in the simulated indoor room.

To land:

```
mode LAND
```

To return to launch position:

```
mode RTL
```

---

## Troubleshooting

### Gazebo shows a black screen on NVIDIA

Force the rendering pipeline:

```bash
sudo nvidia-settings --assign CurrentMetaMode="nvidia-auto-select +0+0 { ForceFullCompositionPipeline = On }"
```

### `colcon build` fails because of venv

Make sure `simulation/venv/` (if it exists) has a `COLCON_IGNORE` file:

```bash
touch ~/claudedrone/simulation/venv/COLCON_IGNORE
```

### `sim_vehicle.py: command not found`

PATH isn't picking up. Verify:

```bash
echo $PATH | tr ':' '\n' | grep ardupilot
```

If the line is empty, re-add to `~/.bashrc`:

```bash
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
source ~/.bashrc
```

### `ros2 topic list` shows no topics from Gazebo

Verify `ros_gz_bridge` is running (Terminal 3 should show it). Test the underlying Gazebo topic:

```bash
gz topic -l
gz topic -e -t /world/indoor_room/pose/info
```

If Gazebo has the topic but ROS2 doesn't, the bridge isn't translating it — check the `arguments` list in `drone.launch.py`.

### Drone won't arm because of pre-arm checks

The `indoor.parm` file disables most pre-arm checks (`ARMING_SKIPCHK 1`, `GPS1_TYPE 0`, etc.) for indoor testing. If you still get refusals, try forcing arm:

```
arm throttle force
```

Don't do this on a real drone — only in simulation.

---

## What's next

Once you have the simulation running:
- Read [`docs/architecture.md`](architecture.md) for the system design
- See [`planning/current-status.md`](planning/current-status.md) for what's actively being worked on
- See [`planning/roadmap-12m.md`](planning/roadmap-12m.md) for the long-term plan
