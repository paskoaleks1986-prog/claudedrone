# claudedrone — Current Status

> **For agents and collaborators:** this file is the source of truth for "where is this project right now". Read this first before any other planning file.
> **Updated:** every Sunday during weekly review.

---

**Last updated:** 2026-05-04
**Updated by:** Aleks
**Owner:** Aleksandr Pasko ([@paskoaleks1986-prog](https://github.com/paskoaleks1986-prog))

## Project at a glance

claudedrone is an autonomous indoor mapping drone for GPS-denied environments using a stop-scan multi-layer approach with ToF sensors and distributed compute.

The high-level public roadmap is in [`roadmap.md`](roadmap.md). The current week's tasks are in [`current-sprint.md`](current-sprint.md).

## Current phase

**Phase 1 — Simulation MVP**

## What's working now

- Gazebo Harmonic + ArduPilot SITL + MAVROS + ROS2 Jazzy pipeline runs end-to-end
- Custom `iris_claudedrone` model with TF-Luna (downward + sweep), 6× VL53L0X, optical flow camera
- Custom static `claudedrone` model (sensor positions reference)
- `indoor_room.sdf` world — 5×4m room with walls, ceiling, lighting, central column
- Basic autonomous takeoff in GUIDED mode
- Indoor parameter file (`indoor.parm`) — GPS disabled, optical flow as position source, slow indoor speeds
- ROS2 Jazzy package `drone_sim` builds and runs
- `sensor_monitor` node — subscribes to all 6 VL53L0X channels and TF-Luna, publishes priority alerts
- `drone.launch.py` — single-command launch of Gazebo + ROS2 nodes + ros_gz_bridge

## What's NOT working yet

- Stop-scan methodology (no FSM, no servo sweep logic)
- Multi-layer mapping (no altitude-based scan storage)
- Cartographer SLAM integration
- Hardware: nothing assembled physically yet
- ESP32 firmware (only planned, no code)
- Pi Zero 2W companion setup
- Distributed compute architecture (only conceptual)

## Active branches

- `main` — last polished state, public-facing
- `feat/ardupilot-sitl` — extended sensors integration, 10 commits ahead of main, ready to merge after review

## Hardware status

| Component | Status |
|---|---|
| Source Two 6" frame | Acquired, not assembled |
| MT2204 motors | Acquired |
| 4-in-1 ESC (model TBC) | Acquired |
| F4/F7 flight controller | Acquired |
| Raspberry Pi Zero 2W | Acquired |
| ESP32 DevKit | Acquired |
| TF-Luna ×2 | Acquired |
| VL53L0X ×6 | Acquired |
| TCA9548A I²C mux | Acquired |
| PMW3901 optical flow | Acquired |
| SG90 servo | Acquired |
| PCA9685 PWM driver | Acquired |
| PDB | Acquired |

## Open questions / unresolved items

- 4-in-1 ESC exact model (currently noted as "AirSelfie 45A" — needs confirmation from datasheet or board markings)

## Public presence

- GitHub: TBD (mirror to be set up Week 1) — will be at github.com/paskoaleks1986-prog/claudedrone
- GitLab: gitlab.com/alekspasko/claudedrone (current main)
- LinkedIn: [aleks-pasko](https://www.linkedin.com/in/aleks-pasko-9aa56b372/)
- YouTube: @ClaudeDrone-f5b6q (no videos yet)
- Dev.to: not started
- X/Twitter: not started

## Long-term direction

Building toward a working hardware prototype with reliable multi-layer mapping, suitable for real industrial pilots in indoor GPS-denied environments.
