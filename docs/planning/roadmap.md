# Roadmap

High-level development phases for claudedrone. Status updates in [current-status.md](current-status.md).

---

## Phase 1 — Simulation MVP

Build a complete simulation pipeline where the drone can autonomously map an indoor space using the stop-scan multi-layer approach.

**Components:**
- Stop-scan finite state machine (HOVER → STOP → SCAN → MOVE)
- Multi-altitude scanning with 2D occupancy grids per layer
- Cartographer 2D SLAM integration
- End-to-end demo: drone explores `indoor_room.sdf` autonomously and produces a 3-layer map

## Phase 2 — Hardware Operational

Get the physical drone flying in GPS-denied mode using optical flow and rangefinder for position.

**Components:**
- Frame, motors, ESC, and FC assembly
- ESP32 firmware for sensor polling
- Pi Zero 2W companion computer setup
- ArduPilot indoor parameter tuning
- First indoor flight in STABILIZE and GUIDED modes

## Phase 3 — Stop-Scan on Real Hardware

Port the simulation methodology to the physical drone and validate against ground truth.

**Components:**
- ESP32 sensor data flowing into ROS2 via Pi Zero 2W
- Stop-scan FSM running on real hardware
- Multi-layer mapping in real rooms
- Performance metrics: scan time, map accuracy

## Phase 4 — Distributed Compute

Move heavy SLAM processing to a remote workstation, keeping onboard compute lightweight.

**Components:**
- WiFi link drone ↔ remote server
- RTAB-Map running server-side for 3D mapping
- Latency measurement and degradation modes
- Web dashboard for real-time visualization

## Phase 5 — Customer-Ready Demo

Reliable, repeatable system suitable for real-world pilots.

**Components:**
- Reliable multi-run operation without manual intervention
- Onboarding documentation for non-developer users
- Performance comparison with alternative solutions
- First field pilot with a real industrial site
