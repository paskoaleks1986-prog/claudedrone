# ClaudeDrone 🚁

> Open-source autonomous drone for GPS-free indoor navigation and 2D/3D mapping.  
> Открытый проект автономного дрона — навигация без GPS, построение карт замкнутых пространств.

![Status](https://img.shields.io/badge/status-in%20development-yellow)
![License](https://img.shields.io/badge/license-MIT-green)
![Platform](https://img.shields.io/badge/platform-RaspberryPi%20%2B%20ESP32-blue)

---

### What is ClaudeDrone?

ClaudeDrone is an open-source robotics project focused on building a drone capable of autonomous navigation **without GPS**.

The drone is designed to operate in indoor or GPS-denied environments — perceiving its surroundings through distance sensors, building maps of the environment geometry, and navigating based on that map.

All software, hardware documentation, and research notes are published openly.

### Core Architecture Concept

ClaudeDrone follows a **distributed compute architecture**:

> The drone itself is not responsible for heavy computation.  
> It acts as a **sensor and actuator platform**.  
> All navigation, mapping, and decision-making runs on an **external compute unit** connected via WiFi.

```
┌─────────────────────┐       WiFi        ┌──────────────────────────┐
│    Drone Platform   │ ◄───────────────► │  External Compute Unit   │
│                     │                   │                          │
│  • Flight control   │  sensor stream    │  • Sensor fusion         │
│  • Sensor reading   │ ────────────────► │  • SLAM / Mapping        │
│  • Motor execution  │                   │  • Path planning         │
│                     │ ◄──────────────── │  • Mission logic         │
└─────────────────────┘  movement commands└──────────────────────────┘
```

This approach allows:
- lightweight drone hardware
- flexible compute upgrades without touching the drone
- easier algorithm experimentation
- safer development and debugging

### Key Principles

- **No GPS** — navigation based entirely on onboard sensors
- **No pilot** — autonomous decision-making and path execution
- **Open source** — all code and hardware designs are public
- **Modular** — every component is independently testable and replaceable
- **Research-driven** — practical exploration of indoor autonomous robotics

---

### Что такое ClaudeDrone?

ClaudeDrone — открытый исследовательский проект автономного дрона для навигации в замкнутых пространствах **без GPS и без пилота**.

Дрон воспринимает окружение через ToF-датчики и лидары, строит карты геометрии пространства и навигирует по ним. Весь код, схемы и документация — в открытом доступе.

### Ключевая архитектурная идея

Дрон — это **платформа сенсоров и актуаторов**. Все вычисления (SLAM, планирование пути, принятие решений) выполняются на **внешнем вычислительном узле** по WiFi. Это делает дрон лёгким, а алгоритмы — легко заменяемыми без изменения железа.

---

## 🔧 Hardware / Железо

| Component | Details |
|---|---|
| **Frame** | Source Two 6" Rack (carbon fiber) |
| **Motors** | MT2204 2300KV × 4 |
| **Props** | 6" tri-blade |
| **ESC** | 4-in-1 ESC stack |
| **Flight Controller** | F4/F7 FC |
| **Onboard Computer** | Raspberry Pi Zero 2W (WiFi bridge + sensor hub) |
| **Microcontroller** | ESP32 DevKit (real-time: servo, sensors, FC bridge) |
| **PWM Driver** | PCA9685 16-ch |
| **Power** | LiPo 2200mAh XT60 + PDB-XT60 BEC 5V/12V |
| **LiDAR (scanning)** | TF-Luna × 1 on SG90 servo (2D horizontal sweep) |
| **LiDAR (altimeter)** | TF-Luna × 1 downward (altitude hold) |
| **ToF sensors** | VL53L0X × 6 (360° obstacle detection) |
| **Optical Flow** | PMW3901 (X/Y velocity estimation) |
| **I2C Multiplexer** | TCA9548A 8-channel |

---

## 🏗️ System Architecture / Архитектура

### Onboard (Drone)
```
ESP32
  ├── SG90 servo + TF-Luna  ──►  2D scan (180° sweep)
  ├── TF-Luna downward       ──►  altitude hold PID
  ├── TCA9548A + VL53 ×6    ──►  360° obstacle detection
  ├── Optical Flow           ──►  X/Y velocity
  └── Flight Controller (F4/F7) via SBUS/MSP

RPi Zero 2W
  ├── Aggregates sensor data from ESP32
  ├── WiFi bridge to external compute unit
  └── Streams telemetry / receives movement commands
```

### External Compute
```
PC / Laptop / Server
  ├── Receives full sensor stream from drone
  ├── SLAM + occupancy map building
  ├── Path planning and mission logic
  └── Sends movement commands back to drone
```

---

## 📁 Repository Structure

```
claudedrone/
├── README.md
├── firmware/
│   ├── esp32/           # ESP32 real-time firmware (PlatformIO)
│   └── fc/              # Flight controller config
├── software/
│   ├── onboard/         # RPi: WiFi bridge, sensor aggregation
│   ├── compute/         # External: SLAM, mapping, path planning
│   └── sensors/         # Sensor drivers and calibration
├── hardware/
│   ├── schematics/      # Wiring diagrams
│   └── mounts/          # 3D printable mounts (STL)
├── docs/
│   ├── architecture.md
│   ├── sensors.md
│   └── roadmap.md
└── media/
    ├── logo/
    └── photos/
```

## 📹 Development Log

## 🤝 Contributing

Open to collaboration in sensor integration, mapping algorithms, navigation systems, and robotics research.

---

## 📬 Contact

- YouTube: [@ClaudeDrone](https://www.youtube.com/@ClaudeDrone-f5b6q)
- LinkedIn: [ClaudeDrone](https://www.linkedin.com/in/aleks-pasko-9aa56b372/)
- GitHub: [github.com/claudedrone](https://github.com/claudedrone)

---
Progress is documented publicly:

*No GPS. No pilot. Built in public.*