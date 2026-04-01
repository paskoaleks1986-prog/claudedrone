# software/ros/

> ROS 2 packages for ClaudeDrone: nodes, launch files, message/service definitions, and parameter configs.
> ROS 2 пакеты ClaudeDrone: ноды, launch-файлы, определения сообщений/сервисов, конфиги параметров.

Всё ROS-специфичное хранится здесь, а не в `software/compute/`, потому что ROS — это middleware-слой: он определяет топики, типы сообщений и граф нод, но не сами алгоритмы.

Everything ROS-specific lives here rather than in `software/compute/` because ROS is the middleware layer — it defines topics, message types, and node graph, not the algorithms themselves.

---

## Что сюда класть / What goes here

- `claudedrone_bringup/` — мета-пакет для запуска всей системы / meta-package to launch the full system
  - `launch/full_system.launch.py`
  - `config/params.yaml`
- `claudedrone_msgs/` — кастомные msg/srv определения / custom message and service definitions
  - `msg/SensorFrame.msg`
  - `msg/DroneCommand.msg`
  - `srv/SetMission.srv`
- `slam_node/` — нода SLAM, оборачивающая алгоритм из `software/compute/`
- `sensor_bridge_node/` — нода приёма потока с дрона и публикации в топики ROS
- `path_planner_node/` — нода планирования пути

**Не класть сюда / Do NOT put here:** чистый алгоритмический код без ROS-зависимостей (→ `software/compute/`), firmware ESP32 (→ `firmware/esp32/`)
