# simulation/

> Gazebo simulation environment for ClaudeDrone: drone model, indoor worlds, and launch configurations.
> Симуляция Gazebo для ClaudeDrone: модель дрона, файлы помещений и конфигурации запуска.

Симуляция вынесена в отдельную папку (а не в `software/`), потому что это независимая среда — она не запускается на реальном дроне, а заменяет его для разработки и тестирования алгоритмов без железа.

Simulation is a separate folder (not inside `software/`) because it's a standalone environment — it doesn't run on the real drone, it replaces the drone for algorithm development and testing without hardware.

---

## Структура / Structure

```
simulation/
├── models/      # URDF/SDF модель дрона / drone URDF/SDF model
├── worlds/      # .world файлы помещений / indoor .world files
└── launch/      # launch-файлы для запуска симуляции / Gazebo launch files
```

## Что сюда класть / What goes here

- Модели дрона в формате URDF или SDF
- Файлы виртуальных помещений (комнаты, коридоры, склады)
- Launch-файлы для запуска Gazebo с нужной моделью и миром
- Конфиги плагинов Gazebo (IMU, лидар, камера)

**Не класть сюда / Do NOT put here:** исходный код алгоритмов (→ `software/`), ROS-пакеты (→ `software/ros/`)
