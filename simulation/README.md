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

---

## Жизненный цикл стека / Stack lifecycle (запуск и teardown)

⚠ **Стек (Gazebo + ArduPilot SITL + MAVROS + ROS-ноды) запускается и тушится ОТДЕЛЬНО от
клиентов. Авто-teardown НЕТ — teardown ЯВНЫЙ.**

**Запуск:** `help_scripts/launch.sh ... -d` → стек поднимается как **detached tmux-сессия**
(`-s <session>`), живёт независимо от любого Python-клиента/смоука. После выхода launch.sh
стек **продолжает работать** — это by design (инфраструктура ≠ клиент).

**Teardown — ТОЛЬКО вручную:** `help_scripts/kill_sim_stack.sh [GRACE_S]` — валит
gz/arducopter/mavros/ros чисто (SIGINT→SIGKILL). **Обязательно вызывать по завершении
работы / перед уходом / перед передачей стека другому агенту.** `kill -9` на launch-parent
оставляет ROS2-ноды orphan'ами — НЕ так; используй `kill_sim_stack.sh`.

**Клиенты НЕ тушат стек:** `comm.close()` (smoke/policy_bridge) закрывает только ROS-узел
клиента — он подключается к живому стеку и отключается, **стек при этом не трогается**.

**Почему стек держат живым между прогонами (намеренно, не баг):** многократный
kill+relaunch Gazebo деградирует NVIDIA EGL после нескольких циклов (gz-pane умирает, AP
уходит во внутреннюю симуляцию) → при итерационной отладке смоуков один gz держат живым и
переподключают Python-клиент (soft-reset в воздухе, без relaunch). Это экономит ~40с boot и
бережёт EGL. **Но по завершении сессии/итераций — обязательный `kill_sim_stack.sh`.**

> Stack (Gazebo + ArduPilot SITL + MAVROS + ROS nodes) is launched and torn down
> **independently of clients. There is NO auto-teardown — teardown is explicit.**
> Launch via `launch.sh -d` (detached tmux, survives launch.sh exit). Tear down ONLY via
> `kill_sim_stack.sh`. Python clients' `comm.close()` closes the client node only, never the
> stack. The stack is deliberately kept alive between smoke iterations to avoid NVIDIA EGL
> degradation from repeated gz kill/relaunch — but ALWAYS run `kill_sim_stack.sh` when done.
