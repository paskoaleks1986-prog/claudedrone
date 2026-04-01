# simulation/launch/

> Launch files for starting Gazebo simulation with the ClaudeDrone model and chosen world.
> Launch-файлы для запуска симуляции Gazebo с моделью дрона и выбранным миром.

Launch-файлы симуляции хранятся здесь, а не в `software/ros/`, потому что они запускают Gazebo (внешний симулятор), а не реальную систему — это принципиально другой контекст запуска.

Simulation launch files live here rather than in `software/ros/` because they start Gazebo (an external simulator), not the real system — a fundamentally different runtime context.

---

## Что сюда класть / What goes here

- `sim_room_simple.launch.py` — запуск дрона в пустой комнате / spawn drone in simple room
- `sim_with_obstacles.launch.py` — запуск с препятствиями / launch with obstacles world
- `sim_slam_test.launch.py` — запуск симуляции + SLAM ноды / simulation + SLAM node together
- `spawn_drone.launch.py` — переиспользуемый launch для спавна модели дрона в произвольном мире
- `gazebo_params.yaml` — параметры Gazebo (physics step, real-time factor, и т.д.)

Для запуска: `ros2 launch simulation/launch/sim_room_simple.launch.py`
