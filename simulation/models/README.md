# simulation/models/

> URDF/SDF drone model and any custom Gazebo models used in simulation worlds.
> URDF/SDF модель дрона и кастомные модели объектов, используемые в симуляционных мирах.

Модели хранятся отдельно от `.world`-файлов, потому что одна модель дрона может использоваться в разных мирах, а кастомные объекты (ящики, стены, препятствия) могут переиспользоваться между мирами.

Models are separate from `.world` files because a single drone model can be used across many worlds, and custom objects (boxes, walls, obstacles) can be reused between environments.

---

## Что сюда класть / What goes here

- `claudedrone/`
  - `model.sdf` или `model.urdf` — полная модель дрона с сенсорами и плагинами Gazebo
  - `model.config` — мета-описание модели для Gazebo
  - `meshes/` — 3D меши (.dae, .stl) кузова и пропеллеров
- `box_obstacle/` — переиспользуемая модель ящика-препятствия
- `wall_segment/` — сегмент стены для сборки помещений
- `tfluna_sensor/` — SDF-модель TF-Luna лидара (с плагином дальномера)

Формат моделей: Gazebo SDF v1.7+ или URDF с xacro-макросами.
