# simulation/worlds/

> Gazebo .world files describing indoor environments for ClaudeDrone simulation.
> Файлы `.world` для Gazebo — описания виртуальных помещений, в которых симулируется полёт дрона.

Файлы помещений хранятся отдельно от модели дрона, потому что одна и та же модель дрона должна работать в разных мирах — это упрощает тестирование разных сценариев навигации.

World files are separate from the drone model because the same drone model should fly in different worlds — this makes it easy to test different navigation scenarios independently.

---

## Что сюда класть / What goes here

- `room_simple.world` — пустая прямоугольная комната 5×5м / empty rectangular room 5×5m
- `room_with_obstacles.world` — комната с ящиками и колоннами / room with boxes and columns
- `corridor.world` — узкий коридор для теста навигации / narrow corridor for navigation testing
- `warehouse.world` — открытый склад с полками / open warehouse with shelves
- `outdoor_gps_denied.world` — открытое пространство без GPS-маяков / open space, GPS denied

Формат: Gazebo `.world` (XML/SDF). Рекомендуется хранить кастомные меши моделей объектов в `simulation/models/`.
