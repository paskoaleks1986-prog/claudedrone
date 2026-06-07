# Протокол смены мира (карты) — чек-лист

> Заведён 2026-06-07 по директиве Aleks после первого переключения
> rl_room_empty_6x6 → indoor_room в видео-сессии. Миров будет много —
> каждый запуск нового мира идёт по этому списку. Дополнять по мере
> находок (формат: пункт + симптом, если пропустить).

## Перед запуском

1. **SDF существует и валиден:** `src/drone_sim/worlds/<world>.sdf`,
   `gz sdf --check`. Симптом пропуска: gz молча падает, пустой мир.
2. **free_mask для coverage (Option A):**
   `worlds/rl_rooms/<world>/free_mask.png` + `metadata.json` (генерит
   rl-lab из .npy). Симптом: WARN `free_mask_path=auto: не нашёл` →
   coverage = visited/4096 (заниженный, НЕ сравним с другими ранами).
   ⚠ Для не-RL миров (indoor_room) маски может не быть осознанно —
   тогда coverage-метрики этого рана НЕ сравнивать с Option-A ранами.
3. **Размер комнаты vs room_size bridge:** PARAM_DEFAULTS.room_size = 6.4
   (грид 64×0.1). Мир больше → дрон выйдет за грид (visited update = None,
   клетки не метятся). Симптом: coverage стоит при движении дрона.
4. **FENCE-параметры vs геометрия мира:** indoor.parm FENCE_RADIUS 3.5 —
   для комнат > 7×7 м пересмотреть (FENCE сейчас disabled, но при
   включении на железе — обязательный пункт).
5. **Spawn-точка:** (0,0) = центр мира — в новом мире там не должно быть
   мебели/стен. Симптом: моментальный SAFETY TRIGGER на старте,
   takeoff в объект.

## Запуск

6. **launch:** `./help_scripts/launch.sh ... -w <world>` (без .sdf).
7. **policy_bridge:** `DEFAULT_WORLD=<world>` в env запуска — иначе
   free_mask ищется от другого мира.
8. **GUI-старт медленнее headless:** MAVROS-коннект до 2-3 мин;
   time-jump ERROR'ы в mavros-логе при GUI — ШУМ, не блокер
   (проверять `connected: true` через /mavros/state, не по логу).
   Watcher-скриптам не верить хартбит-грепу — верить topic echo.

## После запуска (smoke нового мира)

9. `gz sim` жив + sim clock идёт (RTF ≈ 1): два чтения /scan/sweep.
10. Сенсоры видят геометрию: /drone/perimeter не все 2.0 (если дрон
    не в центре пустого зала).
11. **Поведение модели — это OOD-вопрос, не баг мира:** SWEEP-02
    обучена на rl_room 6×6; в чужих мирах (мультирум) ожидаемы
    залипания/осцилляции. Фиксировать треком (matplotlib), не чинить
    параметрами исполнительного слоя.

## Каждый полёт в новом мире (директива Aleks 2026-06-07)

- odom-трек → CSV → matplotlib 2D PNG → `$DRONE_MEDIA_ROOT/sim/tracks/`
- видео → `capture.sh --video -w "Gazebo Sim"` (НЕ "Gazebo GUI" — это
  1×1 Qt-прокси; скрипту нужен DISPLAY=:0 + XAUTHORITY)
- описание поведения + ссылки на артефакты → dev-log
