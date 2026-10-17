# software/h1 — код спринта krot H.1 «Провод по воздуху»

PC-сторона железного спринта H.1: поднять MAVLink-канал FC (MicoAir405Mini) ↔ ESP32 (DroneBridge) ↔ ноутбук.

**Здесь — ТОЛЬКО код + окружение** (grant Aleks 2026-07-30, этот репо публичный showcase).
Логи, dev-log, знания, решения, спринт-лог, замеры, фото — **у firmware-агента**, не здесь:
- dev-log: `claudedrone-firmware:docs/dev-log/13-krot-h1-*`
- знания: `claudedrone-firmware:docs/knowledge/`
- сборка/замеры/чеклист: `claudedrone-firmware:docs/bring-up/h1/`
- спринт-доки + лог: obsidian `planning/live/krot-h1/` (FW-sprint-H1-obshiy, FW-sprint-H1.1-detalno, FW-review-krot-H1, FW-sprint-H1-LOG)
- решения: obsidian `decisions/ADR-011-*`, `ADR-012-*`
- исходники Aleks (чат): obsidian `planning/aleks_docs/krot/firmwaer_sprint_krot/`
- session log / timetracking: `_workspace/`

## Layout
```
checks/     — идентификация контроллеров: esptool chip_id, lsusb/dmesg парсеры
inventory/  — pymavlink-сканеры: каталог телеметрии (H.1.5) + каталог команд (H.1.6)
routing/    — mavlink-router конфиг (H.1.4): 1 вход → QGC + MAVROS + логгер
latency/    — замеры RTT/джиттера/потерь (H.1.7)
```

## Окружение (ставит Aleks сам)
```bash
cd software/h1
python3 -m venv .venv && source .venv/bin/activate
pip install -U pip esptool pymavlink pyserial
```
`.venv/`, `__pycache__/` — уже в `.gitignore` репо. `mavproxy` стоит глобально; `mavlink-router` — отдельно позже.

## SITL-first
Весь код разрабатывается против SITL simulation (эталон задержки ≈0 мс), переключение на железо — переменной эндпоинта. См. `FW-sprint-H1-obshiy`.
