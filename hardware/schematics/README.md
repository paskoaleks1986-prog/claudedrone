# hardware/schematics/

> Fritzing wiring diagrams and exported images for ClaudeDrone electronics.
> Fritzing-схемы подключения и экспортированные изображения для электроники ClaudeDrone.

Схемы хранятся здесь (а не в `docs/`), потому что это рабочие конструкторские файлы железа — их изменяют вместе с изменениями в схемотехнике, а не с изменениями в документации.

Schematics live here (not in `docs/`) because they are active hardware design files — they change alongside circuit revisions, not alongside documentation updates.

---

## Что сюда класть / What goes here

**Исходники Fritzing / Fritzing source files:**
- `power_distribution.fzz` — схема питания: LiPo → PDB → ESC, BEC 5V/12V
- `esp32_sensors.fzz` — подключение ESP32 к TCA9548A, VL53L0X, TF-Luna, PCA9685
- `rpi_bridge.fzz` — схема RPi Zero 2W: UART, I2C, WiFi
- `full_system.fzz` — полная схема всей электроники дрона

**Экспортированные изображения / Exported images:**
- `*.png` — растровый экспорт для просмотра в браузере / raster export for GitHub preview
- `*.svg` — векторный экспорт для печати и документации / vector export for print and docs

Формат исходников: `.fzz` (Fritzing). Экспорт: File → Export → as Image (PNG/SVG).
