# TF-Luna — LiDAR Ranging Module

Одноточечный лидарный дальномер на основе VCSEL (850 nm).  
Используется в ClaudeDrone в двух экземплярах: горизонтальное сканирование и удержание высоты.

## Specifications

| Parameter             | Value                   |
|-----------------------|-------------------------|
| Range                 | 0.2 m – 8 m             |
| Frame rate            | 1 – 250 Hz (adjustable) |
| Supply voltage        | 5V ± 0.1V               |
| Average current       | ≤ 70 mA                 |
| Peak current          | 150 mA                  |
| Average power         | 350 mW                  |
| Communication         | UART / I²C              |
| Logic level           | LVTTL (3.3V)            |
| Default baud rate     | 115200                  |
| Serial config         | 8N1                     |
| Light source          | VCSEL, 850 nm           |
| Operating temperature | −20°C – +75°C           |
| Certifications        | CE, FCC, RoHS           |

## Usage in ClaudeDrone

| Instance     | Mounting          | Purpose                                     |
|--------------|-------------------|---------------------------------------------|
| TF-Luna #1   | SG90 servo (горизонталь) | 2D горизонтальное сканирование 180°   |
| TF-Luna #2   | Вниз (фиксировано) | Удержание высоты (altitude hold PID)       |

Подключение: UART к ESP32. Логика 3.3V совместима с ESP32 напрямую.

## Package Contents

- 1× TF-Luna LiDAR Module
- 1× Male cable
- 1× Female cable
- 1× Cable 1.25mm 6P

## Files

| File                       | Description              |
|----------------------------|--------------------------|
| `TF-Luna-Specification.jpg` | Таблица характеристик из даташита |
| `TF-Luna-Description.txt`  | Исходное описание        |
| `../../media/photos/components/TF-Luna.jpg` | Фото модуля |
