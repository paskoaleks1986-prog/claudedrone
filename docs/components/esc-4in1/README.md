# 4-in-1 ESC — AirSelfie 45A

Четырёхканальный регулятор оборотов (ESC) с интегрированным дизайном 4-в-1.  
Управляет четырьмя бесколлекторными моторами MT2204 2300KV.

## Specifications

| Parameter            | Value                        |
|----------------------|------------------------------|
| Continuous current   | 45A per channel              |
| MCU                  | 8-bit high-speed             |
| PWM support          | High frequency               |
| Weight               | 12.5 g                       |
| Braking              | Hardware synchronous rectification + regenerative braking |
| Target platform      | Small UAVs, racing drones    |

## Key Features

- 4-in-1 design — один блок вместо четырёх отдельных ESC, упрощает монтаж на раме
- Аппаратное синхронное выпрямление и рекуперативное торможение — повышает КПД и возвращает энергию в батарею
- Отличная линейность газа и чувствительное торможение — улучшает манёвренность дрона
- Поддержка высокочастотного PWM

## Usage in ClaudeDrone

Подключён к плате распределения питания (PDB) и управляет четырьмя моторами MT2204 2300KV.  
Сигналы управления приходят от полётного контроллера (F4/F7 FC).

## Files

| File                        | Description               |
|-----------------------------|---------------------------|
| `4IN1-ESC-table-1.jpg`      | Datasheet — таблица 1     |
| `4IN1-ESC-table-2.jpg`      | Datasheet — таблица 2     |
| `4IN1-ESC-Description.txt`  | Исходное описание         |
| `../../media/photos/components/4IN1-ESC-1.jpg` | Фото компонента (вид 1) |
| `../../media/photos/components/4IN1-ESC-2.jpg` | Фото компонента (вид 2) |
