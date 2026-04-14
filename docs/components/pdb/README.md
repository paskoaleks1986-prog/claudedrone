# PDB — Power Distribution Board with Dual BEC

Плата распределения питания с двумя BEC-регуляторами (5V и 12V).  
Центральный узел питания дрона: батарея → ESC, бортовая электроника, камера/VTX.

## Specifications

| Parameter              | Value                                      |
|------------------------|--------------------------------------------|
| Dimensions             | 36 × 50 × 4 mm (без XT60)                 |
| Mounting pattern       | 30.5 × 30.5 mm, отверстия 3 mm            |
| Weight                 | 7.5 g (без XT60) / 11 g (с XT60)          |
| Input voltage          | 2S – 6S LiPo                              |
| PCB                    | 4-слойная, медь 2oz                        |
| ESC pads               | 6 пар (совместимость с X и H компоновками) |
| Continuous current     | 4 × 25 A                                  |
| Peak current (10s)     | 4 × 30 A                                  |

## BEC Outputs

| Output | Type                       | Current | Ripple                        |
|--------|----------------------------|---------|-------------------------------|
| 5V     | Synchronous buck (switch)  | 3 A     | 40 mV (Vin=16V, Iout=2A)      |
| 12V    | Linear regulator           | 0.5 A   | Очень низкий (не мешает видео) |

**12V при 4S:** до 500 mA, точность ±2.5%.  
**12V при 3S:** выходное напряжение = напряжение батареи − 1V.

## Usage in ClaudeDrone

| Consumer         | Rail | Notes                                    |
|------------------|------|------------------------------------------|
| 4-in-1 ESC       | VBAT | Напрямую с батарейных падов              |
| Raspberry Pi Zero 2W | 5V | Через BEC 5V/3A                     |
| ESP32 DevKit     | 5V   | Через BEC 5V/3A                         |
| PCA9685 PWM driver | 5V | Через BEC 5V/3A                        |

## Safety Notes

- Соблюдать полярность — обратное включение уничтожит плату
- Не касаться карбоновой рамы снизу платы — риск короткого замыкания
- XT60 разъём в комплекте — требует пайки пользователем
- Совместима с рамами шириной верхней пластины < 48 mm

## Files

| File                    | Description            |
|-------------------------|------------------------|
| `PDB-Specification.txt` | Исходные спецификации  |
| `../../media/photos/components/PDB-1.jpg` | Фото платы (вид 1) |
| `../../media/photos/components/PDB-2.jpg` | Фото платы (вид 2) |
