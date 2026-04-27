takeoff_node_v1.py
## Разбор takeoff_node — Глава 1: Первый вариант

### Теория: как вообще работает управление дроном через ROS2

Прежде чем смотреть на код — нужно понять три механизма общения которые мы использовали.

**Механизм 1: Topics (Топики) — асинхронная публикация**

Это как радио. Один передаёт, все кто настроены — получают. Отправитель не знает кто слушает и не ждёт ответа.

```
takeoff_node                          MAVROS                    ArduPilot
──────────                           ──────                    ─────────
publish(PoseStamped)  ──→  /mavros/setpoint_position/local
                                          │
                                     переводит в MAVLink
                                          │
                                          └──→  SET_POSITION_TARGET_LOCAL_NED
```

Мы использовали топик для двух вещей:
- **Публикация** setpoint (куда лететь) → `/mavros/setpoint_position/local`
- **Подписка** на состояние дрона → `/mavros/state`

**Механизм 2: Services (Сервисы) — синхронный запрос/ответ**

Это как телефонный звонок. Ты звонишь, ждёшь ответа, получаешь результат. Но мы использовали `call_async` — асинхронный вызов. Ты звонишь и не блокируешь программу пока ждёшь.

```
takeoff_node                          MAVROS                    ArduPilot
──────────                           ──────                    ─────────
call_async(arm=True)  ──→  /mavros/cmd/arming
                                          │
                                     MAVLink: COMMAND_LONG
                                     CMD_COMPONENT_ARM_DISARM
                                          │
                                          └──→  ArduPilot обрабатывает
                                          ←──  COMMAND_ACK: ACCEPTED/DENIED
```

Мы использовали сервисы для команд:
- `/mavros/cmd/arming` — включить моторы
- `/mavros/set_mode` — сменить режим полёта
- `/mavros/cmd/takeoff` — команда взлёта

**Механизм 3: MAVLink — язык ArduPilot**

ArduPilot не знает что такое ROS2. Он знает только MAVLink — бинарный протокол. Каждое сообщение MAVLink — это пакет байт с конкретным ID.

```
Наш Python код → ROS2 топик → MAVROS переводит → MAVLink пакет → ArduPilot
```

Например арминг:
```
CommandBool(value=True)
    ↓ MAVROS
MAVLink: COMMAND_LONG (cmd=400, param1=1.0)
    ↓ UDP
ArduPilot получает, проверяет, отвечает
    ↓ MAVLink
COMMAND_ACK (result=0 = ACCEPTED)
    ↓ MAVROS
ROS2 service response
```

---

### Первый вариант кода — что я хотел сделать

```python
# Идея была простая:
# 1. Всегда слать setpoint (ArduPilot требует поток команд)
# 2. При подключении — один раз сделать takeoff последовательность

self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

def timer_callback(self):
    # Шлём setpoint ВСЕГДА
    self.setpoint_pub.publish(self.target)
    
    if not self.takeoff_done:
        self.takeoff_done = True
        self.create_timer(2.0, self.do_takeoff)  # ← ПРОБЛЕМА
```

**Почему выбрал такой подход:**

Официальная документация MAVROS говорит: *"перед переходом в OFFBOARD режим нужно слать setpoint потоком, иначе переход откажет"*. Я перенёс эту логику на GUIDED режим ArduPilot. Setpoint шлётся всегда → накапливается буфер → потом arm и takeoff.

---

### Что не сработало и почему

**Баг 1: вложенные таймеры**

```python
def timer_callback(self):          # вызывается каждые 0.1 сек
    if not self.takeoff_done:
        self.takeoff_done = True
        self.create_timer(2.0, self.do_takeoff)  # создаём таймер
    
def do_takeoff(self):
    self.create_timer(1.0, self.do_arm)  # создаём ещё таймер
    
def do_arm(self):
    self.arming_client.call_async(arm_req)
    # ← нет защиты от повторного вызова!
```

`create_timer` в ROS2 создаёт **постоянный повторяющийся таймер**. Это не `setTimeout` из JavaScript который срабатывает один раз. `do_arm` вызывался каждую секунду бесконечно — дрон получал команду arm снова и снова.

**Что ты должен был увидеть в логах:**

```
Arming...        ← первый раз
Takeoff to 1.5m!
Arming...        ← второй раз через секунду
Takeoff to 1.5m!
Arming...        ← третий...
```

Это **спам командами**. ArduPilot получал arm/disarm/arm/disarm настолько быстро что просто дисармировался от confusion.

**Баг 2: setpoint конфликтовал с takeoff**

```python
# Мы всегда шлём setpoint z=1.5
self.target.pose.position.z = 1.5
self.setpoint_pub.publish(self.target)
```

Проблема: дрон стоит на земле (z=0.2). Мы шлём "лети на z=1.5". ArduPilot в этот момент ещё не armed. Он получает setpoint но не может его выполнить. Когда потом шлём takeoff — у него уже есть конфликтующий setpoint в буфере.

**Что ты должен был увидеть в логах MAVProxy:**

```
ARMED
AP: Disarming motors   ← сразу после arm
DISARMED
```

Это ArduPilot говорит: *"я вооружился, получил команду взлёта, но setpoint конфликтует с моим внутренним состоянием — дисармируюсь для безопасности"*.

---

### Урок из первого варианта

Первый вариант научил нас двум важным вещам:

**1. `create_timer` в ROS2 ≠ одноразовый таймер**

```python
# Это НЕ то что ты думаешь:
self.create_timer(1.0, self.do_arm)  # вызывает do_arm каждую секунду навсегда

# Для одноразового действия нужен счётчик или флаг:
if self.tick == 50 and not self.arm_done:
    self.arm_done = True
    self.do_arm()
```

**2. Setpoint и takeoff — конкуренты**

ArduPilot обрабатывает команды в очереди. Если ты шлёшь setpoint 10 раз в секунду И команду takeoff — они конкурируют. Нужно чётко разделить фазы:

```
До arm:    → не шли setpoint
После arm: → сначала takeoff команда
Летим:     → тогда setpoint для удержания позиции
```

Готов к разбору второго варианта?

takeoff_node_v2.py
## Глава 2: Второй вариант — State Machine

### Что изменилось архитектурно

Первый вариант использовал вложенные таймеры. Второй вариант ввёл важную концепцию — **State Machine (конечный автомат)**.

Помнишь из roadmap документа — мы говорили про State Machine для полётных шаблонов? Вот первое её применение прямо здесь.

```
step=0 (wait)
    │ counter > 30
    ▼
step=1 (guided)
    │ counter > 50
    ▼
step=2 (arm)
    │ counter > 70 AND armed=True
    ▼
step=3 (flying)
```

Каждое состояние — это конкретная фаза. Переход только вперёд, только по условию.

---

### Что я хотел исправить

Убрать вложенные таймеры. Вместо них — **один таймер + счётчик**:

```python
self.counter = 0
self.create_timer(0.1, self.timer_callback)  # один таймер навсегда

def timer_callback(self):
    self.counter += 1  # растёт каждые 0.1 сек
    
    if self.step == 0 and self.counter > 30:  # 30 * 0.1 = 3 сек
        # делаем один раз и переходим в step=1
        self.step = 1
```

Это чище — один таймер, никаких вложений. Логика переходов прозрачная.

---

### Что всё равно не сработало

Баг из первого варианта остался — **setpoint шлётся всегда, даже до arm**:

```python
def timer_callback(self):
    self.counter += 1

    # ← Это выполняется КАЖДЫЙ тик, независимо от step
    self.target.header.stamp = self.get_clock().now().to_msg()
    self.setpoint_pub.publish(self.target)  # z=1.5 шлётся с tick=0
```

**Что происходило в ArduPilot:**

```
tick 0-29:   setpoint z=1.5 → ArduPilot получает, дрон на земле, игнорирует
tick 30:     SetMode GUIDED → OK
tick 50:     Arm → ACCEPTED
tick 50-70:  setpoint z=1.5 продолжает идти
             ArduPilot: "я вооружён, получаю setpoint,
                         но взлёт не был инициирован через takeoff команду"
             → DISARM (защита)
```

**Ключевой момент который нужно понять:**

В ArduPilot GUIDED режиме есть два разных сценария:

```
Сценарий A (правильный):
arm → takeoff команда → набор высоты → потом setpoint для удержания

Сценарий B (наш):
setpoint z=1.5 → arm → (нет takeoff команды) → ArduPilot confused → DISARM
```

ArduPilot ожидает явную команду `NAV_TAKEOFF` для взлёта с земли. `setpoint_position` — это команда для **уже летящего** дрона. Мы использовали инструмент не по назначению.

---

### Что ты должен был увидеть в логах

В MAVProxy:
```
ARMED
AP: Disarming motors   ← немедленно
DISARMED
```

В ROS2 ноде:
```
Setting GUIDED mode...
Arming...
Not armed yet, retrying...   ← потому что уже DISARMED
Not armed yet, retrying...
Not armed yet, retrying...
```

Это бесконечный retry — нода видит `armed=False` и пытается армировать снова, но ArduPilot снова дисармирует.

---

### Урок из второго варианта

**State Machine — правильный паттерн**, но состояния были неполные. Не хватало состояния `takeoff` между `arm` и `flying`.

```
# Было:
wait → guided → arm → flying

# Нужно было:
wait → guided → arm → takeoff → flying
                         ↑
                    явная команда NAV_TAKEOFF
                    перед setpoint
```

**Важное понимание про setpoint:**

`/mavros/setpoint_position/local` — это топик для режима **OFFBOARD** в PX4. В ArduPilot GUIDED режиме он работает иначе — только когда дрон уже в воздухе. На земле ArduPilot его игнорирует или воспринимает как конфликт.

Готов к третьей главе?

## Глава 3: Третий вариант — добавили CommandTOL

### Что такое CommandTOL

`CommandTOL` — это ROS2 обёртка над MAVLink командой `NAV_TAKEOFF` (ID=22). Именно то что MAVProxy делает когда ты пишешь `takeoff 2`.

```
CommandTOL.Request()        MAVROS           ArduPilot
─────────────────          ──────           ─────────
altitude = 1.5    ──→   MAVLink MSG     ──→  NAV_TAKEOFF
                         cmd=22              param7=1.5 (высота)
                         
                    ←──  COMMAND_ACK    ←──  ACCEPTED
```

Ты правильно заметил — в предыдущем коде шаги были (`step=0,1,2,3`) но **не было реального вызова takeoff**. Дрон армировался и... просто стоял вооружённым, ждал команды которая никогда не приходила.

---

### Почему даже с CommandTOL не взлетел

Смотрим на порядок в третьем варианте:

```python
# tick 0:   начинаем слать setpoint z=1.5  ← ВСЁ ЕЩЁ ЗДЕСЬ
# tick 30:  GUIDED
# tick 50:  ARM
# tick 70:  CommandTOL(altitude=1.5)
```

**Проблема та же** — setpoint шлётся с самого начала. Вот что видел ArduPilot:

```
t=0s:    setpoint z=1.5 (дрон на земле)
t=3s:    GUIDED mode
t=5s:    ARM → ACCEPTED → ARMED
t=5s:    setpoint z=1.5 продолжает идти (10 раз в секунду!)
t=7s:    NAV_TAKEOFF altitude=1.5 → ACCEPTED
         ArduPilot: "у меня конфликт — 
                     setpoint говорит z=1.5
                     И takeoff говорит лети на 1.5м
                     → DISARM"
```

Два источника команд одновременно — ArduPilot выбирает безопасность.

---

### Про BATT_MONITOR 0 и другие параметры

Хороший вопрос. Давай разберём все параметры из `indoor.parm`:

```bash
# GPS отключён — правильно, у нас indoor
GPS1_TYPE 0

# Арминг без проверок — нужен для симуляции
ARMING_SKIPCHK 1

# Высота с rangefinder — пока не работает без моста
# Можно оставить на будущее когда подключим MAVROS→ArduPilot мост
EK3_SRC1_POSZ 2

# Optical flow — тоже на будущее
FLOW_TYPE 6
EK3_SRC1_VELXY 5
EK3_SRC1_POSXY 0

# Медленный полёт indoor — оставить
LOIT_SPEED_MS 3.0
PILOT_SPD_UP 1.0

# Fence — оставить, но пока не работает без GPS/rangefinder
FENCE_ENABLE 1
FENCE_TYPE 2
FENCE_ALT_MAX 2.5
FENCE_ACTION 4

# Батарея отключена — ОСТАВИТЬ для симуляции
BATT_MONITOR 0
```

**Итог по параметрам:**

| Параметр | Оставить? | Почему |
|---|---|---|
| `GPS1_TYPE 0` | ✅ | Indoor, GPS не нужен |
| `ARMING_SKIPCHK 1` | ✅ | Симуляция, проверки мешают |
| `EK3_SRC1_POSZ 2` | ✅ | Задел на будущее (Шаг 7) |
| `BATT_MONITOR 0` | ✅ | Симуляция, батареи нет |
| `FENCE_*` | ✅ | Задел, заработает с rangefinder |
| `LOIT_SPEED_MS` | ✅ | Полезно для indoor |

---

### Урок из третьего варианта

Мы правильно добавили инструмент (`CommandTOL`) но не убрали мешающий (`setpoint` до arm).

**Главный принцип который мы вывели:**

```
Setpoint — для летящего дрона
CommandTOL — для взлёта с земли
Они не должны работать одновременно
```

Именно поэтому четвёртый вариант разделил их по фазам:

```python
phase = 'wait'     → ничего не шлём
phase = 'guided'   → ничего не шлём  
phase = 'arm'      → ничего не шлём
phase = 'takeoff'  → только CommandTOL
phase = 'hover'    → только setpoint
```

Готов к четвёртому — финальному варианту?


takeoff_node_v4.py
## Глава 4: Четвёртый вариант — шаг назад

### Что изменилось по сравнению с третьим

Ты правильно заметил — `CommandTOL` снова убран. Посмотрим почему я это сделал и к чему это привело.

**Изменения в логике:**

```python
# Было в варианте 2 (step машина):
if self.step == 0 and self.counter > 30:   # только один раз
    self.step = 1

# Стало в варианте 4 (tick машина):
if self.tick == 30:    # ровно на тике 30
    ...
if self.tick == 50:    # ровно на тике 50
    ...
if self.tick == 70:    # ровно на тике 70
    ...
```

Разница тонкая но важная:

```
Вариант 2: step=0 И counter>30 → переход, step=1
           следующие тики: step≠0, условие больше не выполняется ✅

Вариант 4: tick==30 → команда
           tick==50 → команда  
           НО tick==30 никогда не повторится ✅
           Зато нет защиты от пропуска тика ⚠️
```

---

### Почему я убрал CommandTOL

Моя логика была такая — раз `CommandTOL` с setpoint конфликтуют, попробуем вообще без `CommandTOL`. Идея: ArduPilot в GUIDED режиме должен сам подняться до setpoint после arm.

```
Теория:
arm → setpoint z=1.5 уже шлётся → ArduPilot видит цель → летит

Практика:
arm → setpoint z=1.5 шлётся ДО arm → конфликт → DISARM
```

Это была **неверная гипотеза**. Setpoint в GUIDED — не команда взлёта. Это команда удержания позиции для уже летящего дрона.

---

### Что происходило в системе

Давай пройдём по каждому тику и посмотрим что видел каждый компонент:

```
НАША НОДА          MAVROS              ArduPilot
──────────         ──────              ─────────

tick 1-29:
publish(z=1.5) →  setpoint топик  →   "получаю позицию, 
                                        но не armed, игнорирую"

tick 30:
SetMode(GUIDED) → MAVLink           → Mode GUIDED ✅
                  SET_MODE
                  ← ACK ACCEPTED

tick 31-49:
publish(z=1.5) →  setpoint топик  →   "GUIDED, не armed,
                                        жду команды взлёта"

tick 50:
Arm(True)      →  MAVLink           → проверяет prearm
                  CMD_ARM_DISARM      "prearm good"
                  ← ACK ACCEPTED    → ARMED ✅

tick 51-70:
publish(z=1.5) →  setpoint топик  →   "ARMED + setpoint
                                        но не было NAV_TAKEOFF
                                        → это опасно
                                        → DISARM" ❌

tick 70:
check armed      state=False        
"Не вооружился!"
tick = 55       ← откат назад
```

**Что ты видел на экране:**

```
Нода запущена, ждём подключения...
Переключаем в GUIDED...
Армируем...
Не вооружился! Пробуем ещё раз...   ← бесконечно
Не вооружился! Пробуем ещё раз...
```

И в MAVProxy:
```
ARMED
AP: Disarming motors
DISARMED
ARMED
AP: Disarming motors
DISARMED
```

---

### Ключевое открытие этого варианта

Именно здесь мы чётко увидели паттерн — **дрон армируется и сразу дисармируется**. Это дало нам понять что проблема не в arming команде, а в том что происходит **после** arm.

```
Вопрос который мы задали:
"Почему дрон дисармируется если команда ACCEPTED?"

Ответ:
ArduPilot принимает arm → ждёт команды взлёта → 
если через N секунд нет NAV_TAKEOFF → считает аварией → DISARM
```

Это защита называется **автодисарм при бездействии**. ArduPilot не будет держать моторы вооружёнными бесконечно если нет команды двигаться.

---

### Флаг armed_once — неиспользованный инструмент

```python
self.armed_once = False  # объявлен в __init__
```

Этот флаг я добавил с идеей отслеживать первый arm. Но **нигде не использовал его**. Это типичная ошибка при итеративной разработке — добавил задел, забыл подключить.

В финальном варианте мы заменили этот флаг на `phase` строку которая явно показывает состояние:

```python
# Было:
self.armed_once = False  # булевый флаг — мало информации

# Стало:
self.phase = 'wait'  # строка — сразу видно в каком состоянии
```

---

### Итоговая таблица всех четырёх вариантов

```
Вариант │ Setpoint до arm │ CommandTOL │ Таймеры      │ Результат
────────┼─────────────────┼────────────┼──────────────┼──────────
1       │ ДА              │ НЕТ        │ вложенные    │ спам команд
2       │ ДА              │ НЕТ        │ один+счётчик │ DISARM сразу
3       │ ДА              │ ДА         │ один+счётчик │ конфликт
4       │ ДА              │ НЕТ        │ один+tick    │ DISARM сразу
5 ✅    │ НЕТ (до hover)  │ ДА         │ один+phase   │ ЛЕТИТ
```

Видишь паттерн? Во всех нерабочих вариантах одна общая черта — **setpoint шлётся до arm**. Финальный вариант исправил именно это.

Готов к финальной главе — разбору рабочего варианта?
## Глава 5: Финальный вариант — всё что мы поняли

### Как я пришёл к этому решению

Посмотри на эволюцию одной строки через все варианты:

```python
# Вариант 1, 2, 3, 4:
def loop(self):
    self.sp_pub.publish(self.target)  # ВСЕГДА, с тика 0
    ...

# Вариант 5:
def loop(self):
    # setpoint НЕ шлётся здесь
    ...
    elif self.phase == 'hover':
        self.sp_pub.publish(self.target)  # только здесь
```

Четыре попытки чтобы понять одно правило. Это и есть исследование — не читать документацию, а **получить отказ системы и понять почему**.

---

### Архитектура ноды — полная карта

Прежде чем разбирать код — нарисуем полную картину кто с кем говорит:

```
╔══════════════════════════════════════════════════════╗
║                   takeoff_node                        ║
║                                                       ║
║  ПОДПИСКИ (слушаем)                                   ║
║  /mavros/state ──────────────→ state_cb()            ║
║                                self.state            ║
║                                                       ║
║  ПУБЛИКАЦИИ (говорим)                                 ║
║  sp_pub ─────────────────────→ /mavros/setpoint_     ║
║                                 position/local       ║
║                                                       ║
║  СЕРВИСЫ (запрос/ответ)                              ║
║  mode_srv ────────────────────→ /mavros/set_mode     ║
║  arm_srv  ────────────────────→ /mavros/cmd/arming   ║
║  takeoff_srv ─────────────────→ /mavros/cmd/takeoff  ║
║                                                       ║
║  ТАЙМЕР                                               ║
║  create_timer(0.1) ──────────→ loop() каждые 0.1с   ║
╚══════════════════════════════════════════════════════╝
         ↕ всё через MAVROS ↕
╔══════════════════════════════════════════════════════╗
║                    ArduPilot SITL                     ║
╚══════════════════════════════════════════════════════╝
```

---

### Подписки — как мы слушаем систему

```python
self.create_subscription(
    State,              # тип сообщения
    '/mavros/state',    # имя топика
    self.state_cb,      # callback функция
    10)                 # QoS — глубина очереди
```

**Что происходит под капотом:**

```
ArduPilot                MAVROS               Наша нода
─────────               ──────               ─────────
HEARTBEAT (1Hz)  ──→   читает MAVLink  ──→  публикует State
STATUSTEXT       ──→   переводит       ──→  в /mavros/state
SYS_STATUS       ──→                        ↓
                                        state_cb(msg) вызывается
                                        self.state = msg  сохраняем
```

**QoS число 10** — это глубина очереди. Если наша нода занята и не успевает читать — ROS2 буферизует до 10 сообщений. Лишние выбрасывает.

**Что мы читаем из State:**

```python
self.state.connected  # bool — MAVROS видит ArduPilot?
self.state.armed      # bool — моторы вооружены?
self.state.mode       # string — "GUIDED", "STABILIZE", etc.
```

Это наши "глаза" — без подписки на state мы слепые. Мы бы слали команды не зная в каком состоянии дрон.

**Урок из наших ошибок:** в вариантах 1-4 мы читали `connected` и `armed` но **не ждали нужного состояния перед следующим шагом**. Переходили по времени (tick>30) а не по факту (mode=='GUIDED').

---

### Публикации — как мы говорим дрону куда лететь

```python
self.sp_pub = self.create_publisher(
    PoseStamped,                          # тип
    '/mavros/setpoint_position/local',    # топик
    10)                                   # QoS
```

**PoseStamped** — это структура:

```python
PoseStamped:
  header:
    stamp:    # время сообщения ← ОБЯЗАТЕЛЬНО обновлять
    frame_id: # система координат (map, base_link...)
  pose:
    position:
      x, y, z   # метры
    orientation:
      x, y, z, w  # кватернион (поворот)
```

**Почему важно обновлять stamp:**

```python
# В финальном варианте:
self.target.header.stamp = self.get_clock().now().to_msg()
self.sp_pub.publish(self.target)
```

ArduPilot смотрит на timestamp. Если время старое — считает что сообщение устаревшее и игнорирует. Это защита от "залежавшихся" команд.

**Ключевое открытие — setpoint только в hover фазе:**

```python
elif self.phase == 'hover':
    self.target.header.stamp = self.get_clock().now().to_msg()
    self.sp_pub.publish(self.target)  # ← только здесь
```

Все четыре провала были потому что setpoint шёл с тика 0. ArduPilot получал "лети на 1.5м" пока дрон стоял на земле. После arm — видел конфликт между своим внутренним состоянием "стою на земле" и нашей командой "ты должен быть на 1.5м" — и дисармировался.

---

### Сервисы — синхронные команды управления

Три сервиса с разными задачами:

**SetMode — смена режима:**

```python
self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')

req = SetMode.Request()
req.custom_mode = 'GUIDED'
self.mode_srv.call_async(req)
```

```
Наша нода          MAVROS             ArduPilot
──────────        ──────             ─────────
SetMode.Request → MAVLink           → обрабатывает
custom_mode=     COMMAND_LONG         проверяет можно ли
'GUIDED'         cmd=176               сменить режим
                 param1=1 (custom)   ← COMMAND_ACK
                 ← SetMode.Response   result=0 (ACCEPTED)
                   success=True
```

**CommandBool — арминг:**

```python
self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')

req = CommandBool.Request()
req.value = True   # True=arm, False=disarm
self.arm_srv.call_async(req)
```

```
CommandBool(True) → CMD_COMPONENT_ARM_DISARM → ArduPilot проверяет:
                                                - prearm checks
                                                - режим GUIDED?
                                                - EKF healthy?
                                                → ARMED или DENIED
```

**CommandTOL — взлёт:**

```python
self.takeoff_srv = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

req = CommandTOL.Request()
req.altitude = 1.5   # метры над землёй
self.takeoff_srv.call_async(req)
```

```
CommandTOL(1.5) → MAVLink NAV_TAKEOFF → ArduPilot:
                   cmd=22                - уже armed?
                   param7=1.5           - тогда взлетаем
                   (высота)             → моторы набирают обороты
                                        → дрон поднимается до 1.5м
                                        → переходит в hover
```

**Почему `call_async` а не `call`:**

```python
# Синхронный вызов — БЛОКИРУЕТ ноду:
response = self.arm_srv.call(req)  # ждём ответа, таймер стоит

# Асинхронный вызов — НЕ блокирует:
self.arm_srv.call_async(req)  # отправили и забыли, таймер продолжает
```

Наш `loop()` вызывается каждые 0.1 сек. Если бы мы использовали синхронный `call` — нода зависала бы на каждой команде. `call_async` отправляет запрос и немедленно возвращает управление.

---

### Таймер — сердце ноды

```python
self.create_timer(0.1, self.loop)  # 0.1 сек = 10 Hz
```

**Что такое 10 Hz и почему именно столько:**

```
1 Hz  = 1 раз в секунду   ← слишком медленно, дрон не успевает реагировать
10 Hz = 10 раз в секунду  ← достаточно для управления
50 Hz = 50 раз в секунду  ← для IMU данных
1000 Hz = 1000 раз        ← для моторов
```

MAVLink HEARTBEAT шлётся с 1 Hz. Setpoint должен идти минимум 2 Hz иначе ArduPilot считает потерю связи. Мы используем 10 Hz — с запасом.

---

### Phase Machine — главный паттерн

```python
self.phase = 'wait'  # начальное состояние
```

Это эволюция State Machine из варианта 2. Вместо цифр (0,1,2,3) — строки. Почему строки лучше:

```python
# Цифры — непонятно:
if self.step == 2 and self.counter > 70:

# Строки — читаемо:
elif self.phase == 'arm' and self.tick > 70:
```

**Полный граф переходов:**

```
'wait'
  │ условие: tick > 30
  │ действие: SetMode(GUIDED)
  ▼
'guided'
  │ условие: tick > 50
  │ действие: Arm(True)
  ▼
'arm'
  │ условие: tick > 70 AND state.armed == True
  │ действие: CommandTOL(altitude=1.5)
  │
  │ если armed == False:
  │   tick = 55, phase = 'guided'  ← откат на шаг назад
  ▼
'takeoff'
  │ условие: tick > 100
  │ действие: ничего, просто ждём пока дрон поднимется
  ▼
'hover'
  │ действие: publish(setpoint) каждый тик
  │ лог каждые 50 тиков
  ▼
  (бесконечно)
```

**Откат при неудаче:**

```python
else:
    self.get_logger().warn('Не вооружился, повтор...')
    self.tick = 55
    self.phase = 'guided'
```

Это важный паттерн — если шаг не выполнился, откатываемся назад и пробуем снова. Не зависаем навсегда.

---

### Резюме всего исследования

Мы не просто написали ноду — мы опытным путём вывели четыре правила:

**Правило 1: Setpoint только для летящего дрона**
```
❌ setpoint с тика 0
✅ setpoint только после подтверждения взлёта
```

**Правило 2: Строгий порядок команд**
```
❌ arm → setpoint (конфликт)
✅ guided → arm → takeoff → setpoint
```

**Правило 3: Один таймер, явные фазы**
```
❌ вложенные create_timer (повторяются бесконечно)
✅ один таймер + phase строка
```

**Правило 4: Проверяй состояние перед следующим шагом**
```
❌ tick > 70 → assume armed
✅ tick > 70 AND state.armed == True → следующий шаг
```

Эти четыре правила мы **не прочитали в документации**. Мы их **извлекли из отказов системы**. Именно так работают настоящие разработчики робототехники — система говорит тебе что не так через своё поведение, ты читаешь логи и делаешь вывод.

Следующий шаг?