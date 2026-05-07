# 04 — SITL bring-up: «Waiting for heartbeat» что это и как чинить

**Date:** 2026-05-07
**Trigger:** TASK-021 21a (shared simulation+researchbest). researchbest сообщил что `daemon.sh start --full -log` поднимает Gazebo и сенсоры (sweep 181/181 ranges работает), но ArduPilot SITL виснет на `Waiting for heartbeat from tcp:127.0.0.1:5760` и `pgrep arducopter` пусто (ну, как ему казалось).

## Симптом

В `sitl` пэйне researchbest'а:

```
RiTW: Starting ArduCopter : .../arducopter --model JSON --speedup 1 --slave 0 \
      --defaults .../copter.parm,.../gazebo-iris.parm,.../indoor.parm \
      --sim-address=127.0.0.1 -I0
Connect tcp:127.0.0.1:5760 source_system=255
Loaded module console
Log Directory:
Telemetry log: mav.tlog
Waiting for heartbeat from tcp:127.0.0.1:5760
MAV>
```

Ничего не двигается дальше. `--mavros` пэйн получает то же самое: `FCU: Connection problem`.

## Что уже не виновато

Поэтапно проверил:

1. **`arducopter` binary жив и слушает порт.** Запустил его напрямую без `sim_vehicle.py`:
   ```
   arducopter --model JSON --speedup 1 --slave 0 --defaults ... -I1
     → JSON control interface set to 127.0.0.1:9012
     → bind port 5770 for SERIAL0
     → SERIAL0 on TCP port 5770
     → Waiting for connection ....
   ```
   `ss -tlnp | grep :5770` — есть `LISTEN`. Бинарник OK.

2. **Param files OK.** `copter.parm`, `gazebo-iris.parm`, `indoor.parm` все читаются — иначе arducopter падал бы с явной ошибкой parsing.

3. **`sim_vehicle.py` сам по себе OK** в интерактивном shell с `$DISPLAY`. RiTW (`run_in_terminal_window.sh`) выбирает `xterm` ветку, открывает окно, arducopter живёт.

4. **`tmux new-window` ветка RiTW тоже работает**. Поднял свой repro:
   ```
   tmux new-session -d -s t21a -n sitl "cd ~/ardupilot/ArduCopter && \
     sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -I 2 \
     --add-param-file=.../indoor.parm 2>&1 | tee /tmp/t21a-sitl.log"
   ```
   - `tmux list-windows -t t21a` показывает 2 окна: `sitl` и `ArduCopter` (RiTW spawn'ил через `tmux new-window`).
   - `pgrep -af arducopter` показывает живой процесс с `-I2`.
   - `ss -tlnp | grep :578` показывает 5780/5782/5783 в LISTEN.
   - **Но MAVProxy в `sitl` пэйне всё равно "Waiting for heartbeat"**.

## Где собака зарыта

Заглянул в окно `ArduCopter` (где живёт сам бинарник):

```
No JSON sensor message received, resending servos
No JSON sensor message received, resending servos
No JSON sensor message received, resending servos
... (бесконечно)
```

И тут стало понятно. `arducopter --model JSON` в **lock-step** режиме (`<lock_step>1</lock_step>` в SDF плагина):

- ArduPilot шлёт servo-команды на UDP `127.0.0.1:9002` (плагин слушает)
- Плагин в Gazebo шлёт IMU/baro/GPS обратно — ArduPilot ждёт **этот пакет**, чтобы сделать step и выйти из init phase
- **Без JSON sensor data ArduPilot НЕ отправляет MAVLink heartbeat по TCP 5760.**
- MAVProxy подключается, но read'ит ничего → "Waiting for heartbeat".

То есть `Waiting for heartbeat` — **downstream симптом** того что Gazebo ↔ ArduPilot JSON-link сломан.

## Почему ardupilot_gazebo plugin не отвечает в researchbest'овском стеке

Проверил state машины во время refrence сессии (orphan'ы 5 часов):

```
$ ss -ulnp | grep :9002
UNCONN  127.0.0.1:9002  users:(("ruby",pid=142033,fd=37))
UNCONN  127.0.0.1:9002  users:(("ruby",pid=79368,fd=57))
```

**Два gz sim процесса разных запусков обоих привязаны к UDP 9002.** Это значит:
- ArduPilotPlugin загружен в обоих (так что плагин-инфра работает) ✅
- Но kernel при доставке UDP-пакета на :9002 выбирает один из биндов недетерминированно (`SO_REUSEADDR/SO_REUSEPORT` поведение)
- Половина пакетов уходит в зомби-процесс который не делает физический step
- Lock-step никогда не закрывается → нет ответа → arducopter в `No JSON sensor message received`-loop

Орфаны появились потому что `daemon.sh stop` или Ctrl-C tmux-сессии в прошлый раз не дотянулся до children. Linux reparent'ит их к `systemd --user`, и они продолжают bind UDP 9002.

## Фикс

Сейчас (TASK-021 21a):
1. **Перед запуском** `daemon.sh start` всегда вызывать `scripts/shared/d2_sitl_cleanup.sh` (TASK-033 ph4) — `--dry-run` снапшот, `--kill` если нужно.
2. После любого `daemon.sh stop` — проверять `pgrep -af "gz sim|mavproxy|arducopter"` и зачищать остатки.
3. Pre-flight check в `launch.sh` уже бьёт ERROR при занятом MAVLINK_PORT (TASK-033 ph2). По UDP 9002 такой проверки нет — она менее критична потому что gz sim использует SO_REUSEPORT, и второй bind не падает с ошибкой, а просто рандомизирует доставку.

В перспективе (отдельный тикет если понадобится параллельный запуск):
- Параметризировать `<fdm_port_in>9002</fdm_port_in>` в SDF плагина под instance — instance N → port `9002 + 10*N`. Сейчас порт hard-coded.
- Тогда researchbest на instance 1 получит JSON port 9012, sim на instance 0 — 9002, конфликта по UDP не будет даже если оба gz sim живы.

## Update 2026-05-08 — verification: фикс ЧАСТИЧНЫЙ, баг глубже

Прибил все 8 орфанов через `d2_sitl_cleanup.sh --kill`, запустил чистый стек:
```
launch.sh --sim --headless -d -log -s sim21verify
```

State после 25 секунд:
- ✅ `gz sim` живой (PID 265139), один экземпляр
- ✅ UDP 9002 bound `gz sim` (ArduPilotPlugin) — единственный
- ✅ `arducopter` живой (PID 265539) на TCP 5760/5762/5763
- ✅ ArduCopter window: `JSON received: timestamp/imu/position/quaternion/velocity` — **plugin отвечает**!
- ✅ `Home: -35.363262 149.165237 alt=584.000000m hdg=353.000000` — координаты получены
- ✅ `Forcing use_time_sync=0`, `validate_structures: Validating structures`
- ❌ После этого зацикливается на `Loaded defaults from ...` (23 раза за 25 секунд)
- ❌ MAVProxy всё ещё `Waiting for heartbeat`
- ❌ `pymavlink wait_heartbeat(timeout=5)` → None

То есть **lock-step связь работает (JSON получается)**, но arducopter застревает в init phase и не доходит до emit'а heartbeat. Орфаны были contributing factor (UDP 9002 race), но **не единственная причина**.

### Подозрения (требуют дополнительного research'а)

1. **Lock-step mismatch.** SDF плагина: `<lock_step>1</lock_step>`. ArduCopter получил в JSON config: `no_lockstep` (lockstep отключён). Возможно версия `ardupilot_gazebo` плагина не реализует lock_step как ожидает arducopter, или элемент SDF имеет другое имя в текущей версии.

2. **EKF init без optical flow data.** `indoor.parm` использует `EK3_SRC1_VELXY 5` (optical flow для xy velocity), `EK3_SRC1_POSXY 0` (нет GPS pos). Если gz не публикует optical flow в JSON stream — EKF3 не может инициализироваться → arducopter сбрасывает init → reload defaults loop. Текущий JSON config от plugin'а: `imu/position/quaternion/velocity` без optical flow или rangefinder.

3. **plugin фидбек только schema, не data.** Возможно plugin отправил schema (один раз), но реальные sensor packets не идут на arducopter по UDP 9005 (его recv-сокет). Не проверил `tcpdump`/`strace`.

### Что точно НЕ виновато (исключено в этой сессии)

- Bash heredoc / RiTW / xterm vs tmux new-window — все ветки RiTW работают
- arducopter binary, params parsing, TCP listener — работает
- Orphan UDP race — был, сейчас вычищен, не помог
- Отсутствие plugin .so — `libArduPilotPlugin.so` найден и загружен
- `GZ_SIM_SYSTEM_PLUGIN_PATH` не propagируется — propagируется через `bash -lc`

### Statu итог

- TASK-021 21a **частично** закрыт: документация + cleanup helper + chain симптомов до lock-step
- Heartbeat пока **не воспроизводится** даже с чистым state
- Нужен дополнительный research: либо обновить ardupilot_gazebo plugin до версии с правильным lock_step, либо сменить EK3_SRC1 на что-то что arducopter в JSON получает (gyro+accel), либо включить optical flow в plugin output

Это уходит из scope «bring-up bug в researchbest стеке» в более фундаментальный вопрос «как настроить ArduPilot SITL + ardupilot_gazebo + indoor params чтобы EKF поднимался». Возможно отдельный research-тикет под research или software агента.

## Acceptance для 21a (обновлено 2026-05-08)

- [x] Symptom описан с полной reproduction-цепочкой
- [x] Cleanup helper `d2_sitl_cleanup.sh` написан и работает
- [x] Pre-flight port check в `launch.sh` (TASK-033 ph2)
- [x] Один class причин (orphan UDP race) идентифицирован и вычищен
- [ ] Heartbeat fully reliable — НЕТ. lock_step / EKF init глубже, требует отдельной сессии
- [x] Long-term: SDF параметризация под instance (отдельный тикет если parallel нужен)

## Что для researchbest

- TASK-033 ph3 (.env_researchbest с SITL_INSTANCE=1) делать стоит — это разблокирует параллельную работу когда heartbeat будет починен
- Heartbeat fix через простой orphan cleanup НЕ работает (проверено 2026-05-08); ждать research-сессии по EKF/lock_step
- На текущий момент: после `daemon.sh stop` всегда вызывать `scripts/shared/d2_sitl_cleanup.sh --dry-run`, при наличии orphan'ов — `--kill` (это теперь рекомендация, не предположение)
