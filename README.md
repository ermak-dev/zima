# Roborock V1 → ROS 2 / Nav2

Превращаем root-нутый Roborock V1 в мобильную базу ROS 2: штатный сервер
**Player** на роботе остаётся драйвером железа, тонкий Rust-мост публикует его
данные в ROS 2, а навигацию считает **Nav2**. Цель — поездка в заданную точку
по карте.

> Перед любыми командами движения прочитайте раздел «Безопасность».

## Содержание

- [Архитектура](#архитектура)
- [Требования](#требования)
- [Установка](#установка)
- [Команды CLI](#команды-cli-robot)
- [Навигация: карта и поездка](#навигация-от-карты-до-поездки)
- [Конфигурация и скрипты](#конфигурация-и-скрипты)
- [Разработка и тесты](#разработка-и-тесты)
- [Безопасность](#безопасность)

## Архитектура

```
┌─ рабочая машина (x86, ROS 2 Jazzy) ────────────────────────────┐
│                                                                │
│  bin/robot  ──ssh──►  disable / mask hold / guard              │
│  (управление)         (снять RoboController, владеть маской)   │
│                                                                │
│  player_to_ros2 (Rust) ══прямой TCP :6665══► Player            │
│    /scan    ◄── laser:0                                        │
│    /odom    ◄── position2d:0        ┌────────────────────────┐ │
│    /cmd_vel ──► position2d:0   ◄────│ Nav2 / SLAM / RViz     │ │
│    /tf (odom→base_link→laser)       └────────────────────────┘ │
└────────────────────────────────────────────────────────────────┘
                                  │
┌─ робот (ARMv7, Ubuntu 14.04) ────┴─────────────────────────────┐
│  player           штатный сервер Player (TCP 6665)             │
│  RoboController   штатный владелец режимов (на сессию снимаем) │
│  rrwatchdoge      супервизор процессов (перезапускает упавших) │
│  robot-supervisor наш статический ARM-бинарь — владелец маски  │
│                   и аварийный откат                            │
└────────────────────────────────────────────────────────────────┘
```

Ключевые решения (обоснованы в `PLAN.md` и `LESSWRONG.md`):

* **Мост живёт на рабочей машине, а не на роботе.** Робот — Ubuntu 14.04 с
  EGLIBC 2.19 и 512 МБ RAM: стек ROS 2 Jazzy там не запустится. Трафик
  тривиален (~10 КБ/с сканы), задержка LAN ничтожна.
* **На роботе — только `mask-owner` / `robot-supervisor`.** Владелец маски
  обязан быть локальным: он работает с внутренними FIFO Player.
* **Аварийный откат независим от сети.** Супервизор следит за heartbeat
  моста: пропал heartbeat (упал мост, оборвалась сеть) → супервизор сам
  возвращает штатное состояние.
* **Лидар без уборки.** Маска `0x10f8ff` включает LDS и колёса, но оставляет
  **обе щётки выключенными** — проверено живьём на роботе.

## Требования

* Рабочая машина: Linux x86_64, **ROS 2 Jazzy** (`/opt/ros/jazzy`), Rust
  (stable), `colcon`, `colcon-cargo`, `colcon-ros-cargo`, `libclang-dev`.
* Робот: root-доступ по ssh через алиас `robot` в `~/.ssh/config`.
* Доступ к Player: TCP `robot:6665` из доверенной сети.

## Установка

### 1. Rust-крейты

```sh
git clone <repo> ~/robot && cd ~/robot/crates
cargo build --release        # robotctl (position/move/stop)
cargo test                   # локальные тесты, робот не нужен
```

ARM-бинарь владельца маски (кросс-сборка под робота):

```sh
cd ~/robot/crates
cargo build -p mask-owner --release --target armv7-unknown-linux-musleabihf
```

### 2. Colcon-рабочее пространство для ROS-моста

Узел `player_to_ros2` требует сгенерированных `rosidl`-биндингов сообщений,
поэтому собирается через `colcon` (в обычном `cargo`-воркспейсе он исключён
из `default-members`):

```sh
mkdir -p ~/robot/ros-ws/src && cd ~/robot/ros-ws
git clone --depth 1 https://github.com/ros2-rust/ros2_rust src/ros2_rust
vcs import src < src/ros2_rust/ros2_rust_jazzy.repos
ln -s ~/robot/crates/player_to_ros2 src/player_to_ros2

source /opt/ros/jazzy/setup.sh
colcon build --packages-up-to sensor_msgs nav_msgs tf2_msgs
colcon build --packages-select player_to_ros2
```

> Путь к этому workspace зашит в `crates/player_to_ros2/Cargo.toml` (зависимость
> `tf2_msgs`). Если разместите его иначе — поправьте путь там.

### 3. Тестовый сервер Player (для интеграционных тестов, необязательно)

```sh
~/robot/tools/get-player.sh    # клонирует и собирает в ~/.cache/robot/player
```

## Команды CLI `robot`

Всё управление роботом идёт через `bin/robot` (удобен симлинк `~/bin/robot`).
CLI говорит с роботом по ssh, а с Player — напрямую по TCP 6665.

```sh
robot status                  # enabled / disabled — состояние контроллера
robot enable                  # вернуть штатный RoboController
robot disable                 # снять RoboController (нужно для mask/nav)

robot position                # свежая одометрия Player (метры, радианы)
robot fan                     # сырой датчик fan_motor_status (IR index 6)
robot stop                    # нулевая скорость
robot move forward 0.5        # ограниченное движение, макс 2 с
robot move backward 0.5
robot move left 0.5           # поворот против часовой
robot move right 0.5
robot move --with-controller forward 0.5   # тест при работающем контроллере

robot mask hold               # держать маску 0x10f8ff (LDS + колёса,
                              #   щётки выключены) до Ctrl-C
robot mask hold --mask 10f0ff # своя маска; контроллер должен быть снят
robot mask restore            # разово вернуть standby-маску

robot bridge                  # надзорная ROS 2-сессия (см. «Навигация»)
robot run                     # RViz + телеметрия + мост: одна команда;
                              #   выход из RViz останавливает робота и
                              #   возвращает штатный контроллер
robot run --sim               # офлайн-плейграунд Nav2 (Gazebo TB3):
                              #   клик-цель в RViz, лимиты как у робота
robot run --sim --slam        # то же, но карта строится по ходу езды
                              #   (SLAM Toolbox вместо готовой карты)
robot run --slam              # живой робот: клик-цель в RViz, Nav2 едет,
                              #   SLAM строит карту; карта сохраняется
                              #   при выходе (см. «Шаг 3»)

robot snapshot ./frame.jpg    # кадр с OpenIPC-камеры (HTTP Basic)

robot bug list                # GitLab-задачи (см. robot bug --help)
```

`robot mask hold` сам заливает ARM-бинарь на робот и запускает его с
guard-процессом: если владелец погибнет, guard вернёт standby-маску.

## Навигация: от карты до поездки

### Шаг 1. Надзорная сессия

```sh
source /opt/ros/jazzy/setup.sh
source ~/robot/ros-ws/install/setup.sh

robot bridge
```

`robot bridge` — это вся сессия одной командой:

1. заливает `robot-supervisor` и `controller-restore.sh` на робот;
2. снимает `RoboController` (`robot disable`);
3. поднимает супервизор, который применяет маску (`LDS + колёса`, щётки
   выключены);
4. запускает `player_to_ros2` с heartbeat на супервизор.

Пока сессия жива, в ROS 2 идут `/scan`, `/odom`, `/tf`, и принимается
`/cmd_vel`. Мост сам ограничивает каждую команду лимитами проекта
(0.1 м/с, 0.4 рад/с) и держит её не дольше `cmd_vel_timeout_s` (1 с):
если команды перестали приходить (упал Nav2, отпущена клавиша), колёса
получают ноль. **Как только мост умирает** (Ctrl-C, обрыв сети, падение) —
супервизор в течение ~5 с сам возвращает штатное состояние: нулевую скорость
→ standby-маску → `RoboController`.

Проверить поток:

```sh
ros2 topic hz /scan     # ~5 Гц
ros2 topic hz /odom     # ~50 Гц
ros2 topic echo /scan --once
```

### Ручное управление с RViz (одна команда)

```sh
robot run            # живой робот
robot run --sim      # офлайн-плейграунд Nav2 (Gazebo TB3), робот не нужен
```

#### Плейграунд `robot run --sim`

Поднимает Gazebo с **моделью самого пылесоса** (`models/roborock_v1/`:
плоский цилиндр 0.345 м, диф-привод, LDS 360 лучей / 5 Гц на штатном
креплении x=−0.100 м — TF-контракт совпадает с живым мостом) и полным
Nav2 + RViz. В RViz: кнопка **2D Goal Pose** → клик по карте → робот
едет. Скорости ограничены в физике модели проектными лимитами (0.1 м/с /
0.4 рад/с), конфиг Нав2 — наш (`nav2_replay.yaml` + оверлей
`config/nav2_sim.yaml`). Окна: 3D-мир Gazebo + RViz; закрытие RViz или
Ctrl-C завершает сессию. Ручки: `ROBOT_SIM_HEADLESS`, `ROBOT_SIM_RVIZ`,
`ROBOT_SIM_PARAMS`, `ROBOT_SIM_ROBOT_SDF`, `ROBOT_SIM_TRIES`. Лог:
`/tmp/robot-sim.log`. Это песочница для отработки «клик — цель» и правки
Nav2-конфигов без живого робота.

С флагом `--slam` вместо готовой карты и AMCL запускается онлайн
**SLAM Toolbox** (`nav2_bringup … slam:=True`, конфиг
`config/slam_online.yaml`): он публикует `/map` и `map → odom`, а карта
растёт по мере езды к кликнутым целям. Планировщик прокладывает путь и в
ещё не открытое пространство (`allow_unknown`). Сохранить карту до выхода:

```sh
ros2 run nav2_map_server map_saver_cli -f ~/robot/maps/<имя> \
    --ros-args -p use_sim_time:=true
```

То же на живом роботе — `robot run --slam` (см. «Шаг 3»).

#### Живой робот `robot run`

`robot run` — вся сессия целиком: сам поднимает `robot bridge`, ждёт поток
лидара, открывает окно RViz (лидар, след одометрии, TF; фрейм `odom`) и
отдельное окно телепульта `teleop_twist_keyboard` (клавиши WASD, скорость
ограничена лимитами проекта: 0.1 м/с и 0.4 рад/с). **Закрытие окна RViz**
(или Ctrl-C в консоли, или гибель моста) останавливает робота: гасится вся
инфраструктура, супервизор возвращает штатный `RoboController`, и `robot
run` проверяет, что вернул (`robot status` → `enabled`). Закрытие одного
окна телепульта сессию не заканчивает — Ctrl-C в нём лишь сбрасывает
ручную скорость. Одно нажатие клавиши двигает робота не дольше 1 с
(срок команды в мосте); удержание — непрерывное движение за счёт
автоповтора клавиатуры. Лог всех компонентов: `/tmp/robot-run.log`. Окружение:
`ROBOT_TELEOP_SPEED`, `ROBOT_TELEOP_TURN`, `ROBOT_RVIZ_CONFIG`,
`ROBOT_RUN_LOG`.

### Шаг 2. Карта (SLAM)

Пишем данные во время сессии:

```sh
# в отдельном терминале при живой сессии robot bridge:
ros2 bag record -o ~/robot-bags/mapping /scan /odom /tf
# ... проезжаем помещение (robot move) ...
# Ctrl-C останавливает запись
```

Карта из записи:

```sh
bash ~/robot/scripts/slam-from-bag.sh     # пишет в ~/robot/maps/
```

Результат: `~/robot/maps/<имя>.pgm` + `.yaml`. Пример готовой карты в репозитории —
`maps/mapping_drive_20261004.yaml`.

> Если запись сделана мостом до фикса штампов времени, прогоните
> `python3 scripts/stamp_bag.py <bag>` — он проставит корректные метки.

### Шаг 3. Поездка в точку с картографией (`robot run --slam`)

```sh
robot run --slam
```

Одна команда поднимает надзорный мост (`robot bridge`), затем SLAM Toolbox
и Nav2 (`navigation_launch.py`, без симуляционного времени) с параметрами
`nav2_replay.yaml` + `nav2_drive.yaml` + `slam_online.yaml` и открывает
RViz со сценой Nav2 (фрейм `map`). Кнопка **2D Goal Pose** → клик по
карте → робот едет, карта растёт. Телепульта в этом режиме нет: он
перебивал бы Nav2 на `/cmd_vel`.

Скорость до колёс проходит четыре слоя: контроллер RPP (0.06 м/с,
0.3 рад/с) → сглаживатель скоростей (потолок для всех источников Nav2,
включая поведения восстановления) → монитор столкновений (режим
`approach`: замедляет так, чтобы до контакта круглого следа оставалось
≥ 1.5 с; скан старше 1 с — остановка) → мост (лимит 0.1 м/с и 0.4 рад/с,
срок команды 1 с). Гибель моста или сети по-прежнему ловит супервизор на
роботе (~5 с).

**Закрытие RViz** (или Ctrl-C, или гибель моста) завершает сессию в
порядке: Nav2 → нулевая скорость → сохранение карты в
`maps/live-<дата-время>.{yaml,pgm}` → SLAM и мост → проверка, что вернулся
`RoboController`. Окружение: `ROBOT_MAP_OUT` (путь карты без расширения),
`ROBOT_SLAM_RVIZ_CONFIG`, `ROBOT_NAV_TRIES`, `ROBOT_RUN_LOG`.

Ограничения: LDS стоит сверху и **не видит низкие препятствия**, датчики
края (cliff) и бампер в ROS не проброшены — у лестниц и порогов не
отпускайте робота без присмотра. Поездка по готовой карте с AMCL одной
командой пока не оформлена; офлайн-проверка стека на записи:

```sh
bash ~/robot/scripts/nav2-replay.sh    # bag → Nav2 → /cmd_vel в файл
```

## Конфигурация и скрипты

| Файл | Назначение |
|------|-----------|
| `crates/player-bridge/` | протокол Player (XDR) + `robotctl`; без ROS |
| `crates/player_to_ros2/` | ROS 2-узел: `/scan`, `/odom`, `/cmd_vel`, `/tf` |
| `crates/mask-owner/` | владелец маски (`mask-owner`) и `robot-supervisor` |
| `config/slam_offline.yaml` | slam_toolbox (режим mapping, фреймы) |
| `config/slam_online.yaml` | slam_toolbox для езды с картографией (`--slam`) |
| `config/amcl_replay.yaml` | AMCL + map_server для локализации |
| `config/nav2_replay.yaml` | полный стек Nav2 (costmaps, контроллер RPP, сглаживатель скоростей, планировщик) |
| `scripts/run-rviz.sh` | сессия `robot run`: мост + RViz + телепульт |
| `scripts/run-sim.sh` | плейграунд `robot run --sim`: Gazebo + Nav2 |
| `models/roborock_v1/` | SDF/URDF-модель пылесоса для плейграунда (frames как у моста) |
| `config/nav2_drive.yaml` | оверлей «Nav2 едет»: цепочка до `/cmd_vel` и монитор столкновений (симуляция и живой робот) |
| `config/nav2_sim.yaml` | оверлей Nav2 для симуляции (точка спавна AMCL) |
| `config/rviz_teleop.rviz` | RViz-сцена для `robot run` (лидар, одометрия, TF) |
| `scripts/slam-from-bag.sh` | bag → карта |
| `scripts/slam-offline.sh` | bag → SLAM с проверками по стадиям |
| `scripts/nav2-replay.sh` | bag → Nav2 → `/cmd_vel` в файл (без робота) |
| `scripts/sim-route.sh` | проверка Nav2 в симуляции: маршрут из 4 целей, вердикт |
| `ros2/route_check.py` | клиент маршрута (NavigateToPose, отчёт, снимки карт) |
| `scripts/amcl-replay.sh` | локализация на карте против bag'а |
| `scripts/stamp_bag.py` | проставить нулевые метки времени в старых bag'ах |
| `ros2/odom_tf_replay.py` | odom → TF для записей без `/tf` |
| `docs/player-platform-survey.md` | разбор платформы Player, протокола, железа |
| `LESSWRONG.md` | журнал: что не сработало и почему |
| `PLAN.md` | этапы и критерии перехода |

Ключевые параметры моста (`--ros-args -p`, значения по умолчанию):

```sh
ros2 run player_to_ros2 player_to_ros2 --ros-args \
  -p player_host:=192.0.2.82 \
  -p player_port:=6665 \
  -p laser_frame:=laser \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p laser_x:=-0.100 -p laser_y:=0.0 -p laser_yaw_deg:=97.6 \
  -p cmd_vel_timeout_s:=1.0   # срок одной команды /cmd_vel (0.05..5 с)
```

Лимиты скорости (0.1 м/с, 0.4 рад/с) зашиты в мост
(`crates/player-bridge/src/drive.rs`) и параметром не меняются.

## Разработка и тесты

```sh
# Rust: юнит- и интеграционные тесты (без робота)
cd crates && cargo test

# shell-тесты CLI и обвязки; используют локальные фейки ssh/Player
bash tests/test_robot.sh
bash tests/test_robot_camera.sh
python3 tests/test_robot_bug.py
bash tests/test_robot_owner.sh
bash tests/test_robot_run.sh
bash tests/test_robot_sim.sh
bash tests/test_robot_route.sh

# проверка Nav2 в headless-симуляции (Gazebo, ~3 мин на режим): маршрут
# из 4 целей; код 0 — все цели достигнуты, снимки карт в /tmp/robot-route
bash scripts/sim-route.sh            # готовая карта + AMCL
bash scripts/sim-route.sh --slam     # онлайн-SLAM

# линтер
shellcheck bin/robot bin/camera/snapshot tests/*.sh scripts/*.sh tools/*.sh
```

Правила проекта (TDD, ограничения на живое железо) — в `CLAUDE.md`.

## Безопасность

* **Не отправляйте движение, не снимайте контроллер и не меняйте робота без
  явной необходимости.** Для рутинной проверки используйте локальные фейки.
* Робот ездит **не быстрее 0.1 м/с / 0.4 рад/с** и не дольше 2 с за команду.
  Мост сам ограничивает этими лимитами любой `/cmd_vel` и держит команду не
  дольше 1 с; Nav2 дополнительно ограничен 0.06 м/с / 0.3 рад/с.
* `robot run --slam` — автономная езда по клику. Монитор столкновений
  видит только то, что видит LDS: низкие препятствия, ступеньки и обрывы
  он не замечает. Первые поездки — в свободной зоне и под присмотром.
* `robot move` дополнительно взводит на роботе 10-секундный guard: если
  команда не завершилась штатно, Player принудительно перезапускается.
  **Это не гарантия мгновенной остановки** — держите робота в свободной зоне
  и будьте готовы обесточить.
* Обе щётки выключены только в режиме `mask hold 0x10f8ff`. Любой сбой без
  подтверждения (`a9`) означает **неизвестное физическое состояние**: нужно
  `robot mask restore` и возврат контроллера.
* `robot bridge` возвращает штатный `RoboController` сам, но **всегда
  проверяйте** `robot status` (должно быть `enabled`) после сессии.
* Камера: HTTP Basic поверх открытого HTTP не защищает ни пароль, ни кадры —
  только доверенная сеть. Пароль держите в `~/.config/robot/camera.netrc`
  (режим 0600), не в репозитории. Домашние изображения и карты не выгружайте
  во внешние сервисы без отдельного решения.
* Порт 6665 не выставляйте за пределы доверенной сети.
