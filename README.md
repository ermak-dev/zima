# Roborock V1

**English summary:** Manual control of a rooted Roborock V1 vacuum robot — live
lidar, sensors and camera in a browser panel, plus a separate Python loop for
recording, speech recognition and the speaker. The stock Player stays the
hardware driver; a Rust server and an optional audio service run on the host.
This is a personal, hardware-specific project: ROS, SLAM-based autonomous
navigation and wake-word are intentionally out of scope. The panel shows a
single live scan relative to the robot body, not a room map.

Ручное управление root-нутым Roborock V1: текущий лидар, датчики и камера
в браузере, отдельный Python-контур записи, распознавания речи и динамика.
Штатный Player остаётся драйвером железа; на хосте работают Rust-сервер
и опциональный аудиосервис.

## Документация / Documentation

* **[HARDWARE.md](HARDWARE.md)** — железо, датчики, питание, аппаратные
  интерфейсы, подтверждённые ограничения и безопасность.
* **[SOFTWARE.md](SOFTWARE.md)** — текущие процессы и модули, протоколы,
  сборка, запуск, аудио, тесты и диагностика.
* **[CLAUDE.md](CLAUDE.md)** — правила работы с репозиторием: TDD,
  разрешения на живое железо и трекер задач.
* **[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md)** — сторонние
  компоненты и их лицензии.

Перед запуском прочитайте разделы безопасности и жизненного цикла сессии
(документация на русском).
`bin/zima run`, `panel` и `bridge` передают управление физическим роботом;
это не пассивный просмотр. Разработка и штатные тесты используют локальные
фейки и не требуют робота.

ROS, SLAM, автономная навигация и wake word в текущую реализацию не входят.
Панель показывает один текущий скан относительно корпуса, а не карту помещения.

## Tests

Offline test suites run on local fakes and do not require the robot:

```sh
bash tests/test_robot.sh
bash tests/test_robot_camera.sh
python3 tests/test_robot_bug.py
shellcheck bin/zima bin/camera/snapshot tests/*.sh
```

## License

The project code is licensed under the [MIT License](LICENSE); third-party
components and their licenses are listed in
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md). The optional YOLO path
(see `tools/yolo/`) pulls in AGPL-3.0 dependencies at setup time.
