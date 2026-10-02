# Robot control

Direct wheel control for the rooted Roborock V1 at `ssh robot`. The CLI uses the robot's Player server (`position2d:0` on TCP 6665), not Valetudo's remote-control mode: on this firmware, that mode switches on the vacuum fan.

## Commands

```sh
robot controller status       # enabled / disabled
robot controller stop         # suspend RoboController supervision
robot position                # fresh Player odometry, metres and radians
robot fan                     # raw Player fan_motor_status sensor (IR index 6)
robot snapshot                # save one camera JPEG under a private /tmp directory
robot snapshot ./frame.jpg    # refuse to replace an existing file
robot move forward 0.5        # bounded velocity for 0.5 s; max 2 s
robot move backward 0.5
robot move left 0.5           # rotate counterclockwise
robot move right 0.5
robot move --with-controller forward 0.5  # supervised test with RoboController still running
robot stop                    # send zero velocity
robot controller start        # restore RoboController supervision
```

Use `~/robot/bin/robot` directly, or keep a `~/bin/robot` symlink pointing at it. All SSH access goes through the `robot` alias in `~/.ssh/config`. The Player client compiles on first use to `${XDG_CACHE_HOME:-~/.cache}/robot/player`. It needs `gcc` and the locally built Player 3.1 library and headers at `~/Projects/robot/ros2/player` (override with `ROBOT_PLAYER_ROOT`). No binaries or libraries are installed on the robot.

`controller stop` saves the original `/opt/rockrobo/watchdog/ProcessList.conf` on the robot, comments out only the `RoboController` entry, restarts `rrwatchdoge` to reload the list, and stops `RoboController`. `controller start` restores that entry and supervision. A failed restart retains a recovery copy and can be retried with `controller start`; inspect manually if it reports an inconsistent config. While disabled, normal buttons, Mi Home and Valetudo control may not work. Always restore the controller when finished.

By default, `robot move` requires `RoboController` to be stopped. `--with-controller` is an explicit, bounded experiment that requires it to be running; the controller may override direct wheel commands or start other motors. The OpenIPC camera serves `http://cam/image.jpg` using HTTP Basic authentication. Create `~/.config/robot/camera.netrc` with `machine cam login USER password PASSWORD`, owned by you with mode `0600`; keep the real password out of the repo and chat. The snapshot command uses this file, never sends Player or motor commands, and validates the JPEG before saving. HTTP Basic over plain HTTP protects neither credentials nor images on the LAN; use it only on a trusted network (or arrange an SSH tunnel). Capture time reported by the command does not prove the camera frame itself is fresh. The RTSP main stream is `rtsp://cam:554/stream=0` for a later continuous reader.

Movement is limited to 0.1 m/s or 0.4 rad/s and 2 seconds per call. The Player client sends zero velocity at the end and on caught signals. Additionally, each move arms an on-robot 10-second guard; if not disarmed on success, it kills the Player process so WatchDoge restarts it. This is a fallback, **not a guarantee of immediate stop** after a network or hardware failure: supervise the robot in a clear area and be ready to power it off. `fan` shows a raw sensor value, not calibrated RPM. A successful CLI exit alone does not prove movement or prove the fan stayed off throughout a run.

## Tests

```sh
bash tests/test_robot.sh
bash tests/test_robot_player.sh
bash tests/test_robot_camera.sh
shellcheck bin/robot bin/camera/snapshot bin/controller/{disable,enable} tests/*.sh
```

The tests use local SSH/Player fakes and do not contact the vacuum or camera. A supervised test on 2026-10-02 moved the robot 0.204 m according to fresh Player odometry; 20 `fan_motor_status` readings overlapping motion were `0.000` (raw sensor, not calibrated RPM). The stock controller and WatchDoge were restored afterward.
