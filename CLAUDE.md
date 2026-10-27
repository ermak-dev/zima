# Robot development

This repository controls a physical Roborock. Read `HARDWARE.md` and `SOFTWARE.md` before touching motion or controller code. Do not send live movement, stop the controller, or modify the robot unless the user explicitly asks; use local fakes for routine verification. If a live handoff is authorized, restore the stock controller when finished.

## TDD

For every behavior change or bug fix: write a focused failing test first; run it to confirm the intended failure; make the smallest change; rerun that test and the affected suite. Do not weaken a test to make it pass. Documentation-only changes do not need a new test. Report any test you could not run.

```sh
bash tests/test_robot.sh
bash tests/test_robot_camera.sh
python3 tests/test_robot_bug.py
shellcheck bin/robot bin/camera/snapshot tests/*.sh
```

## Commands

Use `bin/robot` (or an installed `robot` symlink) for robot operations; see `bin/robot --help` and `SOFTWARE.md`. For GitLab issues use `bin/robot bug`, never `rug bug`: `project`, `labels`, `list [--all]`, `show <iid>`, `new "title" --body-file <file>`, `comment <iid> "text"`, `edit <iid>`, `relabel <iid>`, `start <iid>`, `mr <iid>`, `close <iid>`, `reopen <iid>`, `delete <iid> --yes`. Use `bin/robot bug <command> --help` for flags. Check existing issues before opening one; keep issue titles, bodies and comments in Russian. If tracker functionality is missing from `robot bug`, extend `bin/bug` and its offline tests rather than bypassing the command or silently substituting another tool. This project is in a user namespace, so `robot bug epic new` creates an issue-эпик with a checklist and `robot bug new --epic <iid>` links tasks; it is not a native GitLab group Epic. The CLI reads only GitLab variables from the ignored, mode-0600 `.env`; never print or commit credentials. It targets `ermak/robot` and asks GitLab for the current default branch instead of assuming `master` or `main`.
