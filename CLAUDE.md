# Robot development

This repository controls a physical Roborock. Read `HARDWARE.md` and `SOFTWARE.md` before touching motion or controller code. Do not send live movement, stop the controller, or modify the robot unless the user explicitly asks; use local fakes for routine verification. If a live handoff is authorized, restore the stock controller when finished.

## TDD

For every behavior change or bug fix: write a focused failing test first; run it to confirm the intended failure; make the smallest change; rerun that test and the affected suite. Do not weaken a test to make it pass. Documentation-only changes do not need a new test. Report any test you could not run.

```sh
bash tests/test_robot.sh
bash tests/test_robot_camera.sh
python3 tests/test_robot_bug.py
shellcheck bin/zima bin/camera/snapshot tests/*.sh
```

## Commands

Use `bin/zima` (or an installed `zima` symlink) for robot operations; see `bin/zima --help` and `SOFTWARE.md`. For self-hosted GitLab issue tracking use `bin/zima bug`, never `rug bug`: `project`, `labels`, `list [--all]`, `show <iid>`, `new "title" --body-file <file>`, `comment <iid> "text"`, `edit <iid>`, `relabel <iid>`, `start <iid>`, `mr <iid>`, `close <iid>`, `reopen <iid>`, `delete <iid> --yes`. Use `bin/zima bug <command> --help` for flags. Check existing issues before opening one; keep issue titles, bodies and comments in Russian. If tracker functionality is missing from `zima bug`, extend `bin/bug` and its offline tests rather than bypassing the command or silently substituting another tool. The tracker talks to a private GitLab instance configured via GitLab variables in the ignored, mode-0600 `.env`; it is not part of the public GitHub mirror — use GitHub Issues there. This project is in a user namespace, so `zima bug epic new` creates an issue-эпик with a checklist and `zima bug new --epic <iid>` links tasks; it is not a native GitLab group Epic. Never print or commit credentials; the CLI asks GitLab for the current default branch instead of assuming `master` or `main`.
