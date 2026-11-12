# Third-party notices

This project vendors or builds against the following third-party components.
The project's own code is licensed under the MIT License (see `LICENSE`).

## web/panel.html

`web/panel.html` is the single-file build output of the panel UI in
`tools/panel-ui/` (built with Vite and `vite-plugin-singlefile`). It embeds
minified copies of the following packages, each licensed under the MIT
License by its respective authors:

- React, React DOM, Scheduler, React JSX Runtime —
  Copyright (c) Facebook, Inc. and its affiliates
  (https://reactjs.org/, https://github.com/facebook/react)

## tools/player-modern-gcc.patch

A patch against the upstream Player Project (Player 3.1,
https://github.com/playerproject/player), which is licensed under the
GPL-2.0. The patch is a derivative work of GPL-2.0 code and is distributed
under GPL-2.0 (see https://www.gnu.org/licenses/old-licenses/gpl-2.0.html).
It is applied only to a locally cloned copy of Player used as test
infrastructure and is not linked into any binary built by this project.

## tools/mrpt-adapter

Builds against MRPT (BSD-3-Clause), which is cloned manually into
`tools/mrpt-adapter/third_party/` (not tracked in git; see
`tools/mrpt-adapter/README.md`). `tools/mrpt-adapter/json.h` is original
code of this project.

## tools/yolo (optional)

The optional YOLO object-detection path pins `ultralytics` in
`tools/yolo/requirements.txt`. Ultralytics and the model assets it
downloads (e.g. `yoloe-26l-seg.pt`, `mobileclip2_b.ts`) are licensed
AGPL-3.0 (or a commercial Ultralytics Enterprise license). No Ultralytics
code or model weights are committed to this repository; if you deploy the
YOLO path, review the AGPL-3.0 network-use obligations and
`tools/yolo/README.md`.
