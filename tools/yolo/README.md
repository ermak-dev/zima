# YOLOE-26 adapter

`adapter` is a Python-stdlib launcher on the Linux host. By default it replaces
itself with `ssh gpu`, connecting stdin/stdout directly to
`~/robot-yolo/venv/bin/python ~/robot-yolo/worker.py --stdin`. It never connects to
the robot or camera, reads camera credentials, installs packages, or downloads
models. The caller supplies the original JPEG bytes as base64.

`worker.py` imports Pillow, Torch and Ultralytics only when needed. Initialization
checks prepared local model and text-encoder files, loads `YOLOE`, moves it to the
configured device, calls `get_text_pe(classes)` and
`set_classes(classes, embeddings)`. Prediction uses `Results.orig_shape`,
`boxes.xyxy`, `boxes.conf`, `boxes.cls` and `Results.names`. Coordinates refer to
the original JPEG pixel grid, not the inference letterbox or EXIF orientation.
Masks are computed by the segmentation model but are not returned.

## Panel integration

From the repository root, `bin/zima run` enables both
`tools/mrpt-adapter/adapter` and `tools/yolo/adapter` by default. It is equivalent
to specifying `--slam-adapter tools/mrpt-adapter/adapter --yolo-adapter tools/yolo/adapter`.
The same defaults apply to the `panel` and `bridge` aliases. This hands over the
robot controller; it is not a passive camera viewer.

Use `--no-yolo` to keep MRPT but use the original raw camera stream, or
`--no-slam --no-yolo` to disable both adapters. `ROBOT_YOLO_ADAPTER` overrides
the repository-relative executable path; an explicit `--yolo-adapter EXE`
takes precedence. A missing/non-executable adapter is rejected before robot SSH.
These defaults do not prepare weights or install GPU dependencies: complete the
explicit setup below first. Direct low-level `robot-server` invocation still
requires its `--yolo-adapter` flag.

## Offline tests / standalone fake smoke

From the repository root, with only Python 3 installed:

```sh
python3 tests/test_yolo_adapter.py
python3 tests/test_yolo_adapter.py YoloAdapterTests.test_original_coordinates_names_id_and_cleanup
./tools/yolo/adapter --help
printf '%s\n' 'bad json' | YOLO_LOCAL=1 ./tools/yolo/adapter
```

The first two commands inject fake models and decoded images: no GPU packages,
model downloads, network, robot, or camera. The last command exercises the real
stdio/EOF path without initialization and returns a JSON error. There is no fake
runtime mode that could accidentally be mistaken for live detections.

## Explicit GPU installation

These are **manual deployment commands**, not actions performed by the launcher.
They change only the designated GPU machine. Review Ultralytics' licensing before
installing. Use a trusted SSH alias `gpu`, an NVIDIA driver supporting the CUDA
12.8 wheel runtime, Python 3.10+ with venv support, disk space for Torch/model
assets, and adequate GPU VRAM. No system CUDA toolkit, nvcc, or source build is
needed; pip is restricted to wheels. Driver installation is out of scope.

From the repository root:

```sh
ssh gpu 'mkdir -p ~/robot-yolo'
scp tools/yolo/worker.py tools/yolo/requirements.txt tools/yolo/setup_gpu.sh gpu:robot-yolo/
ssh gpu 'cd ~/robot-yolo && bash setup_gpu.sh --confirm'
printf '%s\n' '{"op":"init"}' | ./tools/yolo/adapter
```

`setup_gpu.sh` refuses to run without `--confirm`. It creates `venv`, installs
Torch **2.10.0**, torchvision **0.25.0**, Ultralytics **8.4.191** and
clip-anytorch **2.6.0**, explicitly downloads `yoloe-26l-seg.pt` and
`mobileclip2_b.ts` from Ultralytics assets release `v8.4.0`, then checks the same
runtime init path. clip-anytorch supplies the compatible `clip` tokenizer as a
wheel, avoiding Ultralytics' automatic Git dependency install. Transitive
packages are not lockfile-pinned. `YOLO_TORCH_INDEX` may explicitly select a
compatible alternative wheel index at setup (for example the CPU index with
`YOLO_DEVICE=cpu`); it is not forwarded by the launcher.

Missing runtime assets fail with `ok:false`, not an implicit download. The
worker disables Ultralytics auto-install/offline connectivity and replaces the
text-encoder asset resolver during initialization with a local-file-only
resolver. Only use trusted checkpoints: PyTorch `.pt` loading is not a sandbox.
For a hard network isolation guarantee, restrict GPU process egress externally;
the adapter is not a general-purpose sandbox for third-party Python packages.

A standalone test with an existing local JPEG, after preparation:

```sh
python3 - /path/to/test.jpg <<'PY' | ./tools/yolo/adapter
import base64, json, pathlib, sys
print(json.dumps({"op": "init"}))
print(json.dumps({"op": "detect", "frame_id": 1,
                  "jpeg": base64.b64encode(pathlib.Path(sys.argv[1]).read_bytes()).decode()}))
PY
```

## Protocol

One strict UTF-8 JSON object per input line, one compact JSON response per output
line. Stdout is exclusively protocol; Python and native-library logs go to
stderr. Requests are processed serially, flushed immediately, with no internal
frame queue or retry. EOF releases the model and exits; no child worker or reader
thread is launched. Closing SSH does not guarantee immediate cancellation of an
in-flight GPU kernel; it can finish before the worker observes EOF.

Initialize once (repeating init is idempotent):

```json
{"op":"init"}
{"ok":true,"model":"yoloe-26l-seg.pt"}
```

Detection requires successful init. `frame_id` must be an unsigned 64-bit integer
(not a boolean or float), and is echoed unchanged:

```json
{"op":"detect","frame_id":42,"jpeg":"<base64 original JPEG>"}
{"ok":true,"frame_id":42,"width":640,"height":480,"objects":[{"label":"chair","confidence":0.9,"box":[0.1,0.2,0.7,0.9]}],"inference_ms":37.2}
```

Empty detections return `objects:[]`. Boxes are `[x1,y1,x2,y2]`, normalized and
clamped to `[0,1]`. Labels come from the model's class-name mapping. Confidence
and coordinates must be finite. `inference_ms` measures the prediction call,
not SSH transfer/JPEG decoding. Errors are `{"ok":false,"error":"..."}`; an error
has no required `frame_id`. Invalid lines are rejected without terminating the
worker; oversized records are drained in bounded chunks.

Limits:

* Decoded JPEG at most **2 MiB**; input line at most **3 MiB**, including newline.
* JPEG only; each side at most **8192** pixels and total at most **16,777,216**
  pixels. Dimensions are checked before decoding pixel data.
* At most **100** objects, both `max_det` at inference and bounded serialization.
* No JSON `NaN`/`Infinity`, no non-finite output, no implicit EXIF transpose.

The host caller owns the **180 s init** and **30 s per-frame** deadlines and
process termination. The worker does not interrupt Torch calls itself. Actual
GPU startup time, memory needs and detection quality depend on the deployment;
offline tests do not establish those properties or physical obstacle safety.

The installed worker was verified on the project's RTX 3090 on 2026-10-28:
`init` plus two detections of the public Ultralytics `bus.jpg` (810×1080)
returned matching IDs, original dimensions and normalized `person` boxes.
The first prediction call took approximately 573 ms and the warmed second
call 29 ms (5.75 s for the complete SSH invocation including startup).
The actual Rust server and bundled web panel were also exercised against
this GPU worker with a local fake Player/camera. This verifies the integration,
not indoor camera accuracy or a guaranteed end-to-end frame rate. No robot
controller handoff or physical motion was performed during these checks.

## Environment

| Variable | Default / meaning |
| --- | --- |
| `YOLO_SSH_HOST` | `gpu`; SSH destination, BatchMode, no pseudo-terminal |
| `YOLO_REMOTE_DIR` | `robot-yolo`; relative to remote home, `~/...` or absolute; shell-quoted |
| `YOLO_LOCAL` | Set exactly `1` to exec the adjacent worker using this launcher's Python; activate the prepared venv first |
| `YOLO_MODEL` | `yoloe-26l-seg.pt`; existing local file on the worker, relative to its working directory |
| `YOLO_CLASSES` | JSON array of 1–100 distinct nonblank labels, each at most 100 characters |
| `YOLO_CONF` | `0.25`; finite confidence threshold in `[0,1]` |
| `YOLO_DEVICE` | `0`; CUDA GPU index, or an Ultralytics device such as `cpu` |

Only the last four configuration variables are explicitly forwarded over SSH;
no camera URL, netrc or credentials are used. Asset files must be adjacent to the
model; custom checkpoints must remain compatible with YOLOE-26 text prompting
and its `mobileclip2_b.ts` encoder. Changing to another model family requires
reviewing its encoder/API rather than relying on automatic downloads.

Default indoor classes:

```json
["person","chair","table","sofa","bed","cabinet","door","stairs","rug","shoe","cable","bag","bottle","cup","plant","cat","dog"]
```

Example custom prompts (interpreted on the GPU):

```sh
YOLO_CLASSES='["chair","table","power cable"]' YOLO_CONF=0.3 ./tools/yolo/adapter
```

Relative paths are resolved after entering the remote directory. In local mode,
the launcher preserves the caller's working directory. SSH uses normal
host-key verification and configuration; provision trust manually, never disable
verification for deployment.

## API references and licensing

Checked against the pinned Ultralytics source and current docs:

* [YOLOE documentation](https://docs.ultralytics.com/models/yoloe/)
* [YOLOE Python class / text prompts](https://github.com/ultralytics/ultralytics/blob/v8.4.191/ultralytics/models/yolo/model.py)
* [Text encoder implementation](https://github.com/ultralytics/ultralytics/blob/v8.4.191/ultralytics/nn/text_model.py)
* [Results / Boxes API](https://docs.ultralytics.com/reference/engine/results/)
* [YOLOE-26 config](https://github.com/ultralytics/ultralytics/blob/v8.4.191/ultralytics/cfg/models/26/yoloe-26-seg.yaml)

Ultralytics code/models are offered under **AGPL-3.0** or an appropriate
commercial Enterprise license; deployment, distribution and network-service use
must meet the applicable terms. This wrapper does not relicense those assets or
remove their obligations. Review [Ultralytics licensing](https://www.ultralytics.com/license)
and the licenses of Torch/torchvision, MobileCLIP and clip-anytorch separately.
This is not legal advice.
