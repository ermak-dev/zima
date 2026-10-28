#!/usr/bin/env bash
# Run explicitly on the GPU machine, never from robot run or the adapter.
set -euo pipefail
if [[ $# != 1 || $1 != --confirm ]]; then
    printf 'Usage: %s --confirm\nInstalls wheels and downloads YOLOE-26 weights/text encoder on THIS machine.\n' "$0" >&2
    exit 2
fi
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
python3 -m venv venv
venv/bin/python -m pip install --only-binary=:all: --upgrade pip
# Bundled CUDA runtime: requires a compatible NVIDIA driver, not system CUDA/nvcc.
# Override the index explicitly for a different driver/CPU deployment.
venv/bin/python -m pip install --only-binary=:all: \
    --index-url "${YOLO_TORCH_INDEX:-https://download.pytorch.org/whl/cu128}" \
    'torch==2.10.0' 'torchvision==0.25.0'
venv/bin/python -m pip install --only-binary=:all: -r requirements.txt
venv/bin/python - <<'PY'
from pathlib import Path
from worker import Config
from ultralytics.utils.downloads import attempt_download_asset

config = Config.from_env()
# Only the setup command is allowed to download. Arbitrary custom models must
# be supplied as existing local files rather than interpreted as download URLs.
if not config.model.is_file():
    if config.model.name not in ('yoloe-26s-seg.pt', 'yoloe-26m-seg.pt', 'yoloe-26l-seg.pt'):
        raise SystemExit('custom YOLO_MODEL must already exist locally')
    downloaded = Path(attempt_download_asset(config.model.name, release='v8.4.0'))
    if downloaded.resolve() != config.model:
        config.model.parent.mkdir(parents=True, exist_ok=True)
        import shutil
        shutil.copyfile(downloaded, config.model)
encoder = config.model.parent / 'mobileclip2_b.ts'
if not encoder.is_file():
    downloaded = Path(attempt_download_asset('mobileclip2_b.ts', release='v8.4.0'))
    if downloaded.resolve() != encoder:
        import shutil
        shutil.copyfile(downloaded, encoder)
config.check_assets()
PY
# Verify text prompting and device placement through the same offline runtime.
printf '%s\n' '{"op":"init"}' | venv/bin/python worker.py --stdin | \
    venv/bin/python -c 'import json,sys; r=json.load(sys.stdin); print(json.dumps(r)); sys.exit(0 if r.get("ok") else 1)'
