"""Explicit atomic installation of the speaker helper; never starts it."""
import hashlib
import os
from pathlib import Path
import stat
import subprocess
import uuid

from .config import validate_config


def deploy(binary, config, *, confirmed=False):
    if not confirmed:
        raise ValueError('deployment requires --confirm; it changes a file on the robot')
    config = validate_config(config)
    try:
        fd = os.open(Path(binary), os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        raise ValueError('deployment requires a regular ARM executable') from None
    with os.fdopen(fd, 'rb') as handle:
        metadata = os.fstat(handle.fileno())
        if not stat.S_ISREG(metadata.st_mode) or not 52 <= metadata.st_size <= 16 * 1024 * 1024:
            raise ValueError('helper is not a bounded regular executable')
        data = handle.read(16 * 1024 * 1024 + 1)
    if data[:7] != b'\x7fELF\x01\x01\x01' or data[18:20] != b'\x28\x00':
        raise ValueError('helper must be a little-endian 32-bit ARM ELF executable')
    digest = hashlib.sha256(data).hexdigest()
    target = config['output']['helper']
    directory = str(Path(target).parent)
    temporary = target + '.upload-' + uuid.uuid4().hex
    command = f'''set -eu
umask 077
test ! -L {directory}
mkdir -p -- {directory}
test ! -L {target}
if test -e {target}; then test -f {target}; sha256sum {target} >/dev/null; fi
trap 'rm -f -- {temporary}' EXIT HUP INT TERM
cat > {temporary}
printf '%s  %s\\n' {digest} {temporary} | sha256sum -c - >/dev/null
chmod 700 {temporary}
mv -f -- {temporary} {target}
trap - EXIT HUP INT TERM
printf 'DEPLOYED\\n'
'''
    result = subprocess.run([os.environ.get('ROBOT_SSH', 'ssh'), '-T', '-oBatchMode=yes',
        '-oConnectTimeout=5', config['output']['host'], command], input=data,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=30)
    if result.returncode or result.stdout.strip() != b'DEPLOYED':
        raise RuntimeError('helper installation failed; no playback was requested')
    return {'deployed': True, 'host': config['output']['host'], 'helper': target, 'sha256': digest}
