#!/usr/bin/env python3
"""Read-only capture of whitelisted vendor lidar/Player files for offline analysis."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import resource
import subprocess
import tarfile

MAX_ARCHIVE=64*1024*1024
MAX_DATA=60*1024*1024
REMOTE=r'''set -euo pipefail
files=()
total=0
for file in \
 /opt/rockrobo/cleaner/lib/libruby*laser*.so* \
 /opt/rockrobo/cleaner/lib/libchassis*.so* \
 /opt/rockrobo/cleaner/lib/libplayer*.so* \
 /opt/rockrobo/cleaner/lib/libuart_api.so* \
 /opt/rockrobo/cleaner/lib/librrlog.so* \
 /opt/rockrobo/cleaner/lib/libroboipc.so* \
 /opt/rockrobo/cleaner/bin/player \
 /opt/rockrobo/cleaner/bin/Player \
 /opt/rockrobo/cleaner/conf/*laser*.cfg \
 /opt/rockrobo/cleaner/conf/*chassis*.cfg \
 /opt/rockrobo/cleaner/conf/*player*.cfg \
 /opt/rockrobo/cleaner/config/*laser*.cfg \
 /opt/rockrobo/cleaner/config/*chassis*.cfg \
 /opt/rockrobo/cleaner/config/*player*.cfg \
 /opt/rockrobo/cleaner/etc/*laser*.cfg \
 /opt/rockrobo/cleaner/etc/*chassis*.cfg \
 /opt/rockrobo/cleaner/etc/*player*.cfg
do
    [[ -f $file && ! -L $file ]] || continue
    size=$(stat -c %s -- "$file")
    (( size <= 16777216 )) || exit 1
    total=$((total+size))
    (( total <= 62914560 && ${#files[@]} < 64 )) || exit 1
    files+=("${file#/}")
done
(( ${#files[@]} > 0 )) || exit 1
exec tar -C / -cf - "${files[@]}"
'''


def allowed(name):
    path=PurePosixPath(name)
    if path.is_absolute() or '..' in path.parts or not name.startswith('opt/rockrobo/cleaner/'):
        return False
    parent=str(path.parent);base=path.name
    if parent=='opt/rockrobo/cleaner/bin':return base in ('player','Player')
    if parent=='opt/rockrobo/cleaner/lib':
        return bool(re.fullmatch(r'lib(?:ruby.*laser.*|chassis.*|player.*|uart_api|rrlog|roboipc)\.so(?:\.[A-Za-z0-9._-]+)?',base))
    if parent in ('opt/rockrobo/cleaner/conf','opt/rockrobo/cleaner/config','opt/rockrobo/cleaner/etc'):
        return base.endswith('.cfg') and any(part in base for part in ('laser','chassis','player'))
    return False


def capture(output):
    if output.exists() or output.is_symlink():raise ValueError('output exists')
    host=os.environ.get('ROBOT_SSH_HOST','robot')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:\[\]-]{0,127}',host):raise ValueError('invalid SSH host')
    output.mkdir(mode=0o700)
    os.chmod(output,0o700)
    archive=output/'bundle.tar'
    def limit():resource.setrlimit(resource.RLIMIT_FSIZE,(MAX_ARCHIVE,MAX_ARCHIVE))
    fd=os.open(archive,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'wb') as stream:
        result=subprocess.run([os.environ.get('ROBOT_SSH','ssh'),'-T','-oBatchMode=yes','-oConnectTimeout=5',
            '-oServerAliveInterval=2','-oServerAliveCountMax=2',host,'bash','-s'],input=REMOTE.encode(),
            stdout=stream,stderr=subprocess.PIPE,timeout=60,preexec_fn=limit)
    if result.returncode or archive.stat().st_size>MAX_ARCHIVE:raise ValueError('capture failed')
    files=[];total=0
    with tarfile.open(archive,'r:') as bundle:
        members=[]
        for member in bundle:
            if not member.isfile() or not allowed(member.name) or member.size>16*1024*1024 or member.size<0:
                raise ValueError('invalid captured member')
            total+=member.size
            members.append(member)
            if len(members)>64 or total>MAX_DATA:raise ValueError('captured data too large')
        if not members:raise ValueError('empty vendor capture')
        for member in members:
            target=output/'files'/member.name
            target.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
            digest=hashlib.sha256()
            source=bundle.extractfile(member)
            if source is None:raise ValueError('missing captured bytes')
            fd=os.open(target,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
            count=0
            with source,os.fdopen(fd,'wb') as destination:
                while chunk:=source.read(65536):
                    count+=len(chunk)
                    if count>member.size:raise ValueError('unexpected captured size')
                    digest.update(chunk);destination.write(chunk)
            if count!=member.size:raise ValueError('truncated captured file')
            files.append({'path':member.name,'bytes':count,'sha256':digest.hexdigest()})
    manifest={'format':'robot-vendor-lidar-capture-v1','read_only':True,'files':files}
    fd=os.open(output/'manifest.json',os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'w') as stream:json.dump(manifest,stream,indent=2)
    return {'directory':str(output),'files':len(files),'bytes':total,'read_only':True}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target',choices=['lidar'])
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    os.umask(0o077)
    try:print(json.dumps(capture(args.output)))
    except (OSError,ValueError,tarfile.TarError,subprocess.SubprocessError):
        parser.exit(1,'robot diagnose: capture failed, output exists or unsafe vendor archive; no controller changes requested\n')

if __name__=='__main__':main()
