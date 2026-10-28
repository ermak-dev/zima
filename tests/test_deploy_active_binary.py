#!/usr/bin/env python3
# FEATURE: Persistent daemon deployment must not replace an ELF used by a live process.
# SPEC: Unchanged files can be reused; updates require no live executable inode.
# IMPLEMENTATION: remote inode checks precede atomic replacement of changed files.
# RESULT: IMPLEMENTED — running helper is preserved; stopped helper can be updated.
# STATUS: DONE
import hashlib
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

class DeployTests(unittest.TestCase):
    def test_active_binary_is_not_replaced_but_stopped_binary_can_be_updated(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp)
            target=directory/'supervisor'
            source=directory/'new-helper'
            shutil.copy2('/bin/sleep',target)
            source.write_bytes(b'updated-helper')
            original=hashlib.sha256(target.read_bytes()).digest()
            text=(ROOT/'bin/robot').read_text()
            function=text[text.index('deploy_file() {'):text.index('controller_status() {')]
            harness='''set -euo pipefail
fail() { printf '%s\\n' "$*" >&2; exit 2; }
ssh_host=fake
ssh_wrap() {
    shift
    if (( $# == 1 )); then bash -c "$1"; else "$@"; fi
}
'''+function+'\ndeploy_file "$1" "$2"\n'
            def deploy():
                return subprocess.run(['bash','-c',harness,'test',str(source),str(target)],capture_output=True,text=True,timeout=10)
            process=subprocess.Popen([str(target),'30'])
            try:
                result=deploy()
                self.assertNotEqual(result.returncode,0,'live persistent daemon ELF must not be replaced')
                self.assertEqual(hashlib.sha256(target.read_bytes()).digest(),original)
                self.assertIsNone(process.poll())
            finally:
                process.terminate();process.wait(timeout=3)
            result=deploy()
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(target.read_bytes(),source.read_bytes())

if __name__=='__main__': unittest.main()
