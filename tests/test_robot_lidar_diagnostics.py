#!/usr/bin/env python3
# FEATURE: Read-only vendor lidar firmware capture through the robot CLI.
# SPEC: No controller/serial changes; fresh private output directory; bounded archive;
# only whitelisted regular files extracted as data, never scripts/links or traversal.
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

class LidarDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.archive=self.root/'remote.tar'
        self.log=self.root/'ssh.log'
        self.config=self.root/'robot.toml';self.config.write_text('');self.config.chmod(0o600)
        self.ssh=self.root/'ssh'
        self.ssh.write_text('''#!/usr/bin/env python3
import os,sys
from pathlib import Path
Path(os.environ['TEST_LOG']).write_text(' '.join(sys.argv[1:])+'\\n'+sys.stdin.read())
sys.stdout.buffer.write(Path(os.environ['TEST_ARCHIVE']).read_bytes())
''');self.ssh.chmod(0o700)
        self.env={**os.environ,'ROBOT_CONFIG':str(self.config),'ROBOT_SSH':str(self.ssh),
                  'TEST_LOG':str(self.log),'TEST_ARCHIVE':str(self.archive)}

    def tar(self,members):
        with tarfile.open(self.archive,'w') as archive:
            for name,data,link in members:
                member=tarfile.TarInfo(name);member.mode=0o777
                if link:
                    member.type=tarfile.SYMTYPE;member.linkname=link;archive.addfile(member)
                else:
                    member.size=len(data);archive.addfile(member,io.BytesIO(data))

    def run_cli(self,output):
        return subprocess.run([str(ROOT/'bin/robot'),'diagnose','lidar',str(output)],env=self.env,
                              capture_output=True,text=True,timeout=10)

    def test_captures_only_vendor_data_with_private_permissions(self):
        self.tar([('opt/rockrobo/cleaner/lib/librubylaserdriver.so',b'fake-driver',None),
                  ('opt/rockrobo/cleaner/bin/player',b'fake-player',None)])
        output=self.root/'capture'
        result=self.run_cli(output)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(output.stat().st_mode&0o777,0o700)
        driver=output/'files/opt/rockrobo/cleaner/lib/librubylaserdriver.so'
        self.assertEqual(driver.read_bytes(),b'fake-driver')
        self.assertEqual(driver.stat().st_mode&0o777,0o600)
        self.assertEqual(len(json.loads((output/'manifest.json').read_text())['files']),2)
        script=self.log.read_text()
        self.assertIn('libruby',script)
        self.assertNotIn('/dev/tty',script)
        self.assertNotIn('kill',script)
        self.assertNotIn('ACTIVATE',script)

    def test_existing_output_is_not_modified_and_does_not_contact_robot(self):
        output=self.root/'existing';output.mkdir();(output/'keep').write_text('preserved')
        result=self.run_cli(output)
        self.assertNotEqual(result.returncode,0)
        self.assertEqual((output/'keep').read_text(),'preserved')
        self.assertFalse(self.log.exists())

    def test_untrusted_archives_cannot_escape_or_create_executable_imports(self):
        for index,member in enumerate([('../../escape',b'x',None),
                ('opt/rockrobo/cleaner/lib/librubylaserdriver.so',b'', '/etc/passwd'),
                ('opt/rockrobo/cleaner/lib/json.py',b'raise SystemExit()',None)]):
            with self.subTest(member=member[0]):
                self.tar([member])
                result=self.run_cli(self.root/f'bad-{index}')
                self.assertNotEqual(result.returncode,0)
                self.assertNotIn('Traceback',result.stderr)
                self.assertFalse((self.root/'escape').exists())

if __name__=='__main__':unittest.main()
