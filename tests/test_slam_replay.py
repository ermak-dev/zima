#!/usr/bin/env python3
# FEATURE: Deterministic offline rolling-scan replay with deliberately imperfect odometry.
# SPEC: No device/network access; generation and geometry metrics reproduce from a seed;
# only trusted local adapter execution, bounded overall time, and exclusive private output files.
# IMPLEMENTATION: synthetic records, constrained replay and pose/wall/admission metrics.
# RESULT: IMPLEMENTED — fakes and real MRPT validate correction, dropout and explicit oracle timing.
# STATUS: DONE
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
TOOL=ROOT/'tools/slam-replay/replay.py'

class ReplayTests(unittest.TestCase):
    def module(self):
        self.assertTrue(TOOL.is_file(),'offline SLAM replay tool is not implemented')
        spec=importlib.util.spec_from_file_location('slam_replay',TOOL)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module

    def test_generation_is_deterministic_and_does_not_hide_odometry_error(self):
        m=self.module()
        header,records=m.generate(steps=12,period=.25,v=.5,w=.4,mode='rolling',odom_scale=.8,seed=7)
        self.assertEqual((header,records),m.generate(steps=12,period=.25,v=.5,w=.4,mode='rolling',odom_scale=.8,seed=7))
        self.assertEqual(len(records),12)
        self.assertEqual(header['format'],'robot-slam-replay-v1')
        self.assertEqual(len(records[0]['request']['scan']['angles']),360)
        self.assertEqual(records[0]['request']['odom'],{'dx':0.,'dy':0.,'dyaw':0.})
        self.assertAlmostEqual(records[1]['request']['odom']['dyaw'],.4*.25*.8)
        self.assertAlmostEqual(records[1]['truth'][2],.4*.25)
        self.assertTrue(all('stamp' in record['request'] for record in records))

    def test_rolling_distortion_and_explicit_oracle_deskew_have_different_inputs(self):
        m=self.module()
        args=dict(steps=3,period=.25,v=0.,w=1.44,odom_scale=.8,seed=3)
        _,instant=m.generate(mode='instant',**args)
        _,rolling=m.generate(mode='rolling',**args)
        _,deskew=m.generate(mode='deskew',**args)
        self.assertNotEqual(instant[0]['request']['scan'],rolling[0]['request']['scan'])
        self.assertNotEqual(rolling[0]['request']['scan'],deskew[0]['request']['scan'])
        self.assertEqual(deskew[0]['timing_model'],'synthetic_verified')
        self.assertTrue(instant[0]['request']['motion_compensated'])
        self.assertTrue(deskew[0]['request']['motion_compensated'])
        self.assertFalse(rolling[0]['request']['motion_compensated'])

    def test_output_records_are_private_and_existing_files_are_not_overwritten(self):
        m=self.module()
        header,records=m.generate(steps=2)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'capture.jsonl'
            m.save(path,header,records)
            self.assertEqual(path.stat().st_mode&0o777,0o600)
            original=path.read_bytes()
            self.assertEqual(m.load(path),(header,records))
            with self.assertRaises(FileExistsError):m.save(path,header,records)
            self.assertEqual(path.read_bytes(),original)
            path.write_text('{"format":"not-a-slam-recording"}\n')
            with self.assertRaises(ValueError):m.load(path)

    def test_fake_adapter_results_measure_pose_error_and_real_acceptance(self):
        m=self.module()
        header,records=m.generate(steps=3,v=.5,w=0.,mode='instant',odom_scale=.8)
        with tempfile.TemporaryDirectory() as tmp:
            adapter=Path(tmp)/'adapter'
            adapter.write_text('''#!/usr/bin/env python3
import json,sys
for line in sys.stdin:
    r=json.loads(line)
    if r['op']=='update':
        print(json.dumps({'ok':True,'pose':{'x':0,'y':0,'yaw':0},'quality':0.4,
            'diagnostics':{'accepted':True,'used_points':360,'mapping_paused':True}}),flush=True)
    elif r['op']=='map':
        print(json.dumps({'ok':True,'grid':{'x_min':0,'y_min':0,'res':0.05,'w':1,'h':1,'cells':'100'}}),flush=True)
    else:print('{"ok":true}',flush=True)
''')
            adapter.chmod(0o700)
            summary=m.run(adapter,header,records)
            self.assertEqual(summary['updates'],3)
            self.assertEqual(summary['accepted_icp'],3)
            self.assertEqual(summary['used_points_min'],360)
            self.assertEqual(summary.get('map_insertions_paused'),3)
            self.assertAlmostEqual(summary['final_xy_error_m'],.25)
            self.assertLess(summary['walls_within_10cm'],1.)
            adapter.write_text('#!/bin/sh\nsleep 3\n')
            with self.assertRaises(TimeoutError):m.run(adapter,header,records,timeout_s=.1)

    def test_malformed_record_shapes_are_rejected_as_data_errors(self):
        m=self.module()
        header,records=m.generate(steps=1)
        for field,value in (('scan','not-an-object'),('scan',None),('odom',None),('odom',17)):
            damaged=json.loads(json.dumps(records))
            damaged[0]['request'][field]=value
            with self.subTest(field=field,value=value),self.assertRaises(ValueError):
                m.save(Path('/not-used'),header,damaged)

    @unittest.skipUnless((ROOT/'tools/mrpt-adapter/adapter').is_file(), 'build the real MRPT adapter first')
    def test_real_adapter_corrects_biased_odometry_and_explicit_oracle_deskew(self):
        m=self.module()
        for options in ({'v':.5,'w':0.,'mode':'instant'},
                        {'v':0.,'w':3.14,'mode':'instant'},
                        {'v':0.,'w':3.14,'mode':'deskew'},
                        {'v':.5,'w':.4,'mode':'instant','drop':.3}):
            with self.subTest(options=options):
                header,records=m.generate(**options,odom_scale=.8,seed=7)
                summary=m.run(ROOT/'tools/mrpt-adapter/adapter',header,records)
                self.assertLess(summary['final_xy_error_m'],.03)
                self.assertLess(summary['final_yaw_error_deg'],2.)
                self.assertGreaterEqual(summary['walls_within_10cm'],.9)
                self.assertGreaterEqual(summary['used_points_min'],200)
                self.assertEqual(summary['accepted_icp'],24)

    @unittest.skipUnless((ROOT/'tools/mrpt-adapter/adapter').is_file(), 'build the real MRPT adapter first')
    def test_real_rolling_scans_pause_keyframes_without_pretending_to_apply_deskew(self):
        m=self.module()
        header,records=m.generate(v=0.,w=1.44,mode='rolling',odom_scale=.8,seed=7)
        self.assertFalse(records[1]['request']['motion_compensated'])
        summary=m.run(ROOT/'tools/mrpt-adapter/adapter',header,records)
        self.assertEqual(summary['map_insertions_paused'],24)
        self.assertGreater(summary['accepted_icp'],0)

    def test_invalid_synthetic_configuration_is_rejected(self):
        m=self.module()
        for options in ({'steps':0},{'steps':10001},{'period':0},{'period':float('nan')},
                        {'mode':'guessed'},{'odom_scale':float('inf')},{'drop':1.1}):
            with self.subTest(options=options),self.assertRaises(ValueError):m.generate(**options)

if __name__=='__main__':unittest.main()
