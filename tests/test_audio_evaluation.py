#!/usr/bin/env python3
"""FEATURE: Local wake-model evaluation with disjoint recordings and speakers.
SPEC: Validation-only threshold choice, honest held-out FA/hour and miss metrics.
"""
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import wave

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))
from robot_audio.evaluation import (calibrate_threshold, evaluate_dataset,
                                    load_dataset, score_detections)


class Scores(unittest.TestCase):
    def test_reports_false_activations_misses_and_latency(self):
        result=score_detections([(0.5,1.0),(3,3.5)], [1.2,2.0], duration_s=5, max_latency_s=.5)
        self.assertEqual(result['true_positives'],1)
        self.assertEqual(result['misses'],1)
        self.assertEqual(result['false_activations'],1)
        self.assertAlmostEqual(result['miss_rate'],.5)
        self.assertAlmostEqual(result['false_activations_per_hour'],720)
        self.assertAlmostEqual(result['latency_p95_s'],.2)

    def test_duplicate_activation_counts_as_false_positive(self):
        result=score_detections([(0,1)], [.9,1.1], duration_s=2)
        self.assertEqual(result['true_positives'],1)
        self.assertEqual(result['false_activations'],1)

    def test_no_positive_data_does_not_claim_zero_miss_rate(self):
        result=score_detections([], [], duration_s=10)
        self.assertIsNone(result['miss_rate'])
        self.assertIsNone(result['latency_p95_s'])
        with self.assertRaises(ValueError): score_detections([],[],duration_s=0)


class DatasetTests(unittest.TestCase):
    def dataset(self, root):
        records=[]
        for i,split in enumerate(('train','validation','test')):
            path=root/f'{split}.wav'
            with wave.open(str(path),'wb') as w:
                w.setparams((1,2,16000,0,'NONE','not compressed'))
                w.writeframes(struct.pack('<h',i+1)*32000)
            records.append(dict(id=split,recording_id=f'r{i}',speaker_id=f's{i}',
                                path=path.name,split=split,condition='quiet',events=[{'start':.4,'end':.8}]))
        data=dict(schema_version=1,phrase='эй робот',language='ru',records=records,
                  criteria=dict(max_false_activations_per_hour=1,max_miss_rate=.1,max_latency_s=.5))
        path=root/'dataset.json';path.write_text(json.dumps(data));return path,data

    def test_loads_local_pcm_dataset_and_rejects_recording_or_speaker_leakage(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);path,data=self.dataset(root)
            dataset=load_dataset(path)
            self.assertEqual(len(dataset['records']),3)
            self.assertEqual(dataset['records'][0]['duration_s'],2)
            data['records'][2]['speaker_id']='s0';path.write_text(json.dumps(data))
            with self.assertRaises(ValueError): load_dataset(path)
            data['records'][2]['speaker_id']='s2';data['records'][2]['recording_id']='r0';path.write_text(json.dumps(data))
            with self.assertRaises(ValueError): load_dataset(path)

    def test_rejects_bad_intervals_and_path_traversal(self):
        with tempfile.TemporaryDirectory() as d:
            path,data=self.dataset(Path(d))
            data['records'][0]['events']=[{'start':1,'end':4}];path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):load_dataset(path)
            data['records'][0]['events']=[];data['records'][0]['path']='../outside.wav';path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):load_dataset(path)

    def test_threshold_calibration_uses_validation_not_test(self):
        with tempfile.TemporaryDirectory() as d:
            path,_=self.dataset(Path(d));calls=[]
            def run(record, threshold):
                calls.append(record['split'])
                return [.8] if threshold>=.5 else [.8,1.4]
            calibrated=calibrate_threshold(path,run,thresholds=[.2,.7])
            self.assertEqual(set(calls),{'validation'})
            self.assertEqual(calibrated['threshold'],.7)
            report=evaluate_dataset(path,run,threshold=.7,split='test')
            self.assertTrue(report['accepted'])
            self.assertEqual(report['records'][0]['id'],'test')
            self.assertEqual(report['split'],'test')

    def test_missing_positive_test_data_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as d:
            path,data=self.dataset(Path(d));data['records'][2]['events']=[];path.write_text(json.dumps(data))
            report=evaluate_dataset(path,lambda record,threshold:[],threshold=.7,split='test')
            self.assertFalse(report['accepted'])


if __name__ == '__main__': unittest.main()
