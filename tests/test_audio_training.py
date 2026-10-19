#!/usr/bin/env python3
"""FEATURE: Local KWS head training/export with no test-set or stress-label leakage.

SPEC: Explicit local feature models, 80 ms windows, ONNX output, unverified artifacts.
"""
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
import wave

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2'))


@unittest.skipUnless(importlib.util.find_spec('onnx') and importlib.util.find_spec('numpy'),
                     'install optional local training dependencies')
class TrainingTests(unittest.TestCase):
    def setUp(self):
        try:
            from robot_audio.training import label_windows, fit_head, export_head, train_model
        except ImportError:
            self.fail('local KWS training is not implemented')
        self.labels,self.fit,self.export,self.train=label_windows,fit_head,export_head,train_model
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)

    def test_labels_require_the_end_of_the_whole_phrase(self):
        labels=self.labels([.08,.16,.24,.32,.4],[(.0,.24)],lead_s=.04,tail_s=.08)
        self.assertEqual(list(labels),[0,0,1,1,0])
        self.assertEqual(list(self.labels([.08,.16],[])),[0,0])

    def test_onnx_scores_match_the_fitted_head(self):
        import numpy as np
        import onnxruntime as ort
        rng=np.random.default_rng(17)
        x=rng.normal(size=(80,2,96)).astype('float32')
        y=(x[:,0,0]+x[:,1,1]>0).astype('int64')
        for kind in ('logistic','mlp'):
            with self.subTest(kind=kind):
                head=self.fit(x,y,kind=kind,seed=17)
                path=self.root/(kind+'.onnx');self.export(head,path,window_frames=2)
                session=ort.InferenceSession(str(path),providers=['CPUExecutionProvider'])
                actual=session.run(None,{session.get_inputs()[0].name:x[:5]})[0].ravel()
                expected=head.predict_proba(x[:5].reshape(5,-1))[:,1]
                np.testing.assert_allclose(actual,expected,rtol=2e-5,atol=2e-5)
                self.assertEqual(session.get_inputs()[0].shape[1:],[2,96])

    def test_refuses_missing_classes_nonfinite_features_and_overwrite(self):
        import numpy as np
        with self.assertRaises(ValueError):self.fit(np.zeros((4,2,96)),np.zeros(4))
        with self.assertRaises(ValueError):self.fit(np.full((4,2,96),np.nan),np.array([0,1,0,1]))
        x=np.zeros((4,2,96),dtype='float32');x[2:]=1
        head=self.fit(x,np.array([0,0,1,1]))
        path=self.root/'head.onnx';self.export(head,path,window_frames=2)
        with self.assertRaises((FileExistsError,ValueError)):self.export(head,path,window_frames=2)

    def dataset(self,*,bad_stress=False):
        records=[]
        for name,value,split in (('positive',2000,'train'),('negative',-2000,'train'),
                                  ('validation',3000,'validation'),('heldout',4000,'test')):
            path=self.root/(name+'.wav')
            with wave.open(str(path),'wb') as w:
                w.setnchannels(1);w.setsampwidth(2);w.setframerate(16000)
                import struct
                w.writeframes(struct.pack('<h',value)*32000)
            positive=name!='negative'
            records.append({'id':name,'recording_id':name,'speaker_id':name,'path':path.name,
                'split':split,'condition':'synthetic','events':[{'start':.2,'end':1.2}] if positive else [],
                'spoken_pronunciation':'Привет, Зима́' if bad_stress and positive else ('Привет, Зи́ма' if positive else 'Привет, Зима́'),
                'pronunciation_verified':True})
        path=self.root/'dataset.json'
        path.write_text(json.dumps({'schema_version':1,'phrase':'Привет, Зима','language':'ru',
            'pronunciation':'Привет, Зи́ма','negative_pronunciations':['Привет, Зима́'],
            'criteria':{'max_false_activations_per_hour':0,'max_miss_rate':.1,'max_latency_s':.5},'records':records}))
        features=self.root/'features';features.mkdir(exist_ok=True)
        for name in ('melspectrogram.onnx','embedding_model.onnx'):(features/name).write_bytes(name.encode())
        return path,features

    def factory(self):
        import numpy as np
        class Features:
            def __init__(self):self.value=0
            def __call__(self,samples):self.value=float(samples.mean())/32768
            def get_features(self,n):return np.full((1,n,96),self.value,dtype='float32')
        return Features

    def test_training_uses_only_train_speakers_and_never_verifies_itself(self):
        manifest,features=self.dataset()
        report=self.train(manifest,features,self.root/'output',window_frames=2,feature_factory=self.factory())
        self.assertEqual(report['train_records'],['positive','negative'])
        self.assertEqual(set(report['train_speakers']),{'positive','negative'})
        result=json.loads((self.root/'output/manifest.json').read_text())
        self.assertFalse(result['verified'])
        self.assertEqual(result['pronunciation'],'Привет, Зи́ма')
        self.assertNotIn('evaluation_report',result)

    def test_wrong_stress_cannot_be_a_positive_training_example(self):
        manifest,features=self.dataset(bad_stress=True)
        with self.assertRaises(ValueError):
            self.train(manifest,features,self.root/'output',window_frames=2,feature_factory=self.factory())


if __name__=='__main__':unittest.main()
