#!/usr/bin/env python3
"""FEATURE: standalone speaker helper with independent mixer restoration.
SPEC: compile native test helper and fake ALSA; never touch a sound device.
"""
import io
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import wave

ROOT=Path(__file__).resolve().parents[1]
FAKE=r'''
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
static const char *names[]={"speaker volume control","Audio speaker out","MIC1 boost AMP gain control"};
static long values[]={0,0,4};
static void logset(int id,long value){FILE *f=fopen(getenv("ROBOT_AUDIO_TEST_LOG"),"a");if(f){fprintf(f,"SET %s %ld\n",names[id],value);fclose(f);}}
int snd_pcm_open(void **p,const char*n,int stream,int mode){if(getenv("ROBOT_AUDIO_TEST_BUSY"))return -16;*p=(void*)1;return 0;}
int snd_pcm_close(void*p){return 0;}
int snd_pcm_set_params(void*p,int f,int a,unsigned c,unsigned r,int s,unsigned l){values[2]=5;return 0;}
long snd_pcm_writei(void*p,const void*b,unsigned long n){struct timespec t={0,20000000};nanosleep(&t,0);return n;}
int snd_pcm_state(void*p){return 3;}
int snd_pcm_start(void*p){return 0;}
int snd_pcm_nonblock(void*p,int n){return 0;}
int snd_pcm_drain(void*p){return 0;}
int snd_pcm_drop(void*p){return 0;}
const char *snd_strerror(int e){return "fake ALSA error";}
int snd_mixer_open(void**p,int m){*p=(void*)1;return 0;}
int snd_mixer_attach(void*p,const char*n){return 0;}
int snd_mixer_selem_register(void*p,void*a,void*b){return 0;}
int snd_mixer_load(void*p){return 0;}
int snd_mixer_close(void*p){return 0;}
void *snd_mixer_first_elem(void*p){return (void*)1;}
void *snd_mixer_elem_next(void*p){long n=(long)p;return n<3?(void*)(n+1):0;}
const char *snd_mixer_selem_get_name(void*p){return names[(long)p-1];}
int snd_mixer_selem_has_playback_volume(void*p){return (long)p!=2;}
int snd_mixer_selem_has_capture_volume(void*p){return 0;}
int snd_mixer_selem_has_playback_switch(void*p){return (long)p==2;}
int snd_mixer_selem_has_capture_switch(void*p){return 0;}
int snd_mixer_selem_is_enumerated(void*p){return 0;}
int snd_mixer_selem_get_playback_volume(void*p,int ch,long*v){if(ch)return -1;*v=values[(long)p-1];return 0;}
int snd_mixer_selem_get_capture_volume(void*p,int ch,long*v){return -1;}
int snd_mixer_selem_get_playback_switch(void*p,int ch,int*v){if(ch)return -1;*v=values[(long)p-1];return 0;}
int snd_mixer_selem_get_capture_switch(void*p,int ch,int*v){return -1;}
int snd_mixer_selem_get_enum_item(void*p,int ch,unsigned*v){return -1;}
int snd_mixer_selem_set_playback_volume(void*p,int ch,long v){values[(long)p-1]=v;logset((long)p-1,v);return 0;}
int snd_mixer_selem_set_capture_volume(void*p,int ch,long v){return 0;}
int snd_mixer_selem_set_playback_switch(void*p,int ch,int v){values[(long)p-1]=v;logset((long)p-1,v);return 0;}
int snd_mixer_selem_set_capture_switch(void*p,int ch,int v){return 0;}
int snd_mixer_selem_set_enum_item(void*p,int ch,unsigned v){return 0;}
int snd_mixer_selem_get_playback_volume_range(void*p,long*lo,long*hi){*lo=0;*hi=54;return 0;}
'''


def wav_data(seconds=.2):
    b=io.BytesIO()
    with wave.open(b,'wb') as f:
        f.setnchannels(1); f.setsampwidth(2); f.setframerate(16000);f.writeframes(b'\0\0'*int(16000*seconds))
    return b.getvalue()


class NativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(); cls.root=Path(cls.tmp.name)
        cls.bin=cls.root/'robot-audio'; cls.lib=cls.root/'fake-alsa.so'
        subprocess.run(['bash',str(ROOT/'bin/audio/build.sh'),'--native','--testing','--output',str(cls.bin)],check=True,capture_output=True)
        source=cls.root/'fake.c'; source.write_text(FAKE)
        subprocess.run(['cc','-shared','-fPIC',str(source),'-o',str(cls.lib)],check=True,capture_output=True)
    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()
    def setUp(self):
        self.log=self.root/(self.id().split('.')[-1]+'.log')
        self.env={**os.environ,'ROBOT_AUDIO_ALSA_LIBRARY':str(self.lib),'ROBOT_AUDIO_TEST_LOG':str(self.log),'ROBOT_AUDIO_LOCK':str(self.root/'lock')}
    def run_helper(self,data,args=('play','0.35')):
        return subprocess.run([str(self.bin),*args],input=data,env=self.env,capture_output=True,timeout=6)
    def assert_restored(self):
        log=self.log.read_text().splitlines()
        for name,val in [('speaker volume control',0),('Audio speaker out',0),('MIC1 boost AMP gain control',4)]:
            matching=[line for line in log if line.startswith('SET '+name+' ')]
            self.assertEqual(matching[-1],f'SET {name} {val}')
    def test_completion_and_all_controls_restored(self):
        result=self.run_helper(wav_data())
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn(b'STARTED\n',result.stdout);self.assertIn(b'COMPLETED\n',result.stdout)
        self.assert_restored()
    def test_rejects_invalid_audio_and_volume_before_mixer(self):
        for data,args in [(b'bad',('play','0.35')),(wav_data(),('play','nan')),(wav_data(),('play','1.01')),(wav_data(),('play','0.5;true'))]:
            result=self.run_helper(data,args)
            self.assertNotEqual(result.returncode,0)
            self.assertNotIn(b'STARTED',result.stdout)
        self.assertFalse(self.log.exists())
    def test_busy_vendor_pcm_does_not_write_or_restore_mixer_controls(self):
        self.env['ROBOT_AUDIO_TEST_BUSY'] = '1'
        result = self.run_helper(wav_data())
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'BUSY', result.stdout)
        self.assertNotIn(b'STARTED', result.stdout)
        self.assertFalse(self.log.exists(), 'busy device must not overwrite the current owner mixer')

    def test_worker_sigkill_restores_mixer(self):
        p=subprocess.Popen([str(self.bin),'play','0.35'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=self.env)
        self.addCleanup(lambda: p.kill() if p.poll() is None else None)
        p.stdin.write(wav_data(2));p.stdin.close()
        worker=None
        for _ in range(5):
            line=p.stdout.readline()
            if line.startswith(b'WORKER '): worker=int(line.split()[1])
            if line==b'STARTED\n': break
        self.assertIsNotNone(worker)
        os.kill(worker,signal.SIGKILL)
        p.wait(timeout=5)
        self.assertNotEqual(p.returncode,0);self.assert_restored()
        p.stdout.close();p.stderr.close()
    def test_cancel_command_waits_for_restoration(self):
        token='a'*32
        p=subprocess.Popen([str(self.bin),'play','0.35',token],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=self.env)
        self.addCleanup(lambda: p.kill() if p.poll() is None else None)
        p.stdin.write(wav_data(2));p.stdin.close()
        while p.stdout.readline()!=b'STARTED\n':
            self.assertIsNone(p.poll())
        wrong=self.run_helper(b'',('cancel','b'*32))
        self.assertNotEqual(wrong.returncode,0)
        cancelled=self.run_helper(b'',('cancel',token))
        self.assertEqual(cancelled.returncode,0,cancelled.stderr)
        self.assertIn(b'CANCELLED',cancelled.stdout)
        p.wait(timeout=5);self.assert_restored()
        p.stdout.close();p.stderr.close()

    def test_parent_cancel_restores_mixer(self):
        p=subprocess.Popen([str(self.bin),'play','0.35'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=self.env)
        self.addCleanup(lambda: p.kill() if p.poll() is None else None)
        p.stdin.write(wav_data(2));p.stdin.close()
        while p.stdout.readline()!=b'STARTED\n':
            self.assertIsNone(p.poll())
        p.terminate();p.wait(timeout=5)
        self.assert_restored();self.assertIn(b'CANCELLED',p.stdout.read())
        p.stdout.close();p.stderr.close()


if __name__=='__main__': unittest.main()
