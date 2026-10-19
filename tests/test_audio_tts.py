#!/usr/bin/env python3
"""FEATURE: OpenRouter audio output, local fake SSE, never paid requests.
SPEC: documented streaming /chat/completions, bounded audio and no leaked key/text.
"""
import base64
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
import urllib.error
import wave

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))
from robot_audio.playback import AudioError
from robot_audio.tts import OpenRouterTTS


def wav_data():
    b=io.BytesIO()
    with wave.open(b,'wb') as f:
        f.setnchannels(1); f.setsampwidth(2); f.setframerate(16000); f.writeframes(b'\0\0'*1600)
    return b.getvalue()


class Response(io.BytesIO):
    status=200
    def __enter__(self): return self
    def __exit__(self,*args): self.close()


def sse(data):
    encoded=base64.b64encode(data).decode(); split=len(encoded)//2
    return b''.join(('data: '+json.dumps({'choices':[{'delta':{'audio':{'data':piece}}}]})+'\n\n').encode() for piece in (encoded[:split],encoded[split:]))+b'data: [DONE]\n\n'


class TTSTests(unittest.TestCase):
    def setUp(self): self.requests=[]; self.body=sse(wav_data())
    def open(self, request, timeout):
        self.requests.append(request); return Response(self.body)
    def client(self,**kw): return OpenRouterTTS(model='provider/audio-model',voice='test-voice',api_key='private-test-token',opener=self.open,**kw)

    def test_streaming_documented_payload_and_decode(self):
        data=self.client().synthesize('Тестовая фраза')
        self.assertEqual(data,wav_data())
        request=self.requests[0]; body=json.loads(request.data)
        self.assertEqual(request.full_url,'https://openrouter.ai/api/v1/chat/completions')
        self.assertTrue(body['stream']); self.assertEqual(body['modalities'],['text','audio'])
        self.assertEqual(body['audio'],{'voice':'test-voice','format':'wav'})
        self.assertEqual(request.get_header('Authorization'),'Bearer private-test-token')
        self.assertEqual(body['messages'][-1]['role'],'user')
        job=json.loads(body['messages'][-1]['content'])
        self.assertEqual(job,{'task':'read_verbatim','text':'Тестовая фраза'})
        self.assertEqual(body['messages'][0]['role'],'system')
        self.assertIn('verbatim',body['messages'][0]['content'])

    def test_provider_wav_is_normalized_to_device_format(self):
        raw=io.BytesIO()
        with wave.open(raw,'wb') as w:
            w.setnchannels(2); w.setsampwidth(2); w.setframerate(24000); w.writeframes(b'\0\0'*2400*2)
        self.body=sse(raw.getvalue())
        result=self.client().synthesize('hello')
        with wave.open(io.BytesIO(result),'rb') as w:
            self.assertEqual((w.getframerate(),w.getnchannels(),w.getsampwidth()),(16000,1,2))
            self.assertEqual(w.getnframes(),1600)

    def test_unconfigured_and_invalid_text_fail_before_request(self):
        for client in (OpenRouterTTS(opener=self.open),OpenRouterTTS(model='x',voice='x',opener=self.open)):
            with self.assertRaises(AudioError) as cm: client.synthesize('hello')
            self.assertEqual(cm.exception.code,'configuration_required')
        for text in ('','x'*1001):
            with self.assertRaises(AudioError): self.client().synthesize(text)
        self.assertEqual(self.requests,[])

    def test_cancelled_before_request_and_during_stream(self):
        cancel=threading.Event(); cancel.set()
        with self.assertRaises(AudioError) as cm: self.client().synthesize('hello',cancel)
        self.assertEqual(cm.exception.code,'cancelled'); self.assertEqual(self.requests,[])
        cancel.clear()
        def opening(req,timeout): cancel.set(); return Response(self.body)
        with self.assertRaises(AudioError): OpenRouterTTS(model='x',voice='x',api_key='k',opener=opening).synthesize('hello',cancel)

    def test_bounds_corruption_and_missing_done(self):
        for body in (b'data: [DONE]\n\n',b'data: {invalid}\n\n',sse(b'not wav'),self.body.replace(b'data: [DONE]\n\n',b'')):
            self.body=body
            with self.assertRaises(AudioError): self.client().synthesize('hello')
        self.body=sse(wav_data())
        with self.assertRaises(AudioError): self.client(max_audio_bytes=10).synthesize('hello')

    def test_http_errors_redacted_and_no_retry(self):
        for status,code in ((401,'auth_failed'),(429,'rate_limited'),(503,'provider_unavailable')):
            calls=[]
            def opening(req,timeout):
                calls.append(req); raise urllib.error.HTTPError(req.full_url,status,'private-test-token text secret',{},None)
            client=OpenRouterTTS(model='x',voice='x',api_key='private-test-token',opener=opening)
            with self.assertRaises(AudioError) as cm: client.synthesize('secret speech')
            self.assertEqual(cm.exception.code,code)
            self.assertNotIn('private-test-token',str(cm.exception)); self.assertNotIn('secret speech',str(cm.exception))
            self.assertEqual(len(calls),1)

    def test_output_tokens_are_bounded_and_usage_is_reported(self):
        usage={'prompt_tokens':40,'completion_tokens':100,'total_tokens':140,'cost':.0003}
        self.body=self.body.replace(b'data: [DONE]',('data: '+json.dumps({'choices':[], 'usage':usage})+'\n\ndata: [DONE]').encode())
        client=self.client(max_output_tokens=128)
        self.assertEqual(client.synthesize('hello'),wav_data())
        self.assertEqual(json.loads(self.requests[-1].data)['max_tokens'],128)
        self.assertEqual(client.last_usage,usage)
        for value in (0,True,5000):
            with self.assertRaises(ValueError): self.client(max_output_tokens=value)

    def test_private_dotenv_loads_only_openrouter_without_executing_shell(self):
        import os
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'.env'; marker=Path(d)/'must-not-exist'
            path.write_text('GITLAB_PASSWORD=not-for-audio\nOTHER=$(touch '+str(marker)+')\nOPENROUTER_API_KEY="private-env-token"\n')
            path.chmod(0o600)
            with patch.dict(os.environ,{'OPENROUTER_API_KEY':''}):
                client=OpenRouterTTS(model='x',voice='x',env_file=path,opener=self.open)
                self.assertEqual(client.synthesize('hello'),wav_data())
            self.assertEqual(self.requests[-1].get_header('Authorization'),'Bearer private-env-token')
            self.assertFalse(marker.exists())
            path.chmod(0o644)
            with self.assertRaises(AudioError):
                OpenRouterTTS(model='x',voice='x',env_file=path,opener=self.open).synthesize('hello')

    def test_streaming_pcm16_is_wrapped_and_resampled(self):
        self.body=sse(b'\x00\x00'*2400)
        data=self.client(audio_format='pcm16',pcm_sample_rate=24000).synthesize('hello')
        self.assertEqual(json.loads(self.requests[-1].data)['audio']['format'],'pcm16')
        with wave.open(io.BytesIO(data),'rb') as reader:
            self.assertEqual((reader.getframerate(),reader.getnchannels(),reader.getnframes()),(16000,1,1600))
        self.body=sse(b'odd')
        with self.assertRaises(AudioError): self.client(audio_format='pcm16').synthesize('hello')

    def test_verbatim_mode_rejects_conversation_or_missing_transcript(self):
        def transcript(text):
            return ('data: '+json.dumps({'choices':[{'delta':{'audio':{'transcript':text}}}]})+'\n\n').encode()
        self.body=transcript('Я вас слышу')+sse(wav_data())
        with self.assertRaises(AudioError) as error:
            self.client(require_verbatim=True).synthesize('Проверка звука')
        self.assertEqual(error.exception.code,'tts_text_mismatch')
        self.body=transcript('Проверка звука.')+sse(wav_data())
        client=self.client(require_verbatim=True)
        self.assertEqual(client.synthesize('Проверка звука'),wav_data())
        self.assertEqual(client.last_transcript,'Проверка звука.')
        self.body=sse(wav_data())
        with self.assertRaises(AudioError) as error:
            self.client(require_verbatim=True).synthesize('Проверка звука')
        self.assertEqual(error.exception.code,'tts_transcript_missing')

    def test_token_truncation_is_never_reported_as_complete_audio(self):
        finish=('data: '+json.dumps({'choices':[{'delta':{},'finish_reason':'length'}]})+'\n\n').encode()
        self.body=self.body.replace(b'data: [DONE]',finish+b'data: [DONE]')
        with self.assertRaises(AudioError) as error:self.client().synthesize('hello')
        self.assertEqual(error.exception.code,'tts_output_truncated')

    def test_previous_usage_is_cleared_even_when_new_request_is_cancelled(self):
        client=self.client();client.last_usage={'cost':1};client.last_transcript='previous'
        cancel=threading.Event();cancel.set()
        with self.assertRaises(AudioError):client.synthesize('hello',cancel)
        self.assertEqual(client.last_usage,{})
        self.assertEqual(client.last_transcript,'')

    def test_private_token_file(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'token'; p.write_text('private-test-token\n'); p.chmod(0o600)
            client=OpenRouterTTS(model='x',voice='x',token_file=p,opener=self.open)
            self.assertEqual(client.synthesize('hello'),wav_data())
            p.chmod(0o644)
            with self.assertRaises(AudioError): OpenRouterTTS(model='x',voice='x',token_file=p,opener=self.open).synthesize('hello')


if __name__ == '__main__': unittest.main()
