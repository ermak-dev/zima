#!/usr/bin/env python3
"""Client of the same host-side speech API used by the web panel."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


def main(argv=None):
    parser = argparse.ArgumentParser(prog='robot say', description=__doc__)
    parser.add_argument('text', nargs='?')
    parser.add_argument('--text-file', type=Path)
    parser.add_argument('--volume', type=float)
    parser.add_argument('--endpoint', default=os.environ.get('ROBOT_API_ENDPOINT', 'http://127.0.0.1:8085'))
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args(argv)
    if args.text_file and args.text is not None:
        parser.error('choose text or --text-file, not both')
    try:
        if args.text_file:
            if args.text_file.stat().st_size > 16000:
                raise ValueError('text file is too large')
            args.text = args.text_file.read_text(encoding='utf-8')
        if not args.text or not args.text.strip() or len(args.text) > 2000 or '\x00' in args.text:
            raise ValueError('text must be 1..2000 characters')
        if args.volume is not None and (not math.isfinite(args.volume) or not 0 <= args.volume <= 1):
            raise ValueError('volume must be between zero and one')
        if not math.isfinite(args.timeout) or not 0 < args.timeout <= 600:
            raise ValueError('timeout must be between zero and 600 seconds')
        url = urllib.parse.urlsplit(args.endpoint)
        if url.scheme not in ('http', 'https') or not url.hostname or url.username or url.password or url.query or url.fragment:
            raise ValueError('invalid API endpoint')
        endpoint = args.endpoint.rstrip('/')
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        deadline = time.monotonic() + args.timeout

        def request(path, body=None):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ValueError('timeout; playback outcome is not confirmed; no automatic retry')
            headers = {'Content-Type': 'application/json'} if body is not None else {}
            data = json.dumps(body).encode() if body is not None else None
            req = urllib.request.Request(endpoint + path, data=data, headers=headers)
            with opener.open(req, timeout=min(10, remaining)) as response:
                data = response.read(16385)
                if len(data) > 16384:
                    raise ValueError('invalid API response')
                value = json.loads(data)
                if not isinstance(value, dict):
                    raise ValueError('invalid API response')
                return value

        payload = {'text': args.text.strip()}
        if args.volume is not None:
            payload['volume'] = args.volume
        submitted = request('/api/audio/say', payload)
        task_id = submitted.get('id')
        if not isinstance(task_id, str) or not task_id or len(task_id) > 128:
            raise ValueError('invalid API task receipt; no automatic retry')
        path = '/api/audio/tasks/' + urllib.parse.quote(task_id, safe='')
        while True:
            task = request(path)
            state = task.get('state')
            if state == 'completed':
                print('robot say: completed')
                return 0
            if state in ('failed', 'unknown', 'cancelled'):
                raise ValueError(f'{state}; playback not completed; no automatic retry')
            if state not in ('queued', 'synthesizing', 'playing'):
                raise ValueError('invalid API task state; playback outcome not confirmed')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ValueError('timeout; playback outcome is not confirmed; no automatic retry')
            time.sleep(min(.2, remaining))
    except urllib.error.HTTPError as error:
        print(f'robot say: API HTTP {error.code}; no automatic retry', file=sys.stderr)
    except (urllib.error.URLError, OSError, TimeoutError):
        print('robot say: API unavailable; playback outcome is not confirmed; no automatic retry', file=sys.stderr)
    except (ValueError, UnicodeError) as error:
        print(f'robot say: {error}', file=sys.stderr)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
