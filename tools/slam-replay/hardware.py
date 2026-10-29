#!/usr/bin/env python3
"""Offline raw Player replay. Run with python3 -I; only execute a trusted adapter.

API: load(path, allow_incomplete=False), prepare(capture, lidar_yaw_deg=...),
run(adapter, capture, lidar_yaw_deg=...). No measurement truth is available.
"""
import argparse
import base64
import binascii
from collections import Counter
import json
import math
import os
from pathlib import Path
import selectors
import signal
import struct
import subprocess
import time

MAX_BYTES = 64 * 1024 * 1024
MAX_QUEUE_CAPACITY = 1024
MAX_PAYLOAD = 65536
MAX_LINE = 90000
MAX_FRAMES = 200000
MAX_POINTS = 4096
MAX_REPLY = 4 * 1024 * 1024
MAX_TOTAL_REPLY_BYTES = 64 * 1024 * 1024
RECEIVE_CLOCK = 'monotonic offset from recorder open in nanoseconds'
SESSION_RECEIVE_CLOCK = 'monotonic offset from session start in nanoseconds'
HEADER_KEYS = {'type', 'format', 'receive_clock', 'source_clock_verified',
               'max_file_bytes', 'max_payload_bytes', 'queue_capacity'}
FRAME_KEYS = {'type', 'interface', 'index', 'msg_type', 'subtype', 'source_stamp',
              'source_seq', 'receive_offset_ns', 'payload_base64'}
FOOTER_KEYS = {'type', 'complete', 'frames_written', 'queue_dropped', 'oversized', 'size_limited'}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _uint(value, maximum):
    return type(value) is int and 0 <= value <= maximum


def _unique(pairs):
    result = {}
    for key, value in pairs:
        _require(key not in result, 'duplicate JSON key')
        result[key] = value
    return result


def _json(data):
    try:
        return json.loads(data, object_pairs_hook=_unique,
                          parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
    except (UnicodeError, RecursionError, json.JSONDecodeError) as exc:
        raise ValueError('invalid JSON') from exc


def decode_laser(payload):
    """Decode raw SCANANGLE, retaining logical array order and XDR u_short words."""
    _require(4 <= len(payload) <= MAX_PAYLOAD, 'invalid laser payload size')
    range_max = struct.unpack_from('>f', payload)[0]
    _require(_finite(range_max) and 0 < range_max <= 1000, 'invalid laser range_max')
    offset = 4
    arrays = []
    for code in ('f', 'f', 'I'):
        _require(offset + 8 <= len(payload), 'truncated laser count')
        count, duplicate = struct.unpack_from('>II', payload, offset)
        offset += 8
        _require(count == duplicate and count <= MAX_POINTS, 'invalid laser array count')
        _require(offset + count * 4 <= len(payload), 'truncated laser array')
        values = list(struct.unpack_from('>' + code * count, payload, offset))
        offset += count * 4
        _require(all(_finite(v) for v in values), 'nonfinite laser geometry')
        if code == 'I':
            _require(all(v <= 65535 for v in values), 'invalid XDR unsigned short')
        arrays.append(values)
    ranges, angles, intensities = arrays
    _require(len(ranges) == len(angles) and len(intensities) in (0, len(ranges)),
             'incompatible laser arrays')
    _require(offset + 4 == len(payload), 'truncated or trailing laser ID')
    return dict(range_max=range_max, ranges=ranges, angles=angles,
                intensities=intensities, scan_id=struct.unpack_from('>I', payload, offset)[0])


def _decode_odom(payload):
    _require(len(payload) == 52, 'invalid position2d payload size')
    values = struct.unpack('>6dI', payload)
    _require(all(_finite(v) for v in values[:6]) and values[6] in (0, 1),
             'invalid position2d geometry or stall')
    return dict(zip(('x', 'y', 'yaw', 'vx', 'vy', 'yaw_rate', 'stalled'), values))


def load(path, *, allow_incomplete=False):
    """Strict bounded JSONL parser; missing/lossy completion requires explicit opt-in.

    Even with opt-in, corrupt data, mismatched counts and malformed footers fail.
    Unknown Player channels are retained as opaque, bounded bytes, never commands.
    """
    header = None
    completion = None
    frames = []
    size = 0
    previous_ns = 0
    with Path(path).open('rb') as stream:
        while True:
            line = stream.readline(MAX_LINE + 1)
            if not line:
                break
            size += len(line)
            _require(size <= MAX_BYTES and len(line) <= MAX_LINE, 'capture size limit exceeded')
            _require(line.endswith(b'\n'), 'unterminated capture row')
            row = _json(line)
            _require(isinstance(row, dict), 'capture row must be an object')
            if header is None:
                _require(set(row) == HEADER_KEYS and row.get('type') == 'header' and
                         row.get('format') == 'robot-player-raw-v1', 'invalid capture header')
                _require(row['source_clock_verified'] is False and row['receive_clock'] in (RECEIVE_CLOCK, SESSION_RECEIVE_CLOCK),
                         'unsupported capture clock declaration')
                for key, maximum in (('max_file_bytes', MAX_BYTES), ('max_payload_bytes', MAX_PAYLOAD),
                                     ('queue_capacity', MAX_QUEUE_CAPACITY)):
                    _require(_uint(row[key], maximum) and row[key] > 0, 'invalid capture limit')
                header = row
                continue
            _require(completion is None, 'data after completion')
            if row.get('type') == 'completion':
                _require(set(row) == FOOTER_KEYS and type(row['complete']) is bool,
                         'invalid capture completion')
                for key in FOOTER_KEYS - {'type', 'complete'}:
                    _require(_uint(row[key], 2**64 - 1), 'invalid completion counter')
                _require(row['frames_written'] == len(frames), 'completion frame count mismatch')
                completion = row
                continue
            _require(set(row) == FRAME_KEYS and row.get('type') == 'frame', 'invalid capture frame')
            _require(len(frames) < MAX_FRAMES, 'capture frame limit exceeded')
            for key, maximum in (('interface', 65535), ('index', 65535), ('msg_type', 255),
                                 ('subtype', 255), ('source_seq', 2**32 - 1),
                                 ('receive_offset_ns', 2**64 - 1)):
                _require(_uint(row[key], maximum), 'invalid frame ' + key)
            _require(row['source_stamp'] is None or _finite(row['source_stamp']), 'invalid publication stamp')
            _require(row['receive_offset_ns'] >= previous_ns, 'receive clock regressed')
            previous_ns = row['receive_offset_ns']
            encoded = row['payload_base64']
            _require(isinstance(encoded, str) and len(encoded) <= 4*((MAX_PAYLOAD+2)//3),
                     'invalid base64 size')
            try:
                payload = base64.b64decode(encoded, validate=True)
            except (ValueError, binascii.Error) as exc:
                raise ValueError('invalid payload base64') from exc
            _require(len(payload) <= header['max_payload_bytes'] and
                     base64.b64encode(payload).decode() == encoded, 'invalid or noncanonical base64')
            row['payload'] = payload
            channel = (row['interface'], row['index'], row['msg_type'], row['subtype'])
            if channel == (6, 0, 1, 3):
                row['laser'] = decode_laser(payload)
            elif channel == (4, 0, 1, 1):
                row['odom'] = _decode_odom(payload)
            frames.append(row)
    _require(header is not None, 'empty capture')
    _require(size <= header['max_file_bytes'], 'declared capture size exceeded')
    complete = completion is not None and completion['complete'] and all(
        completion[key] == 0 for key in ('queue_dropped', 'oversized', 'size_limited'))
    _require(complete or allow_incomplete, 'incomplete capture (explicit opt-in required)')
    return dict(header=header, frames=frames, completion=completion, complete=complete)


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _delta(previous, current):
    dx, dy = current['x'] - previous['x'], current['y'] - previous['y']
    c, s = math.cos(previous['yaw']), math.sin(previous['yaw'])
    result = dict(dx=c*dx+s*dy, dy=-s*dx+c*dy, dyaw=_wrap(current['yaw']-previous['yaw']))
    _require(all(_finite(v) for v in result.values()), 'nonfinite odometry delta')
    return result


def prepare(capture, *, lidar_yaw_deg, max_odom_age_ms=250.):
    """Associate by preceding receive order, never interpolate publication clocks.

    Baseline advances only for usable scan updates; skipped scans retain accumulated
    absolute odometry. Adapter failure aborts replay, so uncertain updates never retry.
    """
    _require(_finite(lidar_yaw_deg) and abs(lidar_yaw_deg) <= 360, 'invalid lidar yaw')
    _require(_finite(max_odom_age_ms) and 0 < max_odom_age_ms <= 1000, 'invalid odometry age budget')
    latest = None
    previous = None
    records, skipped = [], []
    sequence = 0
    yaw = math.radians(lidar_yaw_deg)
    for frame in capture['frames']:
        if 'odom' in frame:
            latest = frame
        if 'laser' not in frame:
            continue
        sequence += 1
        scan = frame['laser']
        association = dict(method='receive_order_unverified', source_time_verified=False,
            beam_time_verified=False, deskew_applied=False, scan_source_stamp=frame['source_stamp'],
            scan_source_seq=frame['source_seq'], scan_receive_offset_ns=frame['receive_offset_ns'],
            odom_source_stamp=latest['source_stamp'] if latest else None,
            odom_source_seq=latest['source_seq'] if latest else None,
            odom_receive_offset_ns=latest['receive_offset_ns'] if latest else None,
            age_ms=(frame['receive_offset_ns']-latest['receive_offset_ns'])/1e6 if latest else None,
            publication_offset_ms=(frame['source_stamp']-latest['source_stamp'])*1000
                if latest and frame['source_stamp'] is not None and latest['source_stamp'] is not None else None)
        record = dict(scan_sequence=sequence, scan_id=scan['scan_id'], association=association)
        reason = None
        if latest is None:
            reason = 'missing_odometry'
        elif association['age_ms'] > max_odom_age_ms:
            reason = 'stale_odometry'
        elif not any(.15 <= r <= scan['range_max'] for r in scan['ranges']):
            reason = 'no_usable_ranges'
        if reason:
            skipped.append(dict(record, reason=reason))
            continue
        current = latest['odom']
        delta = _delta(previous, current) if previous else dict(dx=0., dy=0., dyaw=0.)
        # Keep raw arrays in the capture untouched. No sorting, beam-time inference,
        # scan reindexing or motion compensation. Match runtime's near-range filter.
        record['request'] = dict(op='update', scan=dict(angles=[a+yaw for a in scan['angles']],
            ranges=[r if r >= .15 else 0. for r in scan['ranges']]),
            range_max=scan['range_max'], sensor_x=-.1, odom=delta)
        record['absolute_odom'] = current.copy()
        records.append(record)
        previous = current
    return dict(records=records, skipped=skipped, scans=sequence)


def _exchange(process, request, deadline, budget):
    """Bound both pipe directions and reply size, including an adapter that stalls."""
    data = (json.dumps(request, allow_nan=False) + '\n').encode()
    output = bytearray()
    with selectors.DefaultSelector() as selector:
        selector.register(process.stdin, selectors.EVENT_WRITE)
        selector.register(process.stdout, selectors.EVENT_READ)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('offline adapter exceeded replay budget')
            events = selector.select(remaining)
            if not events:
                raise TimeoutError('offline adapter exceeded replay budget')
            for key, _ in events:
                if key.fileobj is process.stdin:
                    try:
                        written = os.write(process.stdin.fileno(), data[:65536])
                    except BrokenPipeError as exc:
                        raise ValueError('offline adapter closed input') from exc
                    data = data[written:]
                    if not data:
                        selector.unregister(process.stdin)
                else:
                    chunk = os.read(process.stdout.fileno(), 65536)
                    _require(bool(chunk), 'offline adapter closed output')
                    budget[0] -= len(chunk)
                    _require(budget[0] >= 0, 'offline adapter total reply limit exceeded')
                    output.extend(chunk)
                    _require(len(output) <= MAX_REPLY, 'offline adapter reply too large')
                    if b'\n' in output:
                        _require(not data and output.count(b'\n') == 1 and output.endswith(b'\n'),
                                 'unexpected offline adapter reply')
                        reply = _json(output)
                        _require(isinstance(reply, dict) and reply.get('ok') is True,
                                 'offline adapter rejected request')
                        return reply


def _trajectory(poses):
    return dict(final_pose=poses[-1], path_length_m=sum(math.hypot(b['x']-a['x'], b['y']-a['y'])
                for a, b in zip(poses, poses[1:])),
                net_displacement_m=math.hypot(poses[-1]['x']-poses[0]['x'], poses[-1]['y']-poses[0]['y']),
                accumulated_abs_yaw_deg=sum(abs(math.degrees(_wrap(b['yaw']-a['yaw'])))
                for a, b in zip(poses, poses[1:])))


def run(adapter, capture, *, lidar_yaw_deg, max_odom_age_ms=250., timeout_s=120.):
    """Execute only start_mapping + updates on an explicitly trusted local adapter.

    Per-scan results retain actual adapter diagnostics and all association metadata.
    Trajectory numbers describe estimates, not accuracy; all truth errors are null.
    """
    _require(_finite(timeout_s) and 0 < timeout_s <= 600, 'invalid replay timeout')
    prepared = prepare(capture, lidar_yaw_deg=lidar_yaw_deg, max_odom_age_ms=max_odom_age_ms)
    _require(bool(prepared['records']), 'no usable scan/odometry associations')
    deadline = time.monotonic() + timeout_s
    process = subprocess.Popen([str(Path(adapter).resolve())], stdin=subprocess.PIPE,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, start_new_session=True)
    results = []
    try:
        os.set_blocking(process.stdin.fileno(), False)
        os.set_blocking(process.stdout.fileno(), False)
        budget = [MAX_TOTAL_REPLY_BYTES]
        _exchange(process, dict(op='start_mapping'), deadline, budget)
        for record in prepared['records']:
            reply = _exchange(process, record['request'], deadline, budget)
            pose = reply.get('pose')
            _require(isinstance(pose, dict) and all(_finite(pose.get(k)) for k in ('x', 'y', 'yaw')),
                     'invalid adapter pose')
            diagnostics = reply.get('diagnostics')
            _require(diagnostics is None or isinstance(diagnostics, dict), 'invalid adapter diagnostics')
            results.append(dict(scan_sequence=record['scan_sequence'], scan_id=record['scan_id'],
                association=record['association'], pose=pose, diagnostics=diagnostics,
                tracking=reply.get('tracking'), quality=reply.get('quality')))
        process.stdin.close()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('offline adapter exceeded replay budget')
        try:
            process.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            raise TimeoutError('offline adapter exceeded replay budget') from None
        _require(process.returncode == 0, 'offline adapter failed')
    finally:
        # Also clean up descendants if the trusted adapter failed or timed out.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()
        process.stdin.close()
        process.stdout.close()
    diagnostics = [r['diagnostics'] or {} for r in results]
    def count_bool(key):
        values = [d[key] for d in diagnostics if type(d.get(key)) is bool]
        return sum(values) if values else None
    used = [d['used_points'] for d in diagnostics if _uint(d.get('used_points'), MAX_POINTS)]
    goodness = [d['goodness'] for d in diagnostics if _finite(d.get('goodness'))]
    associations = [r['association'] for r in results]
    stamps = [a['scan_source_stamp'] for a in associations]
    intervals = [1000*(b-a) for a, b in zip(stamps, stamps[1:]) if a is not None and b is not None]
    odom_poses = [r['absolute_odom'] for r in prepared['records']]
    return dict(capture_complete=capture['complete'], completion=capture['completion'],
        lidar_yaw_deg=lidar_yaw_deg, sensor_x=-.1, source_time_verified=False,
        beam_time_verified=False, deskew_applied=False, truth_available=False,
        updates=len(results), scans=prepared['scans'], skipped=prepared['skipped'],
        accepted_icp=count_bool('accepted'), map_insertions_paused=count_bool('mapping_paused'),
        used_points_min=min(used) if used else None,
        goodness_mean=sum(goodness)/len(goodness) if goodness else None,
        diagnostic_reasons=dict(Counter(d.get('reason') for d in diagnostics if isinstance(d.get('reason'), str))),
        trajectory=_trajectory([r['pose'] for r in results]),
        odometry_trajectory=_trajectory(odom_poses),
        association=dict(method='receive_order_unverified', max_odom_age_ms=max_odom_age_ms,
            age_ms_max=max(a['age_ms'] for a in associations),
            age_ms_mean=sum(a['age_ms'] for a in associations)/len(associations),
            source_stamp_semantics='publication time, measurement association unverified',
            scan_publication_interval_ms_min=min(intervals) if intervals else None,
            scan_publication_interval_ms_max=max(intervals) if intervals else None),
        final_xy_error_m=None, final_yaw_error_deg=None, wall_error_p95_m=None,
        walls_within_10cm=None, results=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--adapter', type=Path, required=True, help='trusted local executable, never read from capture')
    parser.add_argument('--lidar-yaw-deg', type=float, required=True,
                        help='additional bearing rotation; use 0 for vendor body-aligned SCANANGLE, 97.6 reproduces the former double-yaw bug')
    parser.add_argument('--max-odom-age-ms', type=float, default=250.)
    parser.add_argument('--timeout', type=float, default=120., help='total local adapter budget in seconds')
    parser.add_argument('--allow-incomplete', action='store_true', help='explicitly permit missing/lossy completion')
    args = parser.parse_args()
    try:
        capture = load(args.input, allow_incomplete=args.allow_incomplete)
        result = run(args.adapter, capture, lidar_yaw_deg=args.lidar_yaw_deg,
                     max_odom_age_ms=args.max_odom_age_ms, timeout_s=args.timeout)
        print(json.dumps(result, allow_nan=False))
    except (OSError, ValueError, TimeoutError) as exc:
        parser.exit(1, 'hardware replay: ' + str(exc) + '\n')


if __name__ == '__main__':
    main()
