#!/usr/bin/env python3
"""Deterministic offline SLAM generation and replay; never connects to a robot."""
import argparse
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess

MAX_BYTES = 64 * 1024 * 1024
LDS_X = -0.1


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _box(x0, y0, x1, y1):
    return [[x0,y0,x1,y0],[x1,y0,x1,y1],[x1,y1,x0,y1],[x0,y1,x0,y0]]


def _advance(pose, v, w, dt):
    x,y,a=pose
    dx=v*dt if abs(w)<1e-9 else v/w*math.sin(w*dt)
    dy=0. if abs(w)<1e-9 else v/w*(1-math.cos(w*dt))
    c,s=math.cos(a),math.sin(a)
    return [x+c*dx-s*dy,y+s*dx+c*dy,a+w*dt]


def _hit(px,py,dx,dy,wall):
    x1,y1,x2,y2=wall
    ex,ey=x2-x1,y2-y1
    denominator=dx*ey-dy*ex
    if abs(denominator)<1e-12:return None
    t=((x1-px)*ey-(y1-py)*ex)/denominator
    u=((x1-px)*dy-(y1-py)*dx)/denominator
    return t if t>0 and 0<=u<=1 else None


def generate(*, steps=25, period=.25, v=.5, w=.4, mode='rolling', odom_scale=.8, seed=1, drop=0.):
    if type(steps) is not int or not 1<=steps<=10000 or type(seed) is not int:
        raise ValueError('invalid generation size or seed')
    if not all(_finite(x) for x in (period,v,w,odom_scale,drop)) or not 0<period<=1 or not 0<=drop<=1 or not 0<odom_scale<=2 or abs(v)>1 or abs(w)>4:
        raise ValueError('invalid synthetic motion parameters')
    if mode not in ('instant','rolling','deskew'):raise ValueError('invalid synthetic acquisition mode')
    walls=_box(-3.,-2.,4.,3.)+_box(1.8,1.,2.2,1.4)
    start=[-1.,0.,0.]
    header={'format':'robot-slam-replay-v1','walls':walls,'start':start,
            'scan_period':period,'acquisition':mode,'seed':seed,'odom_scale':odom_scale,
            'timing_model':'synthetic_verified'}
    records=[];rng=random.Random(seed)
    p=start
    for i in range(steps):
        if i:p=_advance(p,v,w,period)
        angles=[];ranges=[]
        for k in range(360):
            if rng.random()<drop:continue
            angle=-math.pi+2*math.pi*k/360
            dt=0. if mode=='instant' else (k/359-.5)*period
            x,y,yaw=_advance(p,v,w,dt)
            lx,ly=x+LDS_X*math.cos(yaw),y+LDS_X*math.sin(yaw)
            dx,dy=math.cos(yaw+angle),math.sin(yaw+angle)
            hits=[t for t in (_hit(lx,ly,dx,dy,wall) for wall in walls) if t is not None]
            if not hits or min(hits)>6.:continue
            distance=min(hits)
            if mode=='deskew':
                # This oracle uses explicit synthetic acquisition poses, not
                # an inferred timing model for the physical Player driver.
                wx,wy=lx+distance*dx-p[0],ly+distance*dy-p[1]
                bx=math.cos(p[2])*wx+math.sin(p[2])*wy-LDS_X
                by=-math.sin(p[2])*wx+math.cos(p[2])*wy
                angle,distance=math.atan2(by,bx),math.hypot(bx,by)
            angles.append(angle);ranges.append(distance)
        displacement=_advance([0.,0.,0.],v,w,period) if i else [0.,0.,0.]
        request={'op':'update','scan':{'angles':angles,'ranges':ranges},'range_max':6.,'sensor_x':LDS_X,
                 'odom':dict(zip(('dx','dy','dyaw'),[d*odom_scale for d in displacement])),
                 'stamp':1000.+i*period,'motion_compensated':mode in ('instant','deskew')}
        records.append({'request':request,'truth':p.copy(),'timing_model':'synthetic_verified'})
    return header,records


def _validate(header,records):
    if not isinstance(header,dict) or header.get('format')!='robot-slam-replay-v1':raise ValueError('invalid replay format')
    walls=header.get('walls');start=header.get('start')
    if not isinstance(walls,list) or not 1<=len(walls)<=1000 or any(not isinstance(wall,list) or len(wall)!=4 or not all(_finite(v) for v in wall) for wall in walls):
        raise ValueError('invalid replay walls')
    if not isinstance(start,list) or len(start)!=3 or not all(_finite(v) for v in start):raise ValueError('invalid replay origin')
    if not isinstance(records,list) or not 1<=len(records)<=10000:raise ValueError('invalid replay length')
    for record in records:
        if not isinstance(record,dict):raise ValueError('invalid replay record')
        request=record.get('request');truth=record.get('truth')
        if not isinstance(request,dict) or request.get('op')!='update' or set(request)-{'op','scan','odom','range_max','sensor_x','stamp','motion_compensated'}:
            raise ValueError('replay accepts only scan updates')
        if 'motion_compensated' in request and type(request['motion_compensated']) is not bool:raise ValueError('invalid compensation declaration')
        if not isinstance(truth,list) or len(truth)!=3 or not all(_finite(v) for v in truth):raise ValueError('invalid replay truth')
        scan=request.get('scan',{})
        if not isinstance(scan,dict):raise ValueError('invalid replay scan')
        angles=scan.get('angles');ranges=scan.get('ranges')
        if not isinstance(angles,list) or not isinstance(ranges,list) or len(angles)!=len(ranges) or len(angles)>4096 or not all(_finite(v) for v in angles+ranges):
            raise ValueError('invalid replay scan')
        odom=request.get('odom',{})
        if not isinstance(odom,dict) or set(odom)!=set(('dx','dy','dyaw')) or not all(_finite(v) for v in odom.values()):raise ValueError('invalid replay odometry')
        if not all(_finite(request.get(k)) for k in ('range_max','sensor_x','stamp')):raise ValueError('invalid replay timing or geometry')


def save(path,header,records):
    _validate(header,records)
    payload='\n'.join(json.dumps(item,allow_nan=False,separators=(',',':')) for item in [header,*records])+'\n'
    data=payload.encode()
    if len(data)>MAX_BYTES:raise ValueError('replay file is too large')
    fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'wb') as stream:stream.write(data)


def load(path):
    with Path(path).open('rb') as stream:
        data=stream.read(MAX_BYTES+1)
    if len(data)>MAX_BYTES:raise ValueError('replay file is too large')
    rows=[json.loads(line) for line in data.splitlines() if line.strip()]
    if not rows:raise ValueError('empty replay')
    header,records=rows[0],rows[1:]
    _validate(header,records)
    return header,records


def _wall_distance(walls,x,y):
    best=math.inf
    for x0,y0,x1,y1 in walls:
        dx,dy=x1-x0,y1-y0
        norm=dx*dx+dy*dy
        fraction=max(0.,min(1.,((x-x0)*dx+(y-y0)*dy)/norm)) if norm else 0.
        best=min(best,math.hypot(x-x0-fraction*dx,y-y0-fraction*dy))
    return best


def run(adapter,header,records,*,timeout_s=30.):
    _validate(header,records)
    if not _finite(timeout_s) or not 0<timeout_s<=120:raise ValueError('invalid replay timeout')
    requests=[{'op':'start_mapping'},*[record['request'] for record in records],{'op':'map'}]
    payload='\n'.join(json.dumps(row,allow_nan=False) for row in requests)+'\n'
    process=subprocess.Popen([str(adapter)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,start_new_session=True)
    try:
        output,_=process.communicate(payload.encode(),timeout=timeout_s)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGKILL)
        process.communicate(timeout=2)
        raise TimeoutError('offline adapter exceeded replay budget') from None
    if process.returncode or len(output)>MAX_BYTES:raise ValueError('offline adapter failed or reply is too large')
    replies=[json.loads(line) for line in output.splitlines() if line.strip()]
    if len(replies)!=len(requests) or not all(isinstance(r,dict) and r.get('ok') for r in replies):raise ValueError('offline adapter rejected replay')
    updates=replies[1:-1]
    poses=[reply.get('pose') for reply in updates]
    if not all(isinstance(p,dict) and all(_finite(p.get(k)) for k in ('x','y','yaw')) for p in poses):raise ValueError('invalid adapter poses')
    last=poses[-1];truth=records[-1]['truth'];sx,sy,sa=header['start'];c,s=math.cos(sa),math.sin(sa)
    tx,ty=c*(truth[0]-sx)+s*(truth[1]-sy),-s*(truth[0]-sx)+c*(truth[1]-sy)
    yaw_error=(last['yaw']-(truth[2]-sa)+math.pi)%(2*math.pi)-math.pi
    grid=replies[-1].get('grid',{})
    w,h=grid.get('w'),grid.get('h')
    if type(w) is not int or type(h) is not int or w<0 or h<0 or w*h>2_000_000 or not isinstance(grid.get('cells'),str):raise ValueError('invalid adapter grid')
    for key in ('res','x_min','y_min'):
        if not _finite(grid.get(key)):raise ValueError('invalid grid geometry')
    if grid['res']<=0:raise ValueError('invalid grid resolution')
    cells=[int(cell) for cell in grid['cells'].split(',')] if grid['cells'] else []
    if len(cells)!=w*h or any(v< -1 or v>100 for v in cells):raise ValueError('invalid grid cells')
    errors=[]
    for index,value in enumerate(cells):
        if value<65:continue
        mx=grid['x_min']+(index%w+.5)*grid['res'];my=grid['y_min']+(index//w+.5)*grid['res']
        x,y=sx+c*mx-s*my,sy+s*mx+c*my
        errors.append(_wall_distance(header['walls'],x,y))
    errors.sort()
    diagnostics=[reply.get('diagnostics') or {} for reply in updates]
    counts=[d['used_points'] for d in diagnostics if type(d.get('used_points')) is int]
    accepted=[d['accepted'] for d in diagnostics if type(d.get('accepted')) is bool]
    paused=[d['mapping_paused'] for d in diagnostics if type(d.get('mapping_paused')) is bool]
    return {'map_insertions_paused':sum(paused) if paused else None,'updates':len(updates),'accepted_icp':sum(accepted) if accepted else None,
            'used_points_min':min(counts) if counts else None,
            'final_xy_error_m':math.hypot(last['x']-tx,last['y']-ty),'final_yaw_error_deg':abs(math.degrees(yaw_error)),
            'occupied_cells':len(errors),'walls_within_10cm':sum(e<=.1 for e in errors)/len(errors) if errors else None,
            'wall_error_p95_m':errors[int(.95*(len(errors)-1))] if errors else None}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    generate_command=commands.add_parser('generate')
    generate_command.add_argument('output',type=Path)
    for name,default in (('period',.25),('v',.5),('w',.4),('odom-scale',.8),('drop',0.)):
        generate_command.add_argument('--'+name,type=float,default=default)
    generate_command.add_argument('--steps',type=int,default=25)
    generate_command.add_argument('--seed',type=int,default=1)
    generate_command.add_argument('--mode',choices=('instant','rolling','deskew'),default='rolling')
    replay_command=commands.add_parser('run')
    replay_command.add_argument('input',type=Path)
    replay_command.add_argument('--adapter',type=Path,required=True)
    replay_command.add_argument('--timeout',type=float,default=30.)
    args=parser.parse_args()
    try:
        if args.command=='generate':
            header,records=generate(steps=args.steps,period=args.period,v=args.v,w=args.w,mode=args.mode,odom_scale=args.odom_scale,seed=args.seed,drop=args.drop)
            save(args.output,header,records)
            print(json.dumps({'saved':str(args.output),'frames':len(records),'timing_model':'synthetic_verified'}))
        else:
            header,records=load(args.input)
            print(json.dumps(run(args.adapter.resolve(),header,records,timeout_s=args.timeout),allow_nan=False))
    except (OSError,ValueError,TimeoutError):
        parser.exit(1,'slam replay: invalid input, existing output or failed/timed-out local adapter\n')

if __name__=='__main__':main()
