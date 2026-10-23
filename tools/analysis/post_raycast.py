"""post_raycast.py -- missing-post finder, method 1 attempt (2026-09-11).
Usage: python3 post_raycast.py <rig.json> <path_pose.csv> <flightlog.txt>...
KEPT FOR THE RECORD: a point-ray model is fooled by panel ends 6 mm from a
post and by 255 meaning "too close" as well as "far". post_holes.py is the
one that worked. See claude/POSTS.md.

Post detector: which lattice posts did the side beams see, judged from the
flight log alone. Method committed before the log (2026-09-11): cast each
side beam from the compiled path pose at the record's s; where the model's
first hit is a post, compare the reading with the range WITH that post and
WITHOUT it (next surface behind); vote."""
import json, math, sys, collections
CELL=180; HALF_WALL=6; POST_HALF=6
rig=json.load(open(sys.argv[1]))['rig']
# rects: (x0,y0,x1,y1) in maze mm
rects=[]
for i in range(7):
    for j in range(7):
        rects.append(('post',(i,j),(i*CELL-POST_HALF,j*CELL-POST_HALF,i*CELL+POST_HALF,j*CELL+POST_HALF)))
for w in rig['walls']:
    x1,y1,x2,y2=[v*CELL for v in w]
    if x1==x2: rects.append(('wall',tuple(w),(x1-HALF_WALL,min(y1,y2)+POST_HALF,x1+HALF_WALL,max(y1,y2)-POST_HALF)))
    else: rects.append(('wall',tuple(w),(min(x1,x2)+POST_HALF,y1-HALF_WALL,max(x1,x2)-POST_HALF,y1+HALF_WALL)))
def ray_rect(ox,oy,dx,dy,r):
    x0,y0,x1,y1=r; tmin=-1e9; tmax=1e9
    for o,d,lo,hi in ((ox,dx,x0,x1),(oy,dy,y0,y1)):
        if abs(d)<1e-9:
            if o<lo or o>hi: return None
        else:
            t1=(lo-o)/d; t2=(hi-o)/d
            if t1>t2: t1,t2=t2,t1
            tmin=max(tmin,t1); tmax=min(tmax,t2)
    if tmax<max(tmin,0): return None
    return max(tmin,0)
def cast(ox,oy,dx,dy,skip=None):
    best=None
    for kind,key,r in rects:
        if skip is not None and kind=='post' and key==skip: continue
        t=ray_rect(ox,oy,dx,dy,r)
        if t is not None and (best is None or t<best[0]): best=(t,kind,key)
    return best
# path pose table
path=[tuple(map(float,l.split(',')[:4])) for l in open(sys.argv[2])]
def pose_at(s):
    i=min(int(s/2.0), len(path)-1); i=max(i,0)
    s0,px,py,yaw=path[i]
    # path frame -> maze: launch N from (90,51)
    mx=90-py; my=51+px; hd=90+yaw
    return mx,my,hd
SENS={'L':(40,-15,72.8,+1),'R':(40,15,-71.0,-1)}   # x fwd, y left, beam angle from fwd; sign for lateral reading
FAR=230
def predicted_reading(sname, range_mm):
    x,y,ang,sgn=SENS[sname]
    lat = y + range_mm*math.sin(math.radians(ang))
    return sgn*lat
votes=collections.defaultdict(lambda: {'present':0,'absent':0,'amb':0})
def run(fn, label):
    n=0
    for line in open(fn):
        p=line.strip().split(',')
        if len(p)<12 or not p[0].isdigit(): continue
        s=float(p[1]); L=float(p[8]); R=float(p[11]); flags=int(p[7])
        mx,my,hd=pose_at(s)
        for sname,reading in (('L',L),('R',R)):
            x,y,ang,sgn=SENS[sname]
            c=math.cos(math.radians(hd)); sn=math.sin(math.radians(hd))
            ox=mx+x*c-y*sn; oy=my+x*sn+y*c
            a=math.radians(hd+ang); dx=math.cos(a); dy=math.sin(a)
            hit=cast(ox,oy,dx,dy)
            if hit is None or hit[1]!='post': continue
            t_with=hit[0]; post=hit[2]
            hit2=cast(ox,oy,dx,dy,skip=post)
            t_wo=hit2[0] if hit2 else 1e9
            rw=predicted_reading(sname,t_with); rwo=predicted_reading(sname,t_wo) if t_wo<1e8 else 999
            if rw>FAR: continue                      # post too far to see anyway
            rwo_c=min(rwo,FAR+25)
            if abs(rwo_c-rw)<25: votes[post]['amb']+=1; continue
            obs=reading if reading<255 else FAR+25
            dw=abs(obs-rw); dwo=abs(obs-rwo_c)
            if dw<20 and dw<dwo: votes[post]['present']+=1
            elif dwo<20 and dwo<dw: votes[post]['absent']+=1
            else: votes[post]['amb']+=1
            n+=1
    return n
for fn in sys.argv[3:]:
    print(fn, 'records judged', run(fn,fn))
print()
print('post     present absent  ambiguous  verdict')
for post in sorted(votes):
    v=votes[post]; tot=v['present']+v['absent']
    if tot<6: verdict='unobservable (too few clear looks)'
    elif v['absent']>=3*max(1,v['present']): verdict='MISSING'
    elif v['present']>=3*max(1,v['absent']): verdict='present'
    else: verdict='unclear'
    print(f"{post}   {v['present']:5d}  {v['absent']:5d}   {v['amb']:5d}     {verdict}")
seen=set(votes)
print('\nposts never looked at:', [ (i,j) for i in range(7) for j in range(7) if (i,j) not in seen])
