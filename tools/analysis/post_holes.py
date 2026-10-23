"""post_holes.py -- missing-post finder, method 2 (2026-09-11, D10).
Usage: python3 post_holes.py <path_pose.csv> <flightlog.txt>...
path_pose.csv = s,x,y,yaw,in_arc every 2 mm from the compiled route (path
frame: x along the launch heading, y left; the harness in test/ dumps it).
See claude/POSTS.md in the project.

Holes in wall faces: a side beam reading a wall steadily (50..120 mm) that
jumps to 255 for 1-4 samples and returns to the same wall = a missing post
(the 12 mm gap between two panels). Located by ray from the path pose."""
import math, sys
path=[tuple(map(float,l.split(',')[:4])) for l in open(sys.argv[1])]
def pose_at(s):
    i=max(0,min(int(s/2.0), len(path)-1)); s0,px,py,yaw=path[i]; return 90-py, 51+px, 90+yaw
SENS={'L':(40,-15,72.8),'R':(40,15,-71.0)}
def hit(s,sname,lat):
    mx,my,hd=pose_at(s); x,y,ang=SENS[sname]
    c=math.cos(math.radians(hd)); sn=math.sin(math.radians(hd))
    ox=mx+x*c-y*sn; oy=my+x*sn+y*c
    r=(lat+15)/math.sin(math.radians(abs(ang)))
    a=math.radians(hd+ang); return ox+r*math.cos(a), oy+r*math.sin(a), hd
found={}
for fn in sys.argv[2:]:
    rows=[]
    for line in open(fn):
        p=line.strip().split(',')
        if len(p)<12 or not p[0].isdigit(): continue
        rows.append((float(p[1]),int(p[8]),int(p[11]),int(p[7])))
    for sname,col in (('L',1),('R',2)):
        v=[r[col] for r in rows]; n=len(v)
        i=0
        while i<n:
            if not (50<=v[i]<=120): i+=1; continue
            # wall run with holes allowed
            j=i; holes=[]; k=i
            while k<n:
                if 50<=v[k]<=120: j=k; k+=1; continue
                # excursion
                m=k
                while m<n and v[m]==255: m+=1
                if m<n and 50<=v[m]<=120 and 2<=(rows[m][0]-rows[k][0])<=30 and abs(v[m]-v[j])<25:
                    holes.append((k,m-1,v[j])); k=m; continue
                break
            if j-i>=8:
                for (a,b,wall) in holes:
                    if a-i<4 or j-b<4: continue      # need wall on both sides
                    s=0.5*(rows[a][0]+rows[b][0]); hx,hy,hd=hit(s,sname,wall); ln=rows[b][0]-rows[a][0]
                    px,py=round(hx/180),round(hy/180); d=math.hypot(hx/180-px,hy/180-py)*180
                    found.setdefault((px,py),[]).append((fn,sname,s,ln,d,hd))
            i=max(j+1,k+1)
for post in sorted(found):
    e=found[post]
    laps=sorted(set(x[0] for x in e))
    print(f"post {post}: seen as a hole in {len(laps)} lap(s): " + "; ".join(f"{x[0][:6]} {x[1]} s={x[2]:.0f} {x[3]:.0f} mm long, {x[4]:.0f} mm off, hd {x[5]:.0f}" for x in e))
