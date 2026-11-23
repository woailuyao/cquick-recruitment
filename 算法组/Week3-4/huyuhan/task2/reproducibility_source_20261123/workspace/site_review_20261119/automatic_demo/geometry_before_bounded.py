"""Camera-to-carrier ground projection and class-aware SE(2) matching.
Carrier: x forward, y left, z up. Camera: x right, y down, z forward.
All map coordinates in meters; map yaw positive counterclockwise.
"""
import math,itertools
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
CLASS_ALIASES={'PenalityPoint':'PenaltyPoint'}
def normalize_class_name(name):
 return CLASS_ALIASES.get(name,name)
NAMES=['LCross','TCross','XCross','PenaltyPoint']
def model_class_names_ok(names):
 return set(names)==set(range(len(NAMES))) and [normalize_class_name(names[i]) for i in range(len(NAMES))]==NAMES
def rot(a):
 c,s=math.cos(a),math.sin(a);return np.array([[c,-s],[s,c]])
def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi
def invalid(status,n=0,error=None):
 return dict(valid=False,status=status,marker_count=int(n),residual=error,residual_unit='m',x_carrier_map=None,y_carrier_map=None,yaw_carrier_map=None,x_camera_map=None,y_camera_map=None,yaw_camera_map=None,matches=[])
def transform(R,t):
 R,t=np.asarray(R,float),np.asarray(t,float)
 if R.shape!=(3,3) or t.shape!=(3,) or not np.isfinite(R).all() or not np.isfinite(t).all() or not np.allclose(R.T@R,np.eye(3),atol=1e-5) or abs(np.linalg.det(R)-1)>1e-5 or t[2]<=0:raise ValueError('Invalid camera-to-carrier transform/height')
 return R,t
def ray_ground(xy,R,t,max_distance=10.):
 R,t=transform(R,t);v=R@np.array([*xy,1.]);v/=np.linalg.norm(v)
 if v[2]>=-.025:return None
 d=-t[2]/v[2];p=t+d*v
 if d<=0 or not np.isfinite(p).all() or np.linalg.norm(p[:2]-t[:2])>max_distance:return None
 return p[:2]
def pixel_ground(uv,K,dist,R,t):
 import cv2
 K=np.asarray(K,float)
 if K.shape!=(3,3) or not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0:raise ValueError('Invalid camera matrix')
 points=cv2.undistortPoints(np.asarray(uv,float).reshape(-1,1,2),K,np.asarray(dist,float)).reshape(-1,2)
 return [ray_ground(p,R,t) for p in points]
def _read_map_raw(path):
 import yaml
 obj=yaml.safe_load(Path(path).read_text(encoding='utf-8'));out=[]
 def add(ident,cls,xy):
  cls='PenaltyPoint' if cls=='PenalityPoint' else cls;p=np.asarray(xy,float)
  if cls not in NAMES or p.shape!=(2,) or not np.isfinite(p).all():raise ValueError('Unsupported map class/coordinate')
  out.append(dict(id=str(ident),class_id=NAMES.index(cls),xy=p.tolist()))
 if not isinstance(obj,dict):raise ValueError('Map must be a YAML mapping')
 if 'landmarks' in obj:
  for r in obj['landmarks']:add(r['id'],r.get('type',r.get('class_name')),[r['x'],r['y']])
 else:
  for cls,values in obj.items():
   if cls not in NAMES+['PenalityPoint']:continue
   if isinstance(values,dict):
    for ident,xy in values.items():add(ident,cls,[xy['x'],xy['y']] if isinstance(xy,dict) else xy)
   elif isinstance(values,list):
    for i,xy in enumerate(values):add(xy.get('id',f'{cls}_{i}') if isinstance(xy,dict) else f'{cls}_{i}',cls,[xy['x'],xy['y']] if isinstance(xy,dict) else xy)
   else:raise ValueError('Map schema needs explicit adapter')
 if len(out)<3 or len({m['id'] for m in out})!=len(out):raise ValueError('Need >=3 map points with unique IDs; verify schema')
 return out
def read_map(path,include_unverified=False):
 import yaml
 path=Path(path)
 obj=yaml.safe_load(path.read_text(encoding='utf-8'))
 markers=_read_map_raw(path)
 verification={}
 if isinstance(obj,dict) and 'landmarks' in obj:
  verification={str(r['id']):r.get('verified',True) is not False for r in obj['landmarks']}
 elif isinstance(obj,dict):
  for cls,values in obj.items():
   if cls not in NAMES+list(CLASS_ALIASES):continue
   if isinstance(values,dict):
    verification.update({str(k):v.get('verified',True) is not False for k,v in values.items() if isinstance(v,dict)})
   elif isinstance(values,list):
    verification.update({str(v.get('id',f'{cls}_{i}')):v.get('verified',True) is not False for i,v in enumerate(values) if isinstance(v,dict)})
 return [dict(m,verified=verification.get(m['id'],True)) for m in markers if include_unverified or verification.get(m['id'],True)]
def fit(a,b):
 aa,bb=a.mean(0),b.mean(0);u,_,vt=np.linalg.svd((a-aa).T@(b-bb));R=vt.T@u.T
 if np.linalg.det(R)<0:vt[-1]*=-1;R=vt.T@u.T
 return R,bb-R@aa
def match(obs,markers,Rc,tc,gate=.22,max_rms=.15,ambiguity_margin=.04):
 markers=[m for m in markers if m.get('verified') is not False]
 if len(obs)<3:return invalid('insufficient_markers',len(obs))
 if len(markers)<3:return invalid('insufficient_verified_map_markers',len(obs))
 if len(obs)>12:return invalid('too_many_observations')
 a=np.array([o['xy'] for o in obs],float);b=np.array([m['xy'] for m in markers],float);ac=np.array([o['class_id'] for o in obs]);bc=np.array([m['class_id'] for m in markers]);candidates={};seen=set()
 if not np.isfinite(a).all() or not np.isfinite(b).all():return invalid('invalid_observations')
 for i,j in itertools.combinations(range(len(a)),2):
  va=a[j]-a[i];da=np.linalg.norm(va)
  if da<.25:continue
  for p in np.flatnonzero(bc==ac[i]):
   for q in np.flatnonzero(bc==ac[j]):
    vb=b[q]-b[p]
    if p==q or abs(da-np.linalg.norm(vb))>max(.12,.08*da):continue
    theta=wrap(math.atan2(vb[1],vb[0])-math.atan2(va[1],va[0]));R=rot(theta);t=b[p]-R@a[i];key=(round(float(t[0])/.04),round(float(t[1])/.04),round(theta/math.radians(2)))
    if key in seen:continue
    seen.add(key)
    if len(seen)>6000:return invalid('search_limit')
    cost=np.linalg.norm((a@R.T+t)[:,None]-b[None,:],axis=2);cost[ac[:,None]!=bc[None,:]]=1e6;ri,ci=linear_sum_assignment(cost);keep=cost[ri,ci]<=gate;ri,ci=ri[keep],ci[keep]
    if len(ri)<3 or np.linalg.svd(a[ri]-a[ri].mean(0),compute_uv=False)[-1]<.06:continue
    R,t=fit(a[ri],b[ci]);error=np.linalg.norm(a[ri]@R.T+t-b[ci],axis=1)
    if np.any(error>gate):continue
    rms=float(np.sqrt(np.mean(error**2)));theta=math.atan2(R[1,0],R[0,0]);key=(round(float(t[0])/.04),round(float(t[1])/.04),round(theta/math.radians(2)));item=(len(ri),rms,R,t,ri,ci)
    if key not in candidates or (len(ri),-rms)>(candidates[key][0],-candidates[key][1]):candidates[key]=item
 if not candidates:return invalid('no_consistent_match')
 ranked=sorted(candidates.values(),key=lambda x:(-x[0],x[1]));n,e,R,t,ri,ci=ranked[0];theta=math.atan2(R[1,0],R[0,0])
 if e>max_rms:return invalid('high_residual',n,e)
 for nn,ee,RR,tt,*_ in ranked[1:]:
  distinct=np.linalg.norm(tt-t)>.30 or abs(wrap(math.atan2(RR[1,0],RR[0,0])-theta))>math.radians(12)
  if distinct and nn==n and ee<=e+ambiguity_margin:return invalid('ambiguous',n,e)
 Rc,tc=transform(Rc,tc);forward=Rc[:2,2]
 if np.linalg.norm(forward)<.1:return invalid('camera_heading_undefined',n,e)
 cam=R@tc[:2]+t
 return dict(valid=True,status='ok',marker_count=n,residual=e,residual_unit='m',x_carrier_map=float(t[0]),y_carrier_map=float(t[1]),yaw_carrier_map=math.degrees(theta),x_camera_map=float(cam[0]),y_camera_map=float(cam[1]),yaw_camera_map=math.degrees(wrap(theta+math.atan2(forward[1],forward[0]))),matches=[dict(observation=int(i),map_id=markers[int(j)]['id']) for i,j in zip(ri,ci)])
