"""Adapta CMU 64_01 a corpos rigidos e articulacoes. Referencia cinematica, sem dinamica."""
from pathlib import Path
import argparse,json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parent

def unit(v):return v/np.linalg.norm(v)
def frame(x,y):
 x=unit(x);y=unit(y-x*(x@y));return np.column_stack([x,y,np.cross(x,y)])
def skew(v):
 x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])
def right_jac(v):
 a=np.linalg.norm(v);s=skew(v)
 if a<1e-7:return np.eye(3)-s/2+s@s/6
 return np.eye(3)-(1-np.cos(a))/a**2*s+(a-np.sin(a))/a**3*s@s

def main(stride=1):
 m=mujoco.MjModel.from_xml_path(str(P/'golfista.xml'));d=mujoco.MjData(m)
 mujoco.mj_resetDataKeyframe(m,d,0);d.eq_active[:]=0
 qi=m.jnt_qposadr[m.actuator_trnid[:,0]];vi=m.jnt_dofadr[m.actuator_trnid[:,0]]
 qbase=d.qpos.copy()
 head=m.geom('cabeca').id
 arm_geoms=[int(g) for side in ['esq','dir'] for b in ['braco_','cotovelo_body_','antebraco_','mao_'] for g in np.where(m.geom_bodyid==m.body(b+side).id)[0]]
 with np.load(P/'captura_cmu_64_01.npz') as f: raw=f['pontos'];names=f['nomes'].tolist();fps=float(f['fps'])
 def markers(k):return dict(zip(names,raw[k]))
 def pelvis(p):
  front=(p['LFWT']+p['RFWT'])/2;back=(p['LBWT']+p['RBWT'])/2
  r=frame(front-back,p['LFWT']-p['RFWT']);return (front+back)/2-np.array([0,0,.09]),r
 _,pr0=pelvis(markers(0))
 foot0={}
 for s in ['L','R']:
  p=markers(0);foot0[s]=frame(p[s+'TOE']-p[s+'HEE'],p[s+'MT5']-p[s+'HEE'] if s=='L' else p[s+'HEE']-p[s+'MT5'])
 x=np.r_[d.qpos[:3],np.zeros(3),d.qpos[qi]]
 ranges=m.jnt_range[m.actuator_trnid[:,0]]
 lo=np.r_[[-1,-1,.65],[-3.2]*3,ranges[:,0]+1e-5];hi=np.r_[[1,1,1.2],[3.2]*3,ranges[:,1]-1e-5]
 frames=sorted(set(list(range(0,len(raw),stride))+[len(raw)-1]));Q=[];metrics=[];last_delta=np.zeros_like(x)
 # Corpos e sites usados repetidamente, resolvidos uma unica vez.
 ids={n:m.body(n).id for n in ['taco','pelvis','tronco','braco_esq','braco_dir','cotovelo_body_esq','cotovelo_body_dir','canela_esq','canela_dir','pe_esq','pe_dir']}
 corners=np.array([[a,b,-.09] for a in [-.095,.185] for b in [-.06,.06]])
 for it,k in enumerate(frames):
  p=markers(k);rootpos,pr=pelvis(p);pr=pr@pr0.T
  sho=(p['LSHO']+p['RSHO'])/2
  z=unit(sho-rootpos);y=p['LSHO']-p['RSHO'];y=unit(y-z*(z@y));tr=np.column_stack([np.cross(y,z),y,z])
  shaft=unit(p['WEP3']-p['WEP1']);wr=(p['LWRA']+p['LWRB'])/2
  grip=p['WEP1']+shaft*((wr-p['WEP1'])@shaft)
  # So o eixo da haste e observado: a rotacao em torno dele fica livre.
  targets=[]
  for s,side in [('L','esq'),('R','dir')]:
   targets.extend([('braco_'+side,np.zeros(3),p[s+'SHO'],8.),('cotovelo_body_'+side,np.zeros(3),p[s+'ELB'],5.),('canela_'+side,np.zeros(3),p[s+'KNE'],4.)])
  feet={}
  for s,side in [('L','esq'),('R','dir')]:
   rf=frame(p[s+'TOE']-p[s+'HEE'],p[s+'MT5']-p[s+'HEE'] if s=='L' else p[s+'HEE']-p[s+'MT5'])@foot0[s].T
   # Apoios planeados para a dinamica: esquerda plana; direita liberta o calcanhar apos a passagem do taco.
   pp0=markers(0);center=(pp0[s+'TOE']+pp0[s+'HEE'])/2;center[2]=0
   if s=='L' or k/fps<2.65:rf=np.eye(3)
   else:
    blend=np.clip((k/fps-2.65)/.25,0,1);blend=blend*blend*(3-2*blend)
    angles=Rotation.from_matrix(rf).as_euler('xyz');angles[0]=0;angles[1]=max(0,angles[1]);rf=Rotation.from_euler('xyz',angles*blend).as_matrix()
   bodypos=center-rf@np.array([.045,0,0])
   if s=='R' and k/fps>=2.65:bodypos=center+np.array([.14,0,0])-rf@np.array([.185,0,-.09])
   bodypos[2]=-np.min((corners@rf.T)[:,2])+.001
   feet[side]=(bodypos,rf)
  previous=x.copy();prediction=previous+.7*last_delta;cache={}
  dt=(frames[it]-frames[it-1])/fps if it else 1.
  step_limit=np.r_[[2*dt]*3,[8*dt]*3,[16*dt]*m.nu]
  lower=np.maximum(lo,previous-step_limit) if it else lo
  upper=np.minimum(hi,previous+step_limit) if it else hi
  def evaluate(v):
   if 'v' in cache and np.array_equal(cache['v'],v):return cache['res'],cache['jac']
   d.qpos[:]=qbase;d.qpos[:3]=v[:3]
   quat=Rotation.from_rotvec(v[3:6]).as_quat();d.qpos[3:7]=quat[[3,0,1,2]];d.qpos[qi]=v[6:]
   mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)
   transform=np.zeros((m.nv,len(v)));transform[:3,:3]=np.eye(3);transform[3:6,3:6]=right_jac(v[3:6]);transform[vi,np.arange(6,len(v))]=1
   res=[];jac=[]
   def point(body,local):
    bid=ids[body];rot=d.xmat[bid].reshape(3,3);pos=d.xpos[bid]+rot@local
    jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,jr,pos,bid)
    return pos,rot,jp@transform,jr@transform
   def site(name):
    sid=m.site(name).id;jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));mujoco.mj_jacSite(m,d,jp,jr,sid)
    return d.site_xpos[sid],d.site_xmat[sid].reshape(3,3),jp@transform,jr@transform
   def add(a,j,w):res.extend((w*np.asarray(a)).ravel());jac.extend(w*np.asarray(j).reshape(-1,len(v)))
   def rotjac(rot,jr):return np.stack([skew(jr[:,i])@rot for i in range(len(v))],axis=-1).reshape(9,len(v))
   for body,local,target,w in targets:
    pp,rr,jp,jr=point(body,local);add(pp-target,jp,w)
   pp,rr,jp,jr=point('pelvis',np.zeros(3));add(pp-rootpos,jp,8);add(rr-pr,rotjac(rr,jr),1)
   pp,rr,jp,jr=point('tronco',np.zeros(3));add(rr-tr,rotjac(rr,jr),3)
   left,rl,jl,wl=site('pega_mao_esq');right,rr,jr,wrj=site('pega_mao_dir');club,rc,jc,wc=site('pega_taco_dir')
   add(left-grip,jl,40);add(rl[:,2]+shaft,-skew(rl[:,2])@wl,12)
   add(right-club,jr-jc,500);add(rr-rc,rotjac(rr,wrj)-rotjac(rc,wc),60)
   for side,(pos,rot) in feet.items():
    pp,rr,jp,jr=point('pe_'+side,np.zeros(3));add(pp-pos,jp,100);add(rr-rot,rotjac(rr,jr),60)
    for c in corners:
     cp,_,cj,_=point('pe_'+side,c);add([min(cp[2],0)],cj[2:3] if cp[2]<0 else np.zeros((1,len(v))),500)
   for xx in [-.03,.08]:
    for yy in [-.027,.027]:
     for zz in [-1.022,-.978]:
      cp,_,cj,_=point('taco',np.array([xx,yy,zz]));add([min(cp[2]-.001,0)],cj[2:3] if cp[2]<.001 else np.zeros((1,len(v))),500)
   for gg in arm_geoms:
    pts=np.zeros(6);dist=mujoco.mj_geomDistance(m,d,head,gg,.1,pts)
    if dist<.015 and abs(dist)>1e-9:
     j1=np.zeros((3,m.nv));j2=np.zeros((3,m.nv))
     mujoco.mj_jac(m,d,j1,None,pts[:3],int(m.geom_bodyid[head]));mujoco.mj_jac(m,d,j2,None,pts[3:],int(m.geom_bodyid[gg]))
     derivative=((pts[3:]-pts[:3])/dist)@(j2-j1)@transform
     add([dist-.015],derivative[None,:],500)
    else:add([0.],np.zeros((1,len(v))),500)
   add(v-prediction,np.eye(len(v)),.3)
   cache.update(v=v.copy(),res=np.array(res),jac=np.array(jac))
   return cache['res'],cache['jac']
  sol=least_squares(lambda v:evaluate(v)[0],np.clip(x,lo,hi),jac=lambda v:evaluate(v)[1],bounds=(lower,upper),max_nfev=300 if it==0 else 100,ftol=1e-7,xtol=1e-7,gtol=1e-7)
  x=sol.x;last_delta=x-previous;evaluate(x);mujoco.mj_forward(m,d)
  err=np.linalg.norm(d.site('pega_mao_dir').xpos-d.site('pega_taco_dir').xpos)
  angle=Rotation.from_matrix(d.site('pega_mao_dir').xmat.reshape(3,3).T@d.site('pega_taco_dir').xmat.reshape(3,3)).magnitude()
  metrics.append([err,angle,np.linalg.norm(d.site('pega_mao_esq').xpos-grip),sol.cost])
  Q.append(d.qpos.copy())
  if it%10==0:print(f'{k/fps:.3f}s pega {err*1000:.3f}mm, orient {np.rad2deg(angle):.3f} graus, ajuste mao {metrics[-1][2]*1000:.1f}mm, custo {sol.cost:.4f}, eval {sol.nfev}',flush=True)
 np.savez_compressed(P/'trajetoria.npz',tempo=np.array(frames)/fps,qpos=Q,erros=metrics)
 print('Maximos',np.max(metrics,axis=0),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--passo',type=int,default=1);main(p.parse_args().passo)
