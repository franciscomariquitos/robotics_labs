"""Reproduz a captura adaptada ao modelo articulado. CINEMATICA, ainda sem controlo dinamico.
Uso: python3 executar.py | python3 executar.py --testar
"""
from pathlib import Path
import argparse,json,threading,time
import mujoco
import numpy as np
P=Path(__file__).resolve().parent

class Movimento:
 def __init__(self):
  self.m=mujoco.MjModel.from_xml_path(str(P/'golfista.xml'));self.d=mujoco.MjData(self.m)
  with np.load(P/'trajetoria.npz',allow_pickle=False) as f:self.t=f['tempo'];self.q=f['qpos'];self.erros=f['erros']
  with np.load(P/'captura_cmu_64_01.npz',allow_pickle=False) as f:self.p=f['pontos'];self.nomes=f['nomes'].tolist();self.fps=float(f['fps'])
  self.v=np.empty((len(self.t)-1,self.m.nv))
  for i in range(len(self.v)):mujoco.mj_differentiatePos(self.m,self.v[i],self.t[i+1]-self.t[i],self.q[i],self.q[i+1])
  self.duration=float(self.t[-1]);self.aplicar(0)
 def aplicar(self,t):
  t=float(np.clip(t,0,self.duration));i=min(max(np.searchsorted(self.t,t,side='right')-1,0),len(self.v)-1)
  self.d.qpos[:]=self.q[i];mujoco.mj_integratePos(self.m,self.d.qpos,self.v[i],t-self.t[i])
  self.d.qvel[:]=0;self.d.time=t;mujoco.mj_forward(self.m,self.d)
 def marcadores(self,t):
  u=np.clip(t*self.fps,0,len(self.p)-1);a=int(u);b=min(a+1,len(self.p)-1);return (1-u+a)*self.p[a]+(u-a)*self.p[b]

def altura_caixa(m,d,name):
 g=m.geom(name).id;s=m.geom_size[g];r=d.geom_xmat[g].reshape(3,3)
 return float(d.geom_xpos[g,2]-np.abs(r[2])@s)

def testar(mov):
 m,d=mov.m,mov.d;ids=m.actuator_trnid[:,0];qi=m.jnt_qposadr[ids];lim=m.jnt_range[ids]
 gap=angle=violation=0.;feet=club=float('inf');shaft=0.
 for t in np.linspace(0,mov.duration,4*(len(mov.t)-1)+1):
  mov.aplicar(t)
  if not np.isfinite(d.qpos).all():raise RuntimeError('Estado nao finito')
  a=d.site('pega_mao_dir');b=d.site('pega_taco_dir')
  gap=max(gap,float(np.linalg.norm(a.xpos-b.xpos)))
  angle=max(angle,float(np.arccos(np.clip((np.trace(a.xmat.reshape(3,3).T@b.xmat.reshape(3,3))-1)/2,-1,1))))
  violation=max(violation,float(np.maximum(lim[:,0]-d.qpos[qi],d.qpos[qi]-lim[:,1]).max()))
  feet=min(feet,altura_caixa(m,d,'sola_esq'),altura_caixa(m,d,'sola_dir'));club=min(club,altura_caixa(m,d,'cabeca_taco'))
  p=mov.marcadores(t);axis=p[mov.nomes.index('WEP3')]-p[mov.nomes.index('WEP1')];axis/=np.linalg.norm(axis)
  actual=-d.site('pega_mao_esq').xmat.reshape(3,3)[:,2];shaft=max(shaft,float(np.arccos(np.clip(axis@actual,-1,1))))
 jumps=np.rad2deg(np.abs(np.diff(mov.q[:,qi],axis=0)));j=int(jumps.max(axis=0).argmax())
 approved=bool(gap<.003 and np.rad2deg(angle)<1 and violation<1e-8 and feet>-.002 and club>-.002 and jumps.max()<12 and mov.erros[:,2].max()<.04 and np.rad2deg(shaft)<10)
 report={'etapa':'adaptacao cinematica; nao certifica dinamica ou biomecanica humana','aprovado_criterios_cinematicos':approved,
 'mujoco':mujoco.__version__,'articulacoes_atuadas':int(m.nu),'amostras_captura':len(mov.t),'instantes_verificados':4*(len(mov.t)-1)+1,
 'erro_max_fecho_maos_mm':gap*1000,'erro_max_orientacao_maos_graus':float(np.rad2deg(angle)),
 'violacao_max_limites_graus':float(np.rad2deg(violation)),'altura_min_solas_mm':feet*1000,'altura_min_cabeca_taco_mm':club*1000,
 'erro_rms_posicao_pega_referencia_mm':float(np.sqrt(np.mean(mov.erros[:,2]**2))*1000),'erro_max_posicao_pega_referencia_mm':float(mov.erros[:,2].max()*1000),
 'erro_max_direcao_haste_graus':float(np.rad2deg(shaft)),
 'maior_variacao_articular_entre_amostras_graus':float(jumps.max()),'articulacao_dessa_variacao':m.joint(int(ids[j])).name,
 'nao_validado':['binarios e saturacoes','equilibrio dinamico','impacto e voo da bola','auto-colisoes humanas','continuidade das aceleracoes','anatomia calibrada'],
 'nota':'Poses interpoladas por integracao de deslocamentos articulares; nao por passos de dinamica.'}
 (P/'validacao.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(json.dumps(report,indent=2,ensure_ascii=False))
 if not approved:raise SystemExit(1)

def main():
 a=argparse.ArgumentParser(description=__doc__);a.add_argument('--testar',action='store_true');a.add_argument('--velocidade',type=float,default=.5);args=a.parse_args()
 if not 0<args.velocidade<=2:a.error('Velocidade deve estar entre 0 e 2.')
 mov=Movimento()
 if args.testar:return testar(mov)
 import mujoco.viewer as viewer_module
 state={'pause':False,'reset':False,'speed':args.velocidade,'markers':False,'camera':0};lock=threading.Lock()
 def key(k):
  with lock:
   if k==32:state['pause']=not state['pause']
   elif k==82:state['reset']=True
   elif k==77:state['markers']=not state['markers']
   elif k==262:state['speed']=min(2,state['speed']*2)
   elif k==263:state['speed']=max(.125,state['speed']/2)
   elif k in [49,50,51]:state['camera']=k-49
 print('SWING ADAPTADO: corpos rigidos, 29 articulacoes, bacia livre e duas maos no taco.')
 print('REPRODUCAO CINEMATICA — motores ainda nao executam o movimento.')
 print('Espaco pausa | R reinicia | M marcadores de referencia | Setas velocidade | 1/2/3 camaras')
 print('Ha cerca de 1 s de preparacao na captura antes do swing.')
 with viewer_module.launch_passive(mov.m,mov.d,key_callback=key) as viewer:
  viewer.cam.lookat[:]=[.3,0,1];viewer.cam.distance=4.;viewer.opt.geomgroup[4]=0
  cams=[(135,-12),(0,-8),(90,-8)];lastcam=-1;t=0.;last=time.perf_counter()
  while viewer.is_running():
   now=time.perf_counter();dt=min(now-last,.1);last=now
   with lock:s=state.copy();state['reset']=False
   if s['reset']:t=0
   if not s['pause']:t=(t+dt*s['speed'])%(mov.duration+.7)
   with viewer.lock():
    mov.aplicar(min(t,mov.duration));viewer.user_scn.ngeom=0
    if s['markers']:
     for p in mov.marcadores(min(t,mov.duration)):
      g=viewer.user_scn.geoms[viewer.user_scn.ngeom]
      mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.01]*3),p,np.eye(3).ravel(),np.array([1,.65,.1,.7],dtype=np.float32));viewer.user_scn.ngeom+=1
    if s['camera']!=lastcam:viewer.cam.azimuth,viewer.cam.elevation=cams[s['camera']];lastcam=s['camera']
   viewer.sync();time.sleep(.005)
if __name__=='__main__':main()
