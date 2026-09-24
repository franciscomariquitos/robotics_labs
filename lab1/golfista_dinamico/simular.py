"""Swing por motores com equilibrio nos tornozelos.
O estado so e inicializado/reposto a pedido. Depois, mj_step integra a dinamica.
"""
from pathlib import Path
import argparse
import json
import threading
import time
import numpy as np
import mujoco
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation

P=Path(__file__).resolve().parent

class Referencia:
 def __init__(self,m,fator=3.):
  self.m=m;self.fator=fator;self.preparacao=1.
  with np.load(P/'trajetoria.npz',allow_pickle=False) as f:t=f['tempo'];q=f['qpos'].copy()
  for i in range(1,len(q)):
   if q[i,3:7]@q[i-1,3:7]<0:q[i,3:7]*=-1
  q=gaussian_filter1d(q,1.5,axis=0,mode='nearest')
  q[:,3:7]/=np.linalg.norm(q[:,3:7],axis=1)[:,None]
  # Assentar as solas, que tinham uma pequena folga na referencia cinematica.
  q[:,2]-=.002
  ids=m.actuator_trnid[:,0];qi=m.jnt_qposadr[ids];lim=m.jnt_range[ids]
  q[:,qi]=np.clip(q[:,qi],lim[:,0]+1e-5,lim[:,1]-1e-5)
  self.spline=CubicSpline(t*fator,q,axis=0,bc_type='clamped')
  self.duracao=float(t[-1]*fator)
 def pose(self,t):
  q=self.spline(np.clip(t-self.preparacao,0,self.duracao));q[3:7]/=np.linalg.norm(q[3:7]);return q
 def velocidade(self,t):
  h=.002;v=np.zeros(self.m.nv)
  mujoco.mj_differentiatePos(self.m,v,2*h,self.pose(t-h),self.pose(t+h));return v

class Simulacao:
 def __init__(self,fator=3.,motores=True):
  self.m=mujoco.MjModel.from_xml_path(str(P/'golfista.xml'));self.d=mujoco.MjData(self.m)
  self.rd=mujoco.MjData(self.m);self.ref=Referencia(self.m,fator)
  self.ids=self.m.actuator_trnid[:,0];self.qi=self.m.jnt_qposadr[self.ids]
  self.ankles={s:{ax:int(np.where(self.ids==self.m.joint('tornozelo_'+s+'_'+ax).id)[0][0]) for ax in ['pitch','roll']} for s in ['esq','dir']}
  self.J=np.zeros((3,self.m.nv));self.Jr=self.J.copy();self.jp=self.J.copy();self.jpr=self.J.copy()
  self.pelvis=self.m.body('pelvis').id
  self.head=self.m.geom('cabeca').id
  self.arm_geoms=[int(g) for side in ['esq','dir'] for b in ['braco_','cotovelo_body_','antebraco_','mao_'] for g in np.where(self.m.geom_bodyid==self.m.body(b+side).id)[0]]
  self.motores=motores;self.reiniciar()
 def ligar_motores(self,on):
  self.motores=bool(on);flag=int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
  if on:self.m.opt.disableflags &= ~flag
  else:self.m.opt.disableflags |= flag
 def reiniciar(self):
  # Unico local, para alem da inicializacao, que prescreve o estado da simulacao.
  mujoco.mj_resetData(self.m,self.d);self.d.qpos[:]=self.ref.pose(0);self.d.qvel[:]=0
  self.ligar_motores(self.motores);mujoco.mj_forward(self.m,self.d)
 def passo(self):
  m,d,rd=self.m,self.d,self.rd
  qr=self.ref.pose(d.time);vr=self.ref.velocidade(d.time)
  # ctrl contem alvos dos servos. Os atuadores position do XML geram
  # tau = kp*(q_alvo-q) - kv*qvel, limitado por forcerange, em N.m.
  # Isto NAO escreve os angulos reais em d.qpos.
  d.ctrl[:]=qr[self.qi]
  actual=Rotation.from_quat(d.qpos[3:7][[1,2,3,0]])
  target=Rotation.from_quat(qr[3:7][[1,2,3,0]])
  er=(actual*target.inv()).as_rotvec()
  ev=actual.apply(d.qvel[3:6])-target.apply(vr[3:6])
  # Estado de referencia separado, usado apenas para calculos cinematicos.
  rd.qpos[:]=qr;mujoco.mj_kinematics(m,rd);mujoco.mj_comPos(m,rd)
  mujoco.mj_jacSubtreeCom(m,d,self.J,self.pelvis)
  mujoco.mj_jacSubtreeCom(m,rd,self.Jr,self.pelvis)
  left=rd.site('apoio_esq').xpos[1];right=rd.site('apoio_dir').xpos[1]
  weight=np.clip((rd.subtree_com[self.pelvis,1]-right)/(left-right),0,1)
  drift=np.zeros(3);driftvel=np.zeros(3)
  for side,w in [('esq',weight),('dir',1-weight)]:
   a=d.site('apoio_'+side);b=rd.site('apoio_'+side)
   mujoco.mj_jacSite(m,d,self.jp,None,a.id);mujoco.mj_jacSite(m,rd,self.jpr,None,b.id)
   drift+=w*(a.xpos-b.xpos);driftvel+=w*(self.jp@d.qvel-self.jpr@vr)
  # Equilibrio relativo aos pes REAIS: nao tenta arrastar o corpo para
  # a localizacao original se os pes sofrerem um pequeno deslocamento.
  ep=d.subtree_com[self.pelvis]-rd.subtree_com[self.pelvis]-drift
  evcom=self.J@d.qvel-self.Jr@vr-driftvel
  bcom=1.5*ep+.6*evcom
  correction=1.2*er+.4*ev+np.array([-bcom[1],bcom[0],0])
  for side in ['esq','dir']:
   for axis in ['pitch','roll']:
    a=self.ankles[side][axis]
    # Projetar nos eixos atuais, em coordenadas do mundo. Isto e essencial
    # quando a bacia roda: os eixos do mundo e do corpo deixam de coincidir.
    d.ctrl[a]+=np.clip(correction@d.xaxis[self.ids[a]],-.4,.4)
  d.ctrl[:]=np.clip(d.ctrl,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
  mujoco.mj_step(m,d)


def testar(sim,duracao=None):
 m,d=sim.m,sim.d
 duration=duracao if duracao is not None else sim.ref.preparacao+sim.ref.duracao+5
 min_z=10.;gap=angle=joint_rms=violation=max_force_excess=joint_max=0.;worst_joint=0;min_head=1.;headcontacts=0;saturation=0;steps=0;trace=[];errlist=[];max_feet_shift=0.
 foot0={s:d.site('apoio_'+s).xpos.copy() for s in ['esq','dir']}
 ranges=m.jnt_range[sim.ids];t0=time.perf_counter();last_print=-1
 while d.time<duration:
  sim.passo();steps+=1
  if np.any(d.qfrc_applied) or np.any(d.xfrc_applied):raise RuntimeError('Forca externa inesperada.')
  if not np.isfinite(d.qpos).all():raise RuntimeError('Estado nao finito.')
  max_force_excess=max(max_force_excess,float(np.max(np.maximum(m.actuator_forcerange[:,0]-d.actuator_force,d.actuator_force-m.actuator_forcerange[:,1]))))
  saturation+=int(np.any(np.abs(d.actuator_force)>=m.actuator_forcerange[:,1]-.001))
  headcontacts+=sum(sim.head in [int(c.geom1),int(c.geom2)] for c in d.contact)
  if steps%10==0:
   mujoco.mj_forward(m,d)
   min_z=min(min_z,float(d.qpos[2]))
   a=d.site('pega_mao_dir');b=d.site('pega_taco_dir')
   gap=max(gap,float(np.linalg.norm(a.xpos-b.xpos)))
   angle=max(angle,float(np.arccos(np.clip((np.trace(a.xmat.reshape(3,3).T@b.xmat.reshape(3,3))-1)/2,-1,1))))
   err=d.qpos[sim.qi]-sim.ref.pose(d.time)[sim.qi]
   rms=float(np.sqrt(np.mean(err**2)));joint_rms=max(joint_rms,rms);errlist.append(rms)
   if np.max(np.abs(err))>joint_max:joint_max=float(np.max(np.abs(err)));worst_joint=int(np.argmax(np.abs(err)))
   violation=max(violation,float(np.maximum(ranges[:,0]-d.qpos[sim.qi],d.qpos[sim.qi]-ranges[:,1]).max()))
   for g in sim.arm_geoms:min_head=min(min_head,float(mujoco.mj_geomDistance(m,d,sim.head,g,.2,None)))
   # Deslocamento do pe esquerdo, planeado como apoio fixo: inclui deslize/rotacao.
   max_feet_shift=max(max_feet_shift,float(np.linalg.norm(d.site('apoio_esq').xpos[:2]-foot0['esq'][:2])))
   trace.append(np.r_[d.time,d.qpos,d.actuator_force])
  sec=int(d.time)
  if sec>last_print and steps>=10:
   print(f't={d.time:5.1f}s | bacia={d.qpos[2]:.3f}m | erro articular RMS={np.rad2deg(rms):.2f} graus',flush=True);last_print=sec
  if d.qpos[2]<.55:break
 warnings={str(i):int(w.number) for i,w in enumerate(d.warning) if w.number}
 finalspeed=float(np.linalg.norm(d.qvel[:3]))
 success=bool(d.time>=duration-.001 and min_z>.70 and joint_rms<np.deg2rad(15) and gap<.005 and angle<np.deg2rad(3) and headcontacts==0 and min_head>0 and max_force_excess<1e-6 and violation<np.deg2rad(2) and finalspeed<.05 and not warnings)
 report={'teste_dinamico_aprovado':success,'motores_ligados':sim.motores,'mujoco':mujoco.__version__,
 'duracao_simulada_s':float(d.time),'duracao_pedida_s':duration,'fator_tempo_movimento':sim.ref.fator,'periodo_simulacao_s':float(m.opt.timestep),
 'altura_minima_bacia_m':min_z,'velocidade_final_bacia_m_s':finalspeed,
 'erro_RMS_articular_maximo_graus':float(np.rad2deg(joint_rms)),'erro_RMS_articular_global_graus':float(np.rad2deg(np.sqrt(np.mean(np.array(errlist)**2)))),
 'erro_articular_individual_maximo_graus':float(np.rad2deg(joint_max)),'articulacao_com_maior_erro':m.joint(int(sim.ids[worst_joint])).name,
 'erro_maximo_pega_mm':gap*1000,'erro_maximo_orientacao_pega_graus':float(np.rad2deg(angle)),
 'folga_minima_bracos_cabeca_mm':min_head*1000,'contactos_bracos_cabeca_contados_por_passo':headcontacts,
 'violacao_maxima_limites_articulares_graus':float(np.rad2deg(violation)),'excesso_maximo_binario_Nm':max_force_excess,
 'percentagem_passos_com_algum_motor_saturado':100*saturation/steps,'deslocamento_horizontal_maximo_apoio_esquerdo_m':max_feet_shift,
 'forcas_externas_de_sustentacao':False,'bacia_livre':True,'avisos_mujoco':warnings,'tempo_de_calculo_s':time.perf_counter()-t0,
 'limitacoes':['teste nominal, sem certificacao de robustez','apoios adaptados; nao reproduz exatamente os pes da captura','outras auto-colisoes nao validadas','sem impacto na bola','parametros antropometricos aproximados']}
 suffix='' if sim.motores else '_sem_motores'
 (P/f'resultado_dinamico{suffix}.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
 np.savez_compressed(P/f'execucao_dinamica{suffix}.npz',amostras=trace,nq=m.nq)
 print(json.dumps(report,indent=2,ensure_ascii=False));return success


def visualizar(sim):
 import mujoco.viewer as vm
 state={'pause':False,'reset':False,'toggle':False};lock=threading.Lock()
 def key(k):
  with lock:
   if k==32:state['pause']=not state['pause']
   elif k==82:state['reset']=True
   elif k==77:state['toggle']=True
 print('SIMULACAO DINAMICA: motores + gravidade + contactos; bacia livre.')
 print(f'Movimento {sim.ref.fator:g} vezes mais lento que a captura; aguarda a preparacao inicial.')
 print('Espaco: pausa | R: reinicia | M: liga/desliga motores | rato: camara')
 print('No final mantem a postura; nao reinicia automaticamente.')
 with vm.launch_passive(sim.m,sim.d,key_callback=key) as viewer:
  viewer.cam.lookat[:]=[.25,0,1];viewer.cam.distance=3.8;viewer.cam.azimuth=135;viewer.cam.elevation=-12;viewer.opt.geomgroup[4]=0
  while viewer.is_running():
   start=time.perf_counter()
   with lock:s=state.copy();state['reset']=state['toggle']=False
   with viewer.lock():
    if s['reset']:sim.reiniciar()
    if s['toggle']:sim.ligar_motores(not sim.motores);print('Motores ligados.' if sim.motores else 'Motores desligados: movimento apenas sob gravidade, contactos e amortecimento.')
    if not s['pause']:
     for _ in range(10):sim.passo()
   viewer.sync();time.sleep(max(0,10*sim.m.opt.timestep-(time.perf_counter()-start)))

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--testar',action='store_true');p.add_argument('--fator-tempo',type=float,default=3.);p.add_argument('--duracao',type=float);p.add_argument('--sem-motores',action='store_true');a=p.parse_args()
 if a.fator_tempo<=0:p.error('--fator-tempo deve ser positivo.')
 if a.duracao is not None and a.duracao<=0:p.error('--duracao deve ser positiva.')
 sim=Simulacao(a.fator_tempo,not a.sem_motores)
 if a.testar:
  if not testar(sim,a.duracao):raise SystemExit(1)
 else:visualizar(sim)
if __name__=='__main__':main()
