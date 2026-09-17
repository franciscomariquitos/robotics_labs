"""Gera o XML e calcula a postura inicial por cinemática inversa.
Executar apenas para reconstruir o modelo: requer scipy, numpy e mujoco.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import mujoco

PASTA = Path(__file__).resolve().parent
root = ET.Element('mujoco', model='golfista_v2')
def add(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
def note(parent, text): parent.append(ET.Comment(' ' + text + ' '))
def vec(x): return ' '.join(f'{v:.12g}' for v in x)
note(root, 'SI: metros, kg, segundos. Angulos XML em graus; qpos e controlos em radianos.')
add(root, 'compiler', angle='degree', autolimits='true')
add(root, 'option', timestep='.001', gravity='0 0 -9.81', integrator='implicitfast', iterations='100', tolerance='1e-10', cone='elliptic')
add(root, 'statistic', center='0.15 0 0.9', extent='2.4')
visual=add(root,'visual')
add(visual,'global',offwidth='1280',offheight='960')
default = add(root, 'default')
add(default, 'joint', damping='1', armature='.015')
add(default, 'geom', contype='0', conaffinity='0', rgba='.83 .64 .49 1')
world = add(root, 'worldbody')
add(world, 'light', pos='1 -3 4', dir='-0.2 0.5 -1')
add(world, 'light', pos='-2 2 3', diffuse='.5 .5 .5')
note(world, 'x: frente do jogador; y: esquerda; z: cima. Geometrias humanas sem auto-colisao nesta fase.')
add(world, 'geom', name='chao', type='plane', size='4 4 .1', rgba='.45 .58 .43 1', contype='1', conaffinity='6', friction='.9 .01 .001', condim='6', solref='.01 1')
rootbody = add(world, 'body', name='pelvis', pos='-.05 0 .90')
note(rootbody, 'Bacia livre: nao existe apoio invisivel nem fixacao ao mundo.')
add(rootbody, 'freejoint', name='base_livre')
add(rootbody, 'geom', name='pelvis_geom', type='ellipsoid', size='.12 .17 .10', mass='10', rgba='.2 .24 .33 1')
params = []
def joint(body, name, axis, limits, kp, kd, limit):
    add(body, 'joint', name=name, type='hinge', axis=axis, range=limits)
    params.append((name, kp, kd, limit))
for side, sign in [('esq',1), ('dir',-1)]:
    note(rootbody, f'Perna {side}: anca 3 eixos, joelho 1, tornozelo 2. Massas aproximadas.')
    thigh = add(rootbody, 'body', name=f'coxa_{side}', pos=f'0 {sign*.13} -.05')
    joint(thigh, f'anca_{side}_yaw', '0 0 1', '-50 50', 1500,80,150)
    joint(thigh, f'anca_{side}_roll', '1 0 0', '-45 45', 1500,80,150)
    joint(thigh, f'anca_{side}_pitch', '0 1 0', '-110 50', 1500,80,180)
    add(thigh, 'geom', type='capsule', fromto='0 0 0 0 0 -.40', size='.06', mass='7', rgba='.23 .28 .40 1')
    shin = add(thigh, 'body', name=f'canela_{side}', pos='0 0 -.40')
    joint(shin, f'joelho_{side}', '0 1 0', '0 145', 1500,70,150)
    add(shin, 'geom', type='capsule', fromto='0 0 0 0 0 -.40', size='.043', mass='3.5', rgba='.29 .35 .47 1')
    foot = add(shin, 'body', name=f'pe_{side}', pos='0 0 -.40')
    joint(foot, f'tornozelo_{side}_pitch', '0 1 0', '-50 40', 1200,60,120)
    joint(foot, f'tornozelo_{side}_roll', '1 0 0', '-30 30', 1200,60,90)
    add(foot, 'geom', name=f'sola_{side}', type='box', pos='.045 0 -.045', size='.14 .06 .045', mass='1', rgba='.12 .12 .14 1', contype='2', conaffinity='1', condim='6', friction='.9 .01 .001', solref='.01 1')
    add(foot, 'site', name=f'apoio_{side}', pos='.045 0 -.09', size='.007', rgba='1 .8 0 1')
trunk = add(rootbody, 'body', name='tronco', pos='0 0 .08')
note(trunk, 'Tronco relativo a bacia: rotacao axial, inclinacao lateral e flexao.')
joint(trunk, 'tronco_yaw','0 0 1','-70 70',700,45,120)
joint(trunk, 'tronco_roll','1 0 0','-35 35',700,45,120)
joint(trunk, 'tronco_pitch','0 1 0','-25 55',900,55,160)
add(trunk, 'geom', type='ellipsoid', pos='0 0 .20', size='.12 .205 .25', mass='25', rgba='.08 .42 .65 1')
add(trunk, 'geom', type='capsule', fromto='0 0 .43 0 0 .50', size='.043', mass='.5')
add(trunk, 'geom', name='cabeca', type='sphere', pos='0 0 .59', size='.095', mass='4.5')
for side, sign in [('esq',1), ('dir',-1)]:
    add(trunk, 'geom', name=f'ombro_visual_{side}', type='sphere', pos=f'0 {sign*.21} .40', size='.063', mass='0', rgba='.08 .42 .65 1')
    arm=add(trunk,'body',name=f'braco_{side}',pos=f'0 {sign*.23} .40')
    note(arm, 'Ombro com 3 eixos: nao confundir rotacao axial com a pronacao do antebraco.')
    joint(arm,f'ombro_{side}_pitch','0 -1 0','-70 170',450,25,80)
    joint(arm,f'ombro_{side}_roll',f'{sign} 0 0','-80 150',450,25,80)
    joint(arm,f'ombro_{side}_axial','0 0 1','-100 100',250,15,50)
    add(arm,'geom',type='capsule',fromto='0 0 0 0 0 -.30',size='.043',mass='3')
    elbow=add(arm,'body',name=f'cotovelo_body_{side}',pos='0 0 -.30')
    joint(elbow,f'cotovelo_{side}','0 -1 0','0 145',250,15,50)
    add(elbow,'geom',type='sphere',size='.035',mass='.1')
    forearm=add(elbow,'body',name=f'antebraco_{side}')
    joint(forearm,f'pronacao_{side}','0 0 1','-100 100',100,6,20)
    add(forearm,'geom',type='capsule',fromto='0 0 0 0 0 -.27',size='.032',mass='1.4')
    hand=add(forearm,'body',name=f'mao_{side}',pos='0 0 -.27')
    joint(hand,f'pulso_{side}_flex','0 1 0','-80 80',100,6,20)
    joint(hand,f'pulso_{side}_desvio','1 0 0','-40 40',100,6,20)
    add(hand,'geom',name=f'luva_{side}',type='box',pos='0 0 -.04',size='.025 .036 .04',mass='.4',rgba='.92 .92 .87 1')
    add(hand,'site',name=f'pega_mao_{side}',pos='0 0 -.04',size='.006',rgba='1 .2 .1 1')
    if side=='esq':
        note(hand, 'Taco rigidamente ligado a mao esquerda; a mao direita fecha a cadeia por weld.')
        club=add(hand,'body',name='taco',pos='0 0 -.04')
        add(club,'geom',name='punho_taco',type='capsule',fromto='0 0 .035 0 0 -.17',size='.014',mass='.05',rgba='.1 .1 .1 1')
        add(club,'geom',name='haste_taco',type='capsule',fromto='0 0 -.17 0 0 -1.00',size='.005',mass='.10',rgba='.65 .68 .72 1',contype='4',conaffinity='9')
        add(club,'geom',name='cabeca_taco',type='box',pos='.025 0 -1.00',size='.055 .027 .022',mass='.20',rgba='.25 .27 .3 1',contype='4',conaffinity='9',solref='.005 1')
        add(club,'site',name='pega_taco_dir',pos='0 0 -.09',size='.006',rgba='.2 1 .1 1')
        add(club,'site',name='centro_cabeca_taco',pos='.025 0 -1.00',size='.004')
note(world, 'Bola livre, sem contacto inicial com o taco. Sem aerodinamica nesta fase.')
ball=add(world,'body',name='bola',pos='.7155 -.095 .02135')
add(ball,'freejoint',name='bola_livre')
add(ball,'geom',name='bola_geom',type='sphere',size='.02135',mass='.0459',rgba='.98 .98 .98 1',contype='8',conaffinity='5',condim='6',friction='.35 .001 .0002',solref='.005 1')
equality=add(root,'equality')
note(equality,'Pega idealizada sem deslizamento. Impoe posicao e orientacao relativas; nao simula dedos.')
add(equality,'weld',name='pega_direita',site1='pega_mao_dir',site2='pega_taco_dir',solref='.015 1',solimp='.95 .99 .001',torquescale='.08')
act=add(root,'actuator')
note(act,'Motores de binario, gear=1. Controlador PD e compensacao estatica no Python.')
for name,kp,kd,limit in params:
    add(act,'motor',name='motor_'+name,joint=name,gear='1',ctrllimited='true',ctrlrange=f'-{limit} {limit}')
custom=add(root,'custom')
add(custom,'numeric',name='kp',data=vec([p[1] for p in params]))
add(custom,'numeric',name='kd',data=vec([p[2] for p in params]))

m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
d=mujoco.MjData(m)
d.eq_active[:]=0
# Bacia livre na postura de referencia; IK resolve as pernas e os bracos.
def index(name):return int(m.jnt_qposadr[m.joint(name).id])
d.qpos[index('tronco_pitch')]=np.deg2rad(20)

def solve(names, targets, seed):
    inds=np.array([index(n) for n in names])
    jids=[m.joint(n).id for n in names]
    lo=m.jnt_range[jids,0]+1e-6; hi=m.jnt_range[jids,1]-1e-6
    def residual(q):
        d.qpos[inds]=q
        mujoco.mj_forward(m,d)
        out=[]
        for site,pos,rot in targets:
            s=d.site(site)
            out.extend((s.xpos-pos)*5)
            out.extend(Rotation.from_matrix(rot.T@s.xmat.reshape(3,3)).as_rotvec())
        return np.array(out)
    result=least_squares(residual,np.clip(seed,lo,hi),bounds=(lo,hi),max_nfev=2000,ftol=1e-12,xtol=1e-12,gtol=1e-12)
    d.qpos[inds]=result.x
    error=np.linalg.norm(residual(result.x))
    if error>1e-5: raise RuntimeError(f'IK falhou: {names[0]} residual={error}')
    return error
for side,sign in [('esq',1),('dir',-1)]:
    names=[f'anca_{side}_yaw',f'anca_{side}_roll',f'anca_{side}_pitch',f'joelho_{side}',f'tornozelo_{side}_pitch',f'tornozelo_{side}_roll']
    solve(names,[(f'apoio_{side}',np.array([.015,sign*.20,-.0002]),np.eye(3))],np.deg2rad([0,sign*5,-20,40,-20,-sign*5]))
R=Rotation.from_euler('y',-20,degrees=True).as_matrix()
left_target=np.array([.35,0,.98])
for side in ['esq','dir']:
    names=[f'ombro_{side}_pitch',f'ombro_{side}_roll',f'ombro_{side}_axial',f'cotovelo_{side}',f'pronacao_{side}',f'pulso_{side}_flex',f'pulso_{side}_desvio']
    target=left_target if side=='esq' else left_target+R@np.array([0,0,-.09])
    solve(names,[(f'pega_mao_{side}',target,R)],np.deg2rad([20,-25,0,30,0,0,0]))
mujoco.mj_forward(m,d)
print('COM:',d.subtree_com[m.body('pelvis').id])
print('Erro pega inicial (m):',np.linalg.norm(d.site('pega_mao_dir').xpos-d.site('pega_taco_dir').xpos))
keys=add(root,'keyframe')
add(keys,'key',name='preparacao',qpos=vec(d.qpos))
ET.indent(root,space='    ')
ET.ElementTree(root).write(PASTA/'golfista.xml',encoding='utf-8',xml_declaration=True)
print('Criado:',PASTA/'golfista.xml')
