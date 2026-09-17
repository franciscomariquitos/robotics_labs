"""Postura de preparacao dinamica. Nao executa ainda um swing.
Uso: python3 executar.py  |  python3 executar.py --testar --duracao 10
"""
import argparse
import json
import time
from pathlib import Path
import mujoco
import numpy as np

PASTA = Path(__file__).resolve().parent

def preparar():
    model = mujoco.MjModel.from_xml_path(str(PASTA / 'golfista.xml'))
    data = mujoco.MjData(model)
    # O keyframe contem a solucao de cinematica inversa. Nao usamos a pose zero.
    mujoco.mj_resetDataKeyframe(model, data, model.key('preparacao').id)
    mujoco.mj_forward(model, data)
    joints = model.actuator_trnid[:, 0]
    qi = model.jnt_qposadr[joints]
    vi = model.jnt_dofadr[joints]
    referencia = data.qpos[qi].copy()
    kp = model.numeric('kp').data.copy()
    kd = model.numeric('kd').data.copy()

    # Compensacao estatica calculada uma vez na postura inicial.
    # Distribuimos o peso pelos dois pes; os pontos de aplicacao
    # ficam na projecao x do COM, dentro das solas nesta postura.
    # Isto calcula os binarios dos motores: NAO aplica forcas externas
    # e NAO cancela a gravidade na bacia. O apoio real vem dos contactos.
    bid = model.body('pelvis').id
    com = data.subtree_com[bid].copy()
    peso = model.body_subtreemass[bid] * np.linalg.norm(model.opt.gravity)
    pes = [data.site('apoio_esq').xpos.copy(), data.site('apoio_dir').xpos.copy()]
    fracao_esq = (com[1] - pes[1][1]) / (pes[0][1] - pes[1][1])
    if not 0 <= fracao_esq <= 1:
        raise RuntimeError('COM fora do apoio lateral.')
    suporte = np.zeros(model.nv)
    for lado, ponto, fracao in zip(['esq', 'dir'], pes, [fracao_esq, 1-fracao_esq]):
        if abs(com[0] - ponto[0]) > .14:
            raise RuntimeError('COM fora do comprimento da sola.')
        ponto[0] = com[0]
        jp = np.zeros((3, model.nv))
        jr = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jp, jr, ponto, model.body('pe_'+lado).id)
        suporte += jp.T @ np.array([0, 0, peso*fracao])
    compensacao = (data.qfrc_bias - suporte)[vi]
    return model, data, qi, vi, referencia, kp, kd, compensacao


def controlar(model, data, qi, vi, referencia, kp, kd, compensacao):
    # Binarios limitados, calculados a cada passo. Nao impomos qpos.
    tau = kp * (referencia - data.qpos[qi]) - kd * data.qvel[vi] + compensacao
    data.ctrl[:] = np.clip(tau, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
    mujoco.mj_step(model, data)


def medir(model, data):
    # Atualizar as posicoes Cartesianas para o estado acabado de integrar.
    mujoco.mj_forward(model, data)
    a = data.site('pega_mao_dir')
    b = data.site('pega_taco_dir')
    distancia = float(np.linalg.norm(a.xpos-b.xpos))
    R = a.xmat.reshape(3,3).T @ b.xmat.reshape(3,3)
    angulo = float(np.arccos(np.clip((np.trace(R)-1)/2, -1, 1)))
    chao = model.geom('chao').id
    contacto = {}
    for lado in ['esq','dir']:
        sola = model.geom('sola_'+lado).id
        contacto[lado] = any({int(c.geom1),int(c.geom2)} == {chao,sola} for c in data.contact)
    com = data.subtree_com[model.body('pelvis').id].copy()
    return distancia, angulo, contacto, com


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--testar', action='store_true', help='Teste sem janela, guarda resultados.json.')
    parser.add_argument('--duracao', type=float, default=10.)
    args = parser.parse_args()
    model, data, qi, vi, ref, kp, kd, ff = preparar()
    print(f'{model.nu} articulacoes atuadas; bacia livre; pega direita ligada por weld.')
    print('Apoio apenas por contacto dos pes. Sem forcas externas de sustentacao.')
    if args.testar:
        if args.duracao <= 1: parser.error('--duracao deve ser superior a 1 segundo.')
        max_pega = max_angulo = max_erro = max_vel = max_violacao = 0.
        faltas_apoio = 0
        passos_saturados = 0
        contactos_taco_bola = 0
        bola_inicial = data.body("bola").xpos.copy()
        bola_id = model.geom("bola_geom").id
        taco_ids = {model.geom("haste_taco").id, model.geom("cabeca_taco").id}
        altura_inicial = float(data.qpos[2])
        minimo_z = altura_inicial
        while data.time < args.duracao:
            controlar(model,data,qi,vi,ref,kp,kd,ff)
            if not np.all(np.isfinite(data.qpos)): raise RuntimeError('Estado nao finito.')
            dist, ang, contato, com = medir(model,data)
            contactos_taco_bola += sum(1 for c in data.contact if (int(c.geom1)==bola_id and int(c.geom2) in taco_ids) or (int(c.geom2)==bola_id and int(c.geom1) in taco_ids))
            max_pega=max(max_pega,dist); max_angulo=max(max_angulo,ang)
            max_erro=max(max_erro,float(np.max(np.abs(data.qpos[qi]-ref))))
            minimo_z=min(minimo_z,float(data.qpos[2]))
            max_vel=max(max_vel,float(np.linalg.norm(data.qvel[:3])))
            jr=model.jnt_range[model.actuator_trnid[:,0]]
            max_violacao=max(max_violacao,float(np.max(np.maximum(jr[:,0]-data.qpos[qi],data.qpos[qi]-jr[:,1]))))
            if data.time > .5 and not all(contato.values()): faltas_apoio+=1
            if np.any(np.abs(data.ctrl) >= model.actuator_ctrlrange[:,1]-.00001): passos_saturados+=1
        warnings={str(i):int(w.number) for i,w in enumerate(data.warning) if w.number}
        aprovado=(max_pega<.003 and np.rad2deg(max_angulo)<2 and minimo_z>.75 and faltas_apoio==0 and passos_saturados==0 and max_violacao<.001 and not warnings and contactos_taco_bola==0 and max_erro<np.deg2rad(5) and np.linalg.norm(data.qvel[:3])<.01)
        resultado={
            'duracao_s':float(data.time), 'teste_postura_aprovado':bool(aprovado),
            'max_erro_pega_mm':max_pega*1000, 'max_erro_orientacao_pega_graus':float(np.rad2deg(max_angulo)),
            'max_erro_articular_graus':float(np.rad2deg(max_erro)),
            'max_violacao_limites_graus':float(np.rad2deg(max_violacao)),
            'altura_inicial_bacia_m':altura_inicial, 'altura_minima_bacia_m':minimo_z,
            'velocidade_final_bacia_m_s':float(np.linalg.norm(data.qvel[:3])),
            'passos_sem_apoio_bilateral_apos_05s':faltas_apoio, 'passos_com_saturacao':passos_saturados,
            'contactos_taco_bola':contactos_taco_bola,
            'deslocamento_horizontal_bola_mm':float(np.linalg.norm(data.body('bola').xpos[:2]-bola_inicial[:2])*1000),
            'COM_final_m':com.tolist(), 'avisos_mujoco':warnings,
            'nota':'Verifica esta postura nominal; nao valida swing, perturbacoes ou biomecanica humana.'}
        (PASTA/'resultados.json').write_text(json.dumps(resultado,indent=2,ensure_ascii=False)+'\n')
        print(json.dumps(resultado,indent=2,ensure_ascii=False))
        if not aprovado: raise SystemExit(1)
    else:
        import mujoco.viewer as viewer_module
        with viewer_module.launch_passive(model,data) as viewer:
            viewer.cam.lookat[:] = [.15,0,.9]
            viewer.cam.distance=3.2; viewer.cam.azimuth=130; viewer.cam.elevation=-12
            while viewer.is_running():
                inicio=time.perf_counter()
                # 10 passos de 1 ms por atualizacao grafica.
                for _ in range(10): controlar(model,data,qi,vi,ref,kp,kd,ff)
                viewer.sync()
                restante=10*model.opt.timestep-(time.perf_counter()-inicio)
                if restante>0: time.sleep(restante)

if __name__=='__main__':main()
