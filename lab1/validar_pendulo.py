from pathlib import Path

import mujoco
import numpy as np

# Carregar o XML que está na mesma pasta deste script.
ficheiro_xml = Path(__file__).parent / "pendulo.xml"
model = mujoco.MjModel.from_xml_path(str(ficheiro_xml))
data = mujoco.MjData(model)

# Identificar a velocidade correspondente à articulação do pêndulo.
joint_id = model.joint("articulacao").id
dof_id = model.jnt_dofadr[joint_id]

# ============================================================
# PREVISÃO TEÓRICA: cápsula uniforme presa por uma extremidade
# Estes valores correspondem à geometria definida no teu XML.
# ============================================================

massa = 1.0         # kg
comprimento = 1.0   # m: distância entre os pontos de fromto
raio = 0.03        # m
gravidade = 9.81   # m/s²

# A cápsula contém um cilindro e duas pontas hemisféricas.
volume_cilindro = np.pi * raio**2 * comprimento
volume_pontas = (4.0 / 3.0) * np.pi * raio**3

# Distribuição da massa, assumindo densidade uniforme.
massa_cilindro = massa * volume_cilindro / (
    volume_cilindro + volume_pontas
)
massa_pontas = massa - massa_cilindro

# Momento de inércia transversal em torno do centro da cápsula.
inercia_centro = (
    massa_cilindro * (comprimento**2 / 12.0 + raio**2 / 4.0)
    + massa_pontas * (
        comprimento**2 / 4.0
        + 3.0 * comprimento * raio / 8.0
        + 2.0 * raio**2 / 5.0
    )
)

# Distância entre a articulação e o centro de massa.
distancia_cm = comprimento / 2.0

# Teorema dos eixos paralelos: inércia na articulação.
inercia_articulacao = inercia_centro + massa * distancia_cm**2

# Período teórico para pequenas oscilações, sem amortecimento.
periodo_teorico = 2.0 * np.pi * np.sqrt(
    inercia_articulacao / (massa * gravidade * distancia_cm)
)

# ============================================================
# MEDIÇÃO NA SIMULAÇÃO
# ============================================================

instantes_extremos = []

while data.time < 20.0:
    tempo_anterior = float(data.time)
    velocidade_anterior = float(data.qvel[dof_id])

    # Avançar um passo de física.
    mujoco.mj_step(model, data)

    velocidade_atual = float(data.qvel[dof_id])

    # Detetar a inversão da velocidade de negativa para positiva.
    # Isso identifica sempre o extremo do mesmo lado da oscilação.
    if velocidade_anterior < 0.0 <= velocidade_atual:
        # Interpolar para estimar o instante entre dois passos.
        fracao = -velocidade_anterior / (
            velocidade_atual - velocidade_anterior
        )
        instante = tempo_anterior + fracao * (
            data.time - tempo_anterior
        )
        instantes_extremos.append(instante)

if len(instantes_extremos) < 3:
    raise RuntimeError(
        "Oscilações insuficientes. Confirma a inclinação e a articulação."
    )

# Entre dois extremos consecutivos do mesmo lado passa um período.
periodos = np.diff(instantes_extremos)
periodo_medido = float(np.mean(periodos))
erro_percentual = (
    abs(periodo_medido - periodo_teorico) / periodo_teorico * 100.0
)

print(f"Período teórico:       {periodo_teorico:.6f} s")
print(f"Período simulado:      {periodo_medido:.6f} s")
print(f"Diferença relativa:    {erro_percentual:.4f} %")
print(f"Períodos medidos:      {len(periodos)}")

# Correção aproximada do período para a amplitude inicial de 5 graus.
amplitude = np.deg2rad(5.0)
periodo_corrigido = periodo_teorico * (1.0 + amplitude**2 / 16.0)

erro_corrigido = (
    abs(periodo_medido - periodo_corrigido)
    / periodo_corrigido
    * 100.0
)

print(f"Período corrigido:     {periodo_corrigido:.6f} s")
print(f"Diferença corrigida:   {erro_corrigido:.6f} %")