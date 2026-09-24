# Golfista — modelo base em MuJoCo

Este diretório contém o modelo atual do conjunto jogador + taco + bola.
O ficheiro `golfista.xml` é a **única fonte de verdade do modelo MuJoCo**:
geometria, massas, articulações, limites, contactos, actuators, ganhos `Kp`/`Kd`
e a postura inicial estão todos definidos nele.

**Não existe qualquer script que regenere ou reescreva o XML.**
As alterações ao modelo devem ser feitas diretamente em `golfista.xml`.

Nesta etapa o modelo mantém apenas a postura de preparação; ainda não executa
o swing completo.

## Ficheiros

- `golfista.xml` — modelo físico completo e keyframe inicial `preparacao`.
- `executar.py` — carrega o XML, aplica o controlador e executa os testes.
- `resultados.json` — resultados do último teste automático executado.
- `preview.png` — imagem de referência da postura atual.
- `README.md` — documentação desta versão.

## Executar

Requer Python, MuJoCo e NumPy.

A partir de qualquer diretório:

```bash
python3 ~/robotics_labs/lab1/golfista/executar.py
```

O `executar.py` **apenas lê** `golfista.xml`; não o modifica nem o reescreve.

Não abrir apenas o XML com `mujoco.viewer` para avaliar esta postura: o viewer
não carrega automaticamente o keyframe `preparacao` nem executa o controlador
Python. Como a bacia é livre, o jogador necessita da atuação definida no
`executar.py` para manter a postura.

## Estrutura do modelo

- Bacia livre: 3 translações + 3 rotações; sem actuator na base.
- Cada perna: anca com 3 eixos, joelho com 1 e tornozelo com 2.
- Tronco: rotação axial, flexão/extensão e inclinação lateral.
- Cada braço: ombro com 3 eixos, cotovelo com 1, pronação/supinação com 1 e
  pulso com 2.
- Cabeça e pescoço são rígidos relativamente ao tronco.
- Total: 29 coordenadas articulares atuadas, mais a base livre e a bola livre.
- A mão esquerda está rigidamente ligada ao taco.
- A mão direita fecha a cadeia através de uma restrição `weld` entre sites.
- O apoio é produzido pelos contactos das solas com o chão.

## Postura inicial

A postura inicial já está guardada diretamente no XML:

```xml
<keyframe>
    <key name="preparacao" qpos="..." />
</keyframe>
```

O vetor `qpos` foi previamente calculado para colocar os pés no chão e as duas
mãos na pega. A partir de agora este valor faz parte do próprio modelo e só deve
ser alterado deliberadamente no `golfista.xml`.

## Controlo atual

O `executar.py` usa controlo PD articular com referência constante e uma
compensação estática de binário calculada quando o modelo é carregado:

```text
tau = Kp (q_ref - q) - Kd q_dot + tau_ff
```

Os ganhos `Kp` e `Kd` estão armazenados no próprio `golfista.xml`, na secção
`<custom>`. Os limites dos motores também estão no XML, na secção `<actuator>`.

A compensação estática distribui o peso pelos dois pontos de apoio dos pés para
estimar os binários necessários na postura inicial. Não aplica forças externas
de sustentação: as forças reais de apoio são calculadas pelo solver de contacto
do MuJoCo.

Este controlador é adequado apenas para validar a postura atual. Não é ainda o
controlador final do swing.

## Teste automático

```bash
python3 ~/robotics_labs/lab1/golfista/executar.py --testar --duracao 10
```

O teste corre sem janela e grava `resultados.json`. Verifica:

- erro de posição e orientação da pega;
- limites articulares;
- altura da bacia;
- apoio bilateral;
- saturação dos actuators;
- ausência de contacto taco-bola nesta fase;
- avisos numéricos do MuJoCo;
- erro de seguimento da postura.

Critérios atuais de engenharia:

- erro de pega < 3 mm;
- erro de orientação da pega < 2 graus;
- altura da bacia > 0,75 m;
- apoio bilateral após os primeiros 0,5 s;
- nenhuma saturação de actuator;
- violação dos limites < 0,001 rad;
- erro articular < 5 graus;
- velocidade final da bacia < 0,01 m/s;
- nenhum contacto taco-bola durante a preparação.

Estes critérios servem para validar esta etapa do desenvolvimento; não provam
que o modelo seja biomecanicamente validado nem que o swing seja realista.

## Limitações atuais

- Segmentos humanos representados por corpos rígidos e geometrias simples.
- Massas, limites articulares, limites de binário e ganhos ainda são parâmetros
  de modelação e não uma calibração biomecânica completa.
- Pega idealizada rígida, sem dedos nem deslizamento.
- Auto-colisões do corpo estão desativadas nesta fase.
- Colisões principais ativas: pés-chão, taco-chão, taco-bola e bola-chão.
- Sem aerodinâmica da bola.
- O teste atual valida apenas a postura nominal; swing, impacto e perturbações
  por grau de liberdade ainda serão desenvolvidos.

## Regra de desenvolvimento

`golfista.xml` não deve ser gerado automaticamente por outro ficheiro.
Qualquer alteração ao modelo é feita diretamente no XML e fica registada pelo
Git. O Python pode ler o XML, controlar a simulação e produzir resultados, mas
não deve substituir silenciosamente o modelo.
