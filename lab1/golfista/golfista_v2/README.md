# Golfista v2 — postura de preparação

Esta versão substitui a base fixa por uma bacia livre e pernas articuladas.
O objetivo desta etapa é sustentar uma postura de preparação com duas mãos
na pega. **Ainda não executa um swing e não é um modelo biomecânico validado.**

## Executar

Requer Python, MuJoCo 3.13.0 e NumPy (já usados nos exercícios anteriores).
A partir de qualquer pasta:

```bash
python3 ~/robotics_labs/lab1/golfista/golfista_v2/executar.py
```

Não abrir apenas o XML com `mujoco.viewer`: isso não carrega automaticamente
o keyframe de preparação nem executa o controlador. Como a bacia é livre,
o corpo cai sem atuação adequada.

O script carrega o keyframe `preparacao`, calculado por cinemática inversa,
e aplica binários limitados a cada passo. Não reescreve as posições durante
a simulação. A postura mantém-se parada; o movimento do swing será outra etapa.
Os comandos do painel Control são substituídos pelo controlador Python.

## Estrutura

- Bacia livre: 3 translações + 3 rotações; sem atuador na base.
- Cada perna: anca com 3 eixos, joelho com 1 e tornozelo com 2.
- Tronco: rotação axial, flexão/extensão e inclinação lateral.
- Cada braço: ombro com 3 eixos, cotovelo com 1, pronação/supinação com 1,
  pulso com 2. Cabeça e pescoço rígidos relativamente ao tronco.
- Total: 29 coordenadas articulares atuadas + 6 velocidades da base livre,
  antes da redução causada pelas restrições. A bola tem mais 6 velocidades.
- Mão esquerda rigidamente ligada ao taco; mão direita ligada ao segundo
  ponto de pega, separado por 9 cm, através de uma restrição `weld` entre sites.
- Apoio por contactos das solas com o chão. Sem weld nos pés, sem apoio da bacia,
  sem compensação artificial da gravidade nos corpos e sem forças externas.

## Controlo

PD articular com referência constante e compensação estática de binário.
A compensação é calculada na postura inicial, repartindo o peso por dois
pontos de apoio dentro das solas. Serve para calcular binários nos motores;
não aplica essas forças diretamente no simulador. As forças reais de apoio
são produzidas pelo solver de contacto.

A compensação estática e os ganhos desta etapa não constituem um controlador
completo de equilíbrio para um swing ou para grandes perturbações.

## Teste reproduzível

```bash
python3 ~/robotics_labs/lab1/golfista/golfista_v2/executar.py --testar --duracao 10
```

Executa sem janela e grava `resultados.json`. Verifica erro da pega,
orientação relativa, limites articulares, altura da bacia, apoio bilateral,
saturação dos motores, ausência de contacto taco–bola e avisos numéricos.

Critérios de engenharia deste teste, não requisitos do enunciado:
- erro de pega < 3 mm e erro de orientação < 2 graus;
- altura da bacia > 0,75 m;
- apoio bilateral em todos os passos após 0,5 s;
- nenhuma saturação nem aviso numérico;
- violação de limites < 0,001 rad;
- erro de seguimento < 5 graus e velocidade final da bacia < 0,01 m/s;
- nenhum contacto taco–bola durante a preparação.

Os resultados incluídos foram produzidos com MuJoCo 3.13.0, Python 3.12
num ambiente Linux. Reexecutar no WSL para confirmar o comportamento local.

## Reconstruir a postura (opcional)

`golfista.xml` já contém a postura calculada. Não é preciso instalar SciPy
para executar a simulação. Para modificar a geometria e recalcular a postura:

```bash
python3 -m pip install --user scipy
python3 ~/robotics_labs/lab1/golfista/golfista_v2/preparar_modelo.py
```

O gerador usa least_squares para alinhar os sites dos pés com o chão e os
sites das mãos com uma pega prescrita, respeitando os limites articulares.
O taco é orientado através da mão esquerda. A pega direita é fechada depois
de encontrar a postura compatível. Alterações diretas no XML são substituídas
se o gerador voltar a ser executado.

## Limitações a manter explícitas

- Segmentos rígidos e geometrias simples; massas, limites e ganhos iniciais
  estimados, sem calibração antropométrica ou biomecânica.
- Pega idealizada rígida, sem dedos nem deslizamento.
- Auto-colisões do corpo desativadas nesta etapa: movimentos futuros devem
  ser revistos para impedir atravessamentos. Isto não foi validado para swing.
- Colisões ativas: pés–chão, taco–chão, taco–bola e bola–chão.
- Contactos complacentes e pequenas penetrações numéricas são esperados.
- Atrito da bola estimado; sem aerodinâmica nem deformação física detalhada.
- Critérios testados apenas na postura nominal durante 10 s. Estabilidade
  face a perturbações, levantamento do calcanhar, swing e impacto estão pendentes.

`preview.png` mostra a simulação após 2 segundos com o controlador ativo.
