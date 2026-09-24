# Golfista: swing com motores e dinâmica

Executado e testado com Python 3 e MuJoCo 3.13.0. Dependências: numpy, scipy e mujoco.

## Executar

```bash
python3 -m pip install --user scipy
python3 ~/robotics_labs/lab1/golfista_dinamico/simular.py
```

Espera cerca de quatro segundos pela preparação e início visível do swing. Espaço pausa, R reinicia e M desliga/liga os motores. No final, mantém a postura. Usa este programa para carregar a trajetória e o controlador; abrir apenas o XML não executa o swing.

## Cinemática e motores

Na reprodução cinemática, o programa impõe diretamente a posição de cada articulação em cada instante. A pose aparece mesmo que exigisse esforços impossíveis.

Nesta versão, a trajetória fornece alvos aos 29 atuadores de posição. Cada atuador produz binário segundo tau = kp*(alvo - posição) - kv*velocidade, limitado pelo seu intervalo de força. MuJoCo integra o movimento com gravidade, inércia e contactos. Há também uma correção dos alvos dos tornozelos para equilibrar o corpo relativamente aos apoios. A bacia é livre: não há forças externas de sustentação nem fixação dos pés ao mundo. Estes atuadores aproximam o esforço muscular; não são um modelo de músculos e tendões.

As posições do corpo simulado só são impostas na inicialização e no reinício explícito. Durante o movimento, o controlador altera os comandos dos motores e chama mj_step. A segunda mão está ligada ao taco por uma restrição de pega com alguma flexibilidade numérica.

## Validação incluída

```bash
python3 ~/robotics_labs/lab1/golfista_dinamico/simular.py --testar
```

O teste sem janela grava resultado_dinamico.json e execucao_dinamica.npz. O pacote inclui os resultados da execução nominal: 17,176 segundos, incluindo cinco segundos de manutenção final.

- Sem contactos entre braços e cabeça; folga mínima de 13,24 mm.
- Erro máximo de posição da pega: 1,01 mm; orientação: 2,04 graus.
- Erro articular RMS global: 2,45 graus. Erro instantâneo individual máximo: 21,05 graus no pulso direito.
- Binários dentro dos limites; algum motor saturado em cerca de 1,01% dos passos.
- Limites articulares suaves excedidos no máximo em 0,94 graus.
- Deslocamento horizontal máximo do apoio esquerdo: 2,95 cm.
- Sem avisos numéricos do MuJoCo e sem queda na execução nominal.

## Limites desta etapa

O movimento demora três vezes o tempo da captura original. Os apoios foram adaptados para pés rígidos: pé esquerdo plano e elevação tardia do calcanhar direito. A referência também foi ajustada para afastar os braços da cabeça. A geometria e os parâmetros corporais são aproximações.

Este teste não valida robustez a perturbações, todas as outras auto-colisões, fidelidade biomecânica ou execução à velocidade original. Ainda não há impacto validado na bola. Alterar --fator-tempo muda as exigências dinâmicas e requer nova validação.

## Ficheiros

- simular.py: controlador, integração física, visualização e teste.
- golfista.xml: corpo articulado, actuadores, contactos e pega.
- trajetoria.npz: referência já preparada; não é necessário recalcular.
- adaptar_referencia.py: recalcula a referência por otimização com afastamento da cabeça e adaptação dos apoios.
- captura_cmu_64_01.npz e proveniencia.json: captura de origem e proveniência.
- ver_referencia.py: visualizador cinemático auxiliar, distinto da simulação por motores.
- execucao_dinamica.npz: estados efetivamente simulados; amostras contém tempo, qpos e forças dos atuadores, nessa ordem; nq indica o número de coordenadas de posição.

Referência técnica: https://mujoco.readthedocs.io/en/stable/computation/index.html
