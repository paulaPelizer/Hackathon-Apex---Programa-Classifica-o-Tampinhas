# Caps Reader - SINGLE SHOT + Serial

Este programa faz **uma única leitura** (cor + tamanho) e **envia um sinal** para abrir **servos** no ângulo configurado de acordo com a cor reconhecida. Além de enviar o serial para leitura no sistema de Hardware, os dados também são registrados em arquivo CSV (Solução provisória)

## Instalação
```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
# macOS/Linux
source .venv/bin/activate

pip install -r requirements.txt
```

## Uso
```bash
python main.py
```
- Coloque **uma tampinha** sobre **fundo escuro** dentro da **ROI (retângulo verde)**.
- Quando ficar **estável**, o sistema:
  1) identifica a **cor** e o **diâmetro (px)**;
  2) grava dados em `reads.csv`, incluindo **ângulo de abertura do servo** e **índice da cor** (1..5);
  3) envia `SERVO:<ângulo>;COLOR:<cor>\n` via **Serial** (precisa habilitar, pois esta é uma situação simulada) para o Arduino;
  4) mostra "LEITURA OK" e **finaliza**.

## Configurações principais (no topo do main.py)
- `SERIAL_ENABLED = True/False` — liga/desliga envio via porta serial
- `SERIAL_PORT = "COM3"` — ajuste para sua porta (Windows: COMx | Linux/macOS: /dev/ttyUSB0 ou /dev/ttyACM0)
- `ANGLE_MAP` — ângulos por cor (padrão: 0°, 45°, 90°, 135°, 180°)
- `COLOR_RANGES` — faixas HSV por cor (ajuste conforme iluminação) - Necessária calibragem inicial!

## Protocolo Serial sugerido para o Arduino
Receber linhas no formato:
```
SERVO:<angulo>;COLOR:<nome>\n
```
No Arduino, leia `Serial.readStringUntil('\n')`, parseie `SERVO:` e mova o servo com `myservo.write(angulo);`.

