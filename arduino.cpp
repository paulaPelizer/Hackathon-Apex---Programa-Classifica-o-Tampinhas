#include <Servo.h>

// --- Pinos ---
const int LED_BRANCO     = 8;
const int SERVO_SORT_PIN = 9;   // servo principal (seleção)
const int SERVO_GATE_PIN = 10;  // servo de abertura/comporta

Servo meuServo;        // servo de seleção
Servo servoAbertura;   // servo da abertura
int ultimaPosicao = -1;

// Lista de comandos simulados (como se viessem do Python)
String comandos[] = {
  "SERVO:0;COLOR:vermelho",
  "SERVO:45;COLOR:verde",
  "SERVO:90;COLOR:amarelo",
  "SERVO:135;COLOR:azul",
  "SERVO:180;COLOR:rosa",
  "SERVO:90;COLOR:amarelo",
  "SERVO:0;COLOR:vermelho"
};
const int totalComandos = sizeof(comandos) / sizeof(comandos[0]);
int indiceComando = 0;

// se quiser ciclar para sempre, mude para true
const bool LOOP_COMANDOS = false;

void setup() {
  pinMode(LED_BRANCO, OUTPUT);
  digitalWrite(LED_BRANCO, HIGH);

  meuServo.attach(SERVO_SORT_PIN);
  servoAbertura.attach(SERVO_GATE_PIN);

  meuServo.write(0);
  servoAbertura.write(0); // começa fechado

  Serial.begin(115200);
  delay(300);
  Serial.println(F("Simulação iniciada (dois servos, comandos internos)."));
}

void loop() {
  if (indiceComando < totalComandos) {
    String comando = comandos[indiceComando];
    comando.trim();

    if (comando.startsWith("SERVO:")) {
      int colon = comando.indexOf(':');
      int semicolon = comando.indexOf(';', colon + 1);

      String angStr;
      if (colon >= 0) {
        if (semicolon > colon) angStr = comando.substring(colon + 1, semicolon);
        else angStr = comando.substring(colon + 1);
        angStr.trim();
      }
      int angulo = constrain(angStr.toInt(), 0, 180);

      if (angulo != ultimaPosicao) {
        // move servo principal
        meuServo.write(angulo);
        ultimaPosicao = angulo;
        Serial.print(F("Executado -> "));
        Serial.println(comando);

        delay(1000); // tempo para posicionar

        // aciona servo de abertura
        servoAbertura.write(90); // abre
        Serial.println("Abertura acionada.");
        delay(1000); // tempo para liberar tampinha
        servoAbertura.write(0); // fecha
        Serial.println("Abertura fechada.");

        delay(1500); // espera antes do próximo comando
      }
    }

    indiceComando++;
    delay(2000); // espera 2s antes do próximo comando
  } else {
    if (LOOP_COMANDOS) {
      indiceComando = 0;  // recomeça a sequência
    } else {
      while (true) { delay(1000); } // fim da simulação
    }
  }
}
