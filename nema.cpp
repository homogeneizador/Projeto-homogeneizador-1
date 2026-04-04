#include <AccelStepper.h>

// --- PINOS DO MOTOR NEMA ---
#define STEP_PIN 12
#define DIR_PIN  14
#define EN_PIN   13 

// --- PINOS DO KY-040 (ENCODER) ---
#define ENC_CLK  26
#define ENC_DT   27
#define ENC_SW   25 // Botão do encoder

AccelStepper stepper(1, STEP_PIN, DIR_PIN);

// --- LIMITES DE SEGURANÇA ---
long limiteBase = 5000; // Ajuste após medir o curso do seu fuso
int ultimoCLK;

void setup() {
  Serial.begin(115200);
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  digitalWrite(EN_PIN, LOW); // Motor travado para segurar o peso
  
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  
  ultimoCLK = digitalRead(ENC_CLK);
  
  Serial.println("Sistema Pronto. Posicione no topo e aperte o botão do Encoder.");
}

void loop() {
  // 1. LEITURA DO GIRO (Subida/Descida Manual)
  int leituraCLK = digitalRead(ENC_CLK);
  if (leituraCLK != ultimoCLK) {
    if (digitalRead(ENC_DT) != leituraCLK) {
      // Girou para um lado -> Tenta Descer
      long alvo = stepper.currentPosition() + 100; 
      if (alvo <= limiteBase) stepper.moveTo(alvo);
    } else {
      // Girou para o outro -> Tenta Subir
      long alvo = stepper.currentPosition() - 100;
      if (alvo >= 0) stepper.moveTo(alvo);
    }
  }
  ultimoCLK = leituraCLK;

  // 2. BOTÃO DE RESET (SET HOME)
  if (digitalRead(ENC_SW) == LOW) {
    delay(200); // Debounce
    stepper.setCurrentPosition(0); // Define aqui como o TOPO (0)
    Serial.println(">>> TOPO CONFIRMADO (ZERO)");
    // Pisca um LED ou envia aviso para o Blynk se desejar
  }

  stepper.run();
}
