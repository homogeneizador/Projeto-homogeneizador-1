#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>

// --- CONFIGURAÇÕES DE REDE ---
const char* ssid = "t";
const char* pass = "9";
const char* mqtt_server = "test.mosquitto.org";

// --- PINOS ---
#define STEP_PIN 12
#define DIR_PIN  14
#define EN_PIN   13 
#define ENC_CLK  26
#define ENC_DT   27
#define ENC_SW   25 

AccelStepper stepper(1, STEP_PIN, DIR_PIN);
WiFiClient espClient;
PubSubClient client(espClient);

// --- VARIÁVEIS DE CONTROLE ---
long limiteBase = 8000; // Valor inicial (ajustaremos no fuso real)
int ultimoCLK;
unsigned long ultimoDebounceSW = 0;
unsigned long ultimoGiro = 0;
const int intervaloDebounce = 5;
int ultimoEstadoCLK;

void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  
  if (msg == "SUBIR") {
    stepper.moveTo(0);
    Serial.println("Comando: Indo para o TOPO");
  } 
  else if (msg == "DESCER") {
    stepper.moveTo(limiteBase);
    Serial.println("Comando: Indo para a BASE");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT);
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  digitalWrite(EN_PIN, LOW); // Ativa o motor
  ultimoEstadoCLK = digitalRead(ENC_CLK);
  
  // Configurações para Fonte de 1A (Suave)
  stepper.setMaxSpeed(400); 
  stepper.setAcceleration(200);
  
  ultimoCLK = digitalRead(ENC_CLK);

  WiFi.begin(ssid, pass);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
unsigned long ultimaTentativaMQTT = 0;

void reconnect() {
  if (millis() - ultimaTentativaMQTT > 5000) { 
    ultimaTentativaMQTT = millis();
    Serial.print("Tentando reconectar ao MQTT...");
    
    // Tenta conectar sem travar o código
    if (client.connect("ESP32_NEMA_Z_Axis")) {
      client.subscribe(/nema/posicao");
      Serial.println("Conectado com sucesso!");
    } else {
      Serial.println(" Falhou. Próxima tentativa em 5s. Controle local segue ativo.");
    }
  }
}

void loop() {
  if (!client.connected()) reconnect();
  else client.loop();

  // --- CONTROLE MANUAL COM FILTRO DE AMOSTRAGEM ---
  int leituraCLK = digitalRead(ENC_CLK);

  if (leituraCLK != ultimoEstadoCLK) {
    // FILTRO DE CONFIANÇA: Lemos o pino DT 10 vezes
    int contaHigh = 0;
    for (int i = 0; i < 10; i++) {
      if (digitalRead(ENC_DT) == HIGH) contaHigh++;
      delayMicroseconds(50); // Pequena pausa entre amostras
    }

    // Se o DT foi HIGH na maioria das amostras, consideramos HIGH
    int estadoDT = (contaHigh > 5) ? HIGH : LOW;

    if (estadoDT != leituraCLK) {
      // Sentido Horário -> DESCER
      long novoAlvo = stepper.currentPosition() + 200; 
      if (novoAlvo <= limiteBase) stepper.moveTo(novoAlvo);
    } 
    else {
      // Sentido Anti-horário -> SUBIR
      long novoAlvo = stepper.currentPosition() - 200;
      if (novoAlvo >= 0) stepper.moveTo(novoAlvo);
    }
    
    delay(5); // Delay de estabilização mecânica aumentado
  }
  
  ultimoEstadoCLK = leituraCLK;

  // --- BOTÃO RESET (SET HOME) ---
  if (digitalRead(ENC_SW) == LOW && (millis() - ultimoDebounceSW > 500)) {
    stepper.setCurrentPosition(0);
    Serial.println(">>> TOPO DEFINIDO <<<");
    ultimoDebounceSW = millis();
  }

  stepper.run();
}
