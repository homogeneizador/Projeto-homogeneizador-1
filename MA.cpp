#define BLYNK_TEMPLATE_ID   "T"
#define BLYNK_TEMPLATE_NAME "Homogeneizador"
#define BLYNK_AUTH_TOKEN    "P"

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h>
#include <AccelStepper.h>
#include <PID_v1.h>

// --- CONFIGURAÇÕES DE HARDWARE (PINAGEM CONSOLIDADA) ---
#define MOTOR_PIN 25
#define FG_PIN 18
#define ENC_CLK 26
#define ENC_DT  27
#define ENC_SW  14
#define PWM_CHANNEL 0

#define PWM_CHANNEL 0
#define PWM_FREQ    1000  // Frequência de 1kHz é ideal para o motor brushless
#define PWM_RES     8     // Resolução de 8 bits (0 a 255)

// PINOS DO NEMA (Ajustados para não conflitar)
#define NEMA_STEP 16
#define NEMA_DIR  13
#define NEMA_EN   15
#define NEMA_ENC_CLK 32
#define NEMA_ENC_DT  33

// --- OBJETOS E VARIÁVEIS ---
U8G2_SH1107_PIMORONI_128X128_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
AccelStepper stepper(1, NEMA_STEP, NEMA_DIR);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

char ssid[] = ""
char pass[] = ""
const char* mqtt_server = "test.mosquitto.org";

// --- VARIÁVEIS VOLATILE (PARA COMPARTILHAMENTO ENTRE CORES) ---
volatile int valor_pwm = 0;
volatile bool travaEmergencia = false;
volatile long alvoNema = 0;
volatile int16_t encoderAcumulado = 0;
volatile int ultimoEstadoCLK;
volatile int ultimoEstadoCLK_Misturador;
volatile int ultimoEstadoCLK_Nema;
volatile bool nemaPrecisaMover = false;

// --- VARIÁVEIS DE TELEMETRIA E TIMER (ATUALIZADA) ---
float temperatura = 25.0;
bool cronometroRodando = false; 

// Mudamos para 'long' para evitar erros matemáticos na contagem regressiva
// Iniciamos com 600 segundos (10 minutos) como você pediu
volatile long tempoRestanteSegundos = 600; 

unsigned long tempoAnteriorSerial = 0;
unsigned long tempoAnteriorBlynk = 0;
int ultimoPWMEnviado = -1;
unsigned long ultimoTempoEnviado = 0;
volatile bool precisaAtualizarOLED = false;
volatile unsigned long contadorPulsosFG = 0;

// --- VARIÁVEIS DO PID ---
double Setpoint, Input, Output;
// Constantes Kp, Ki, Kd (Precisarão de ajuste fino/Tuning)
// No topo, mude as constantes para 0 para garantir que o PID esteja "morto"
double Kp=0, Ki=0, Kd=0; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ================================================================
// CORE 0: MOTORES (MALHA ABERTA PARA LEVANTAMENTO DE CURVA)
// ================================================================
void TaskMotores(void * pvParameters) {

  const int PULSOS_POR_VOLTA = 2;   
  const int TEMPO_MEDICAO_MS = 100; 
  static unsigned long ultimoTempoRPM = 0;

  for(;;) {
    // 1. Controle do Nema 17
    if (!travaEmergencia) {
        stepper.run(); 
    } else {
        stepper.stop();
    }

    // 2. Controle do Misturador (Malha Aberta Pura)
    if (!travaEmergencia && valor_pwm >= 10) {

      // --- CÁLCULO DO RPM REAL ---
      unsigned long agora = millis();
      if (agora - ultimoTempoRPM >= TEMPO_MEDICAO_MS) {
        ultimoTempoRPM = agora;

        noInterrupts();
        unsigned long pulsos = contadorPulsosFG;
        contadorPulsosFG = 0;
        interrupts();

        double tempoSeg = (double)TEMPO_MEDICAO_MS / 1000.0;
        Input = (pulsos / tempoSeg) * (60.0 / PULSOS_POR_VOLTA);
      }

      // --- COMANDO DIRETO (A correção está aqui) ---
      // Em vez de usar 'Output' (que vem do PID zerado), usamos 'valor_pwm'
      ledcWrite(PWM_CHANNEL, valor_pwm); 

    } else {
      ledcWrite(PWM_CHANNEL, 0);
      Input = 0;
    }

    vTaskDelay(2 / portTICK_PERIOD_MS); 
  }
}

// ================================================================
// CORE 1: TAREFA DE REDE, OLED E ENCODER
// ================================================================

// --- V1: SLIDER DE VELOCIDADE ---
BLYNK_WRITE(V1) {
    int rpmRecebido = param.asInt(); 
    int novo_pwm = map(rpmRecebido, 0, 6400, 0, 255);

    if (novo_pwm != valor_pwm) {
        valor_pwm = novo_pwm;
        precisaAtualizarOLED = true;
    }
}

BLYNK_WRITE(V3) {
    bool estadoBotao = (param.asInt() == 1);
    if (estadoBotao) {
        travaEmergencia = true;
        valor_pwm = 0;
        ledcWrite(PWM_CHANNEL, 0);
        Serial.println("!!! EMERGÊNCIA ATIVA !!!");
    } else {
        travaEmergencia = false;
        ledcWrite(PWM_CHANNEL, 0);
        Serial.println("Sistema liberado.");
    }
    precisaAtualizarOLED = true;
}

// --- V4: START/STOP ---
BLYNK_WRITE(V4) {
    int estadoBotaoApp = param.asInt(); 
    
    if (estadoBotaoApp == 1) {
        if (tempoRestanteSegundos > 0) {
            cronometroRodando = true;
            Serial.println("Cronometro: RODANDO");
        } else {
            cronometroRodando = false;
            Blynk.virtualWrite(V4, 0); 
            Serial.println("Aviso: Adicione tempo antes de iniciar!");
        }
    } else {
        cronometroRodando = false;
        Serial.println("Cronometro: PAUSADO");
    }
    precisaAtualizarOLED = true;
}

// --- V10: Slider de Tempo (0 a 10 min) ---
BLYNK_WRITE(V10) {
    tempoRestanteSegundos = param.asInt() * 60; 
    precisaAtualizarOLED = true;
}

// --- V11, V12, V13: Botões de Soma ---
BLYNK_WRITE(V11) { if(param.asInt()) { tempoRestanteSegundos += 60;  precisaAtualizarOLED = true; } }
BLYNK_WRITE(V12) { if(param.asInt()) { tempoRestanteSegundos += 300; precisaAtualizarOLED = true; } }
BLYNK_WRITE(V13) { if(param.asInt()) { tempoRestanteSegundos += 600; precisaAtualizarOLED = true; } }

// 1. INTERRUPÇÃO DO MISTURADOR
void IRAM_ATTR tratarEncoderMisturador() {
    int sCLK = digitalRead(ENC_CLK);
    int sDT  = digitalRead(ENC_DT);
    
    if (sCLK != ultimoEstadoCLK_Misturador) {
        if (sDT != sCLK) encoderAcumulado++; 
        else encoderAcumulado--;
    }
    ultimoEstadoCLK_Misturador = sCLK;
}

// 2. INTERRUPÇÃO DO NEMA
void IRAM_ATTR tratarEncoderNema() {
    int sCLK = digitalRead(NEMA_ENC_CLK);
    int sDT  = digitalRead(NEMA_ENC_DT);
    
    if (sCLK != ultimoEstadoCLK_Nema) {
        if (sDT != sCLK) alvoNema += 100; 
        else alvoNema -= 100;
        
        nemaPrecisaMover = true;
    }
    ultimoEstadoCLK_Nema = sCLK;
}

void atualizarTelaOLED() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "HOMOGENEIZADOR");
    u8g2.drawHLine(0, 15, 128);

    u8g2.setFont(u8g2_font_helvB14_tf);
    u8g2.setCursor(0, 45);
    
    int m = tempoRestanteSegundos / 60;
    int s = tempoRestanteSegundos % 60;
    u8g2.print("Tempo: ");
    if (m < 10) u8g2.print("0"); u8g2.print(m);
    u8g2.print(":");
    if (s < 10) u8g2.print("0"); u8g2.print(s);

    u8g2.setCursor(0, 80);
    int rpm_preview = map(valor_pwm, 0, 255, 0, 6400);
    u8g2.print("RPM: "); u8g2.print(rpm_preview);

    u8g2.drawFrame(0, 110, 120, 10);
    int barWidth = map(constrain(tempoRestanteSegundos, 0, 600), 0, 600, 0, 118);
    u8g2.drawBox(1, 111, barWidth, 8);
    u8g2.sendBuffer();
}

void IRAM_ATTR contarFG() {
    contadorPulsosFG++;
}

void setup() {
    Serial.begin(115200);
    
    u8g2.begin();
    u8g2.setBusClock(400000);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 50, "Iniciando Sistema...");
    u8g2.sendBuffer();

    // Brushless (Misturador)
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PIN, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0); 

    // PID CONFIG FIXA (NÃO MUDA MAIS NO LOOP)
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(50);      // igual janela RPM (50ms)
    myPID.SetOutputLimits(0, 255);

    // Nema 17
    pinMode(NEMA_EN, OUTPUT);
    digitalWrite(NEMA_EN, LOW);
    stepper.setMaxSpeed(5000);
    stepper.setAcceleration(4000);

    // Encoder 1: Misturador
    pinMode(ENC_CLK, INPUT_PULLUP);
    pinMode(ENC_DT, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);
    ultimoEstadoCLK_Misturador = digitalRead(ENC_CLK);
    attachInterrupt(digitalPinToInterrupt(ENC_CLK), tratarEncoderMisturador, CHANGE);

    // Encoder 2: Nema
    pinMode(NEMA_ENC_CLK, INPUT_PULLUP);
    pinMode(NEMA_ENC_DT, INPUT_PULLUP);
    ultimoEstadoCLK_Nema = digitalRead(NEMA_ENC_CLK);
    attachInterrupt(digitalPinToInterrupt(NEMA_ENC_CLK), tratarEncoderNema, CHANGE);

    // FG Brushless
    pinMode(FG_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FG_PIN), contarFG, FALLING);

    WiFi.begin(ssid, pass);
    Blynk.config(BLYNK_AUTH_TOKEN); 
    mqttClient.setServer(mqtt_server, 1883);

    xTaskCreatePinnedToCore(
        TaskMotores,
        "TaskMotores",
        10000,
        NULL,
        1,
        NULL,
        0
    );

    atualizarTelaOLED();
    Serial.println("Sistema Dual Core Pronto!");
}

void reconnectMQTT() {
    if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
        Serial.print("Tentando conexão MQTT...");
        String clientId = "ESP32-Homog-" + String(random(0xffff), HEX);
        
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("conectado!");
        } else {
            Serial.println("falhou, tentando no próximo ciclo.");
        }
    }
}

void gerenciarBotaoCronometro() {
    static bool ultimoEstadoBotao = HIGH;
    bool estadoAtualBotao = digitalRead(ENC_SW);

    if (ultimoEstadoBotao == HIGH && estadoAtualBotao == LOW) {
        delay(50); 

        if (tempoRestanteSegundos <= 0) {
            tempoRestanteSegundos = 600; 
            if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V10, 10); 
        }

        cronometroRodando = !cronometroRodando; 
        
        if (WiFi.status() == WL_CONNECTED) {
            Blynk.virtualWrite(V4, cronometroRodando ? 1 : 0); 
        }
        
        precisaAtualizarOLED = true;
    }
    ultimoEstadoBotao = estadoAtualBotao;
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        Blynk.run();
        mqttClient.loop();
    }
    gerenciarBotaoCronometro(); 

    if (encoderAcumulado != 0 && !travaEmergencia) {
        noInterrupts();
        int passos = encoderAcumulado;
        encoderAcumulado = 0;
        interrupts();

        valor_pwm += (passos * 5);
        valor_pwm = constrain(valor_pwm, 0, 255);
        precisaAtualizarOLED = true;
    }

    if (nemaPrecisaMover) {
        stepper.moveTo(alvoNema); 
        nemaPrecisaMover = false;
    }

    if (precisaAtualizarOLED) {
        atualizarTelaOLED();
        precisaAtualizarOLED = false;
    }

    unsigned long tempoAtual = millis();
    if (tempoAtual - tempoAnteriorBlynk >= 1000) {
        tempoAnteriorBlynk = tempoAtual;

        if (cronometroRodando && tempoRestanteSegundos > 0 && valor_pwm > 0) {
            tempoRestanteSegundos--;
            precisaAtualizarOLED = true;

            if (tempoRestanteSegundos <= 0) {
                tempoRestanteSegundos = 0;
                valor_pwm = 0;
                cronometroRodando = false;
                if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V4, 0); 
            }
        }

        if (WiFi.status() == WL_CONNECTED) {
            
            if (valor_pwm != ultimoPWMEnviado) {
                int rpm_para_slider = map(valor_pwm, 0, 255, 0, 6400);
                Blynk.virtualWrite(V1, rpm_para_slider); 
                Blynk.virtualWrite(V2, rpm_para_slider);
                ultimoPWMEnviado = valor_pwm;
            }
            
            if (tempoRestanteSegundos != ultimoTempoEnviado) {
                int minutos = tempoRestanteSegundos / 60;
                int segundos = tempoRestanteSegundos % 60;
                char tempoFormatado[10];
                sprintf(tempoFormatado, "%02d:%02d", minutos, segundos);

                Blynk.virtualWrite(V0, tempoFormatado); 
                ultimoTempoEnviado = tempoRestanteSegundos;
            }
        }

        if (mqttClient.connected()) {
            StaticJsonDocument<128> mqttDoc;
            mqttDoc["tempo"] = tempoRestanteSegundos;
            mqttDoc["pwm"] = valor_pwm;
            mqttDoc["status"] = travaEmergencia ? "ALERTA" : (cronometroRodando ? "RODANDO" : "STOP");
            mqttDoc["z_pos"] = stepper.currentPosition();

            char buffer[128];
            serializeJson(mqttDoc, buffer);
            mqttClient.publish("ifsudestemg/homogeneizador/dados", buffer);
        }
    }

    if (tempoAtual - tempoAnteriorSerial >= 500) { 
        tempoAnteriorSerial = tempoAtual;
        Serial.print("> PWM: "); Serial.print(valor_pwm);
        Serial.print(" | RPM Real: "); Serial.print(Input);
        Serial.print(" | Tempo: "); Serial.print(tempoRestanteSegundos);
        Serial.print(" | Z: "); Serial.println(stepper.currentPosition());
    }
}
