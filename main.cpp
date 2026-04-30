#define BLYNK_TEMPLATE_ID   ""
#define BLYNK_TEMPLATE_NAME "Homogeneizador"
#define BLYNK_AUTH_TOKEN    ""

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

#define PWM_FREQ    20000  // Frequência de 1kHz é ideal para o motor brushless
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

char ssid[] = ""; //"";
char pass[] = ""; //"";
const char* mqtt_server = "test.mosquitto.org";

// --- PROTÓTIPOS DE FUNÇÕES ---
void reconnectMQTT();
void atualizarTelaOLED();
void gerenciarBotaoCronometro();

// --- VARIÁVEIS VOLATILE (PARA COMPARTILHAMENTO ENTRE CORES) ---
volatile int valor_pwm = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
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
static double setpointSuave = 0;
float rpmExibicao = 0;


// --- VARIÁVEIS DO PID ---
double Setpoint, Input, Output;
// Constantes Kp, Ki, Kd (Precisarão de ajuste fino/Tuning)
// No topo, mude as constantes para 0 para garantir que o PID esteja "morto"
// Ajuste Kp para controlar a resposta geral, Ki para eliminar o erro estacionário, e Kd para suavizar a resposta.
// kd faz o motor balançar muito, qndo aumenta, em 3k rpm
double Kp=0.0, Ki=0.0, Kd=0.0; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ================================================================
// CORE 0: MOTORES (MALHA FECHADA)
// ================================================================
void TaskMotores(void * pvParameters) {
    const int PULSOS_POR_VOLTA = 6;
    const int TEMPO_MEDICAO_MS = 50; 
    static unsigned long ultimoTempoRPM = 0;
    static bool motorEstavaParado = true;

    for(;;) {
        if (!travaEmergencia) stepper.run();
        else stepper.stop();

        if (!travaEmergencia && valor_pwm >= 10) {
            
            if (motorEstavaParado) {
                myPID.SetMode(MANUAL);
                Output = 0;
                myPID.SetMode(AUTOMATIC);
                motorEstavaParado = false;
                Input = 0; 
            }

            unsigned long agora = millis();
            if (agora - ultimoTempoRPM >= TEMPO_MEDICAO_MS) {
                ultimoTempoRPM = agora;

                // --- PROTEÇÃO DO FG (Freq. Generator) ---
                portENTER_CRITICAL(&mux); 
                unsigned long pulsos = contadorPulsosFG;
                contadorPulsosFG = 0;
                portEXIT_CRITICAL(&mux); 

                double tempoSeg = (double)TEMPO_MEDICAO_MS / 1000.0;
                double rpmBruto = (pulsos / tempoSeg) * (60.0 / (double)PULSOS_POR_VOLTA);

                Input = (Input * 0.90) + (rpmBruto * 0.10);
                rpmExibicao = (Input * 0.92) + (rpmBruto * 0.08);
            }

            double alvoFinal = map(valor_pwm, 0, 255, 0, 5000);

            if (setpointSuave < alvoFinal) {
                setpointSuave += 2; 
                if (setpointSuave > alvoFinal) setpointSuave = alvoFinal;
            } else if (setpointSuave > alvoFinal) {
                setpointSuave -= 2;
                if (setpointSuave < alvoFinal) setpointSuave = alvoFinal;
            }

            Setpoint = setpointSuave;
            
            static bool modoBaixaRPM = false;
            if (Setpoint < 3500 && !modoBaixaRPM) {
                myPID.SetTunings(0.12, 0.01, 0.005); 
                modoBaixaRPM = true;
            } else if (Setpoint >= 3500 && modoBaixaRPM) {
                myPID.SetTunings(0.30, 0.08, 0.001); 
                modoBaixaRPM = false;
            }
            
            myPID.Compute();
            
            int pwmFinal = (int)Output; 
            if (valor_pwm < 10) {
                ledcWrite(PWM_CHANNEL, 0); 
                motorEstavaParado = true;
                setpointSuave = 0;
            } else {
                ledcWrite(PWM_CHANNEL, constrain(pwmFinal, 25, 255));
            }

        } else {
            ledcWrite(PWM_CHANNEL, 0); 
            motorEstavaParado = true;
            Input = 0;
            Output = 0;
            setpointSuave = 0;
        }

        // --- PROTEÇÃO DO ENCODER (KY-040) ---
        if (encoderAcumulado != 0 && !travaEmergencia) {
            portENTER_CRITICAL(&mux);
            int passos = encoderAcumulado;
            encoderAcumulado = 0;
            portEXIT_CRITICAL(&mux);

            valor_pwm = constrain(valor_pwm + (passos * 5), 0, 255);
            Setpoint = map(valor_pwm, 0, 255, 0, 5000);
            precisaAtualizarOLED = true;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ================================================================
// CORE 1: TAREFA DE REDE, OLED E ENCODER
// ================================================================

// --- V1: SLIDER DE VELOCIDADE ---
BLYNK_WRITE(V1) {
    int percentual = param.asInt(); 
    int novo_pwm = map(percentual, 0, 100, 0, 255);

    if (novo_pwm != valor_pwm) {
        valor_pwm = novo_pwm;
        Setpoint = map(percentual, 0, 100, 0, 5000);
        precisaAtualizarOLED = true;
    }
}

BLYNK_WRITE(V3) {
    bool estadoBotao = (param.asInt() == 1);
    if (estadoBotao) {
        travaEmergencia = true;
        valor_pwm = 0;
        ledcWrite(PWM_CHANNEL, 255);
        Serial.println("!!! EMERGÊNCIA ATIVA !!!");
    } else {
        travaEmergencia = false;
        ledcWrite(PWM_CHANNEL, 255);
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
    
    // Cabeçalho
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "HOMOGENEIZADOR");
    u8g2.drawHLine(0, 15, 128);

    // Exibição do Cronômetro
    u8g2.setFont(u8g2_font_helvB14_tf);
    u8g2.setCursor(0, 45);
    
    int m = tempoRestanteSegundos / 60;
    int s = tempoRestanteSegundos % 60;
    u8g2.print("Tempo: "); // Abreviei para sobrar espaço para os números
    if (m < 10) u8g2.print("0"); u8g2.print(m);
    u8g2.print(":");
    if (s < 10) u8g2.print("0"); u8g2.print(s);

    // Exibição do RPM REAL (O que vem do sensor FG)
    u8g2.setCursor(0, 80);
    u8g2.print("RPM: "); 
    
    // Usamos o rpmExibicao que é calculado no Core 0
    // Adicionamos uma trava visual: se o PWM for 0, forçamos o visor a mostrar 0
    if (valor_pwm < 10) {
        u8g2.print("0");
    } else {
        u8g2.print((int)rpmExibicao);
    }

    // Barra de Progresso do Tempo (Visual)
    // Moldura da barra
    u8g2.drawFrame(0, 110, 120, 10);
    // Preenchimento proporcional aos 600 segundos (10 min)
    int barWidth = map(constrain(tempoRestanteSegundos, 0, 600), 0, 600, 0, 118);
    u8g2.drawBox(1, 111, barWidth, 8);
    
    u8g2.sendBuffer();
}

void IRAM_ATTR contarFG() {
    static unsigned long ultimoMicros = 0;
    unsigned long agoraMicros = micros();
    
    // Ignora pulsos com intervalo menor que 500 microssegundos (filtra ruídos acima de 2kHz)
    if (agoraMicros - ultimoMicros > 1200) { 
        contadorPulsosFG++;
        ultimoMicros = agoraMicros;
    }
}

void TaskInternet(void * pvParameters) {
    static int ultimoMinutoEnviado = -1;
    static int ultimoRPMEnviadoApp = -1;
    static int ultimoPercentualEnviado = -1;

    for(;;) {
        if (WiFi.status() == WL_CONNECTED) {
            Blynk.run();
            mqttClient.loop();

            // 1. ENVIO DO TEMPO (Econômico: apenas quando muda o minuto)
            // Arredondamos para cima: 9min e 1s aparece como 10min no App
            int minutosRestantes = (tempoRestanteSegundos + 59) / 60; 
            if (tempoRestanteSegundos == 0) minutosRestantes = 0; // Garante o zero no fim

            if (minutosRestantes != ultimoMinutoEnviado) {
                Blynk.virtualWrite(V0, minutosRestantes);
                ultimoMinutoEnviado = minutosRestantes;
            }

            // 2. FEEDBACK DO SLIDER (0-100%)
            // Sincroniza o Slider do celular se você girar o botão físico (KY-040)
            int percentualAtual = map(valor_pwm, 0, 255, 0, 100);
            if (percentualAtual != ultimoPercentualEnviado) {
                Blynk.virtualWrite(V1, percentualAtual);
                ultimoPercentualEnviado = percentualAtual;
            }

            // 3. ENVIO DO RPM REAL (Com Histerese de 15 RPM para evitar spam)
            int rpmAtualInt = (int)rpmExibicao;
            if (abs(rpmAtualInt - ultimoRPMEnviadoApp) > 15) {
                Blynk.virtualWrite(V2, rpmAtualInt); 
                ultimoRPMEnviadoApp = rpmAtualInt;
            }

            // 4. MANUTENÇÃO MQTT
            if (!mqttClient.connected()) {
                reconnectMQTT();
            }
        }
        
        // Delay de 15ms é o "ponto doce" para manter o App responsivo 
        // sem estressar o processador ou a pilha do Wi-Fi
        vTaskDelay(pdMS_TO_TICKS(15)); 
    }
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
    pinMode(FG_PIN, INPUT);
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
    // Criar a tarefa da Internet no Core 1
    xTaskCreatePinnedToCore(TaskInternet, "TaskInternet", 10000, NULL, 1, NULL, 1);

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
  // 1. INTERFACE LOCAL: BOTÃO DO ENCODER (START/STOP)
  // Como o delay(50) do debounce está aqui, ele não afeta o PID que está no Core 0
  gerenciarBotaoCronometro(); 

  // 2. MOVIMENTAÇÃO DO NEMA 17 (EIXO Z)
  // O loop processa o pedido de movimento vindo da interrupção do encoder 2
  if (nemaPrecisaMover) {
    stepper.moveTo(alvoNema); 
    nemaPrecisaMover = false;
  }

  // 3. ATUALIZAÇÃO DA TELA OLED
  // Só redesenha se houver mudança (economiza processamento)
  if (precisaAtualizarOLED) {
    atualizarTelaOLED();
    precisaAtualizarOLED = false;
  }

  unsigned long tempoAtual = millis();
  
  // 4. TIMER DE 1 SEGUNDO: LÓGICA DO CRONÔMETRO
  // Rodando no loop para garantir que a contagem no OLED seja fluida
  if (tempoAtual - tempoAnteriorBlynk >= 500) {
    tempoAnteriorBlynk = tempoAtual;

    if (cronometroRodando && tempoRestanteSegundos > 0 && valor_pwm > 0) {
      tempoRestanteSegundos--;
      precisaAtualizarOLED = true;

      if (tempoRestanteSegundos <= 0) {
        tempoRestanteSegundos = 0;
        valor_pwm = 0;
        cronometroRodando = false;
        // A atualização para o Blynk será feita pela TaskInternet ao ler essa variável
      }
    }
  }

  // 5. MONITOR SERIAL (DEBUG LOCAL)
  // Essencial para validar o sistema sem depender da internet
  if (tempoAtual - tempoAnteriorSerial >= 500) { 
    tempoAnteriorSerial = tempoAtual;
    Serial.print("> PWM User: "); Serial.print(valor_pwm);
    Serial.print(" | Setpoint: "); Serial.print((int)Setpoint);
    Serial.print(" | RPM Real: "); Serial.print((int)rpmExibicao);
    Serial.print(" | Tempo: "); Serial.println(tempoRestanteSegundos);
    precisaAtualizarOLED = true;
  }
}
