#define BLYNK_TEMPLATE_ID   "_"
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

const char* mqtt_server = "test.mosquitto.org"; // Broker público gratuito
WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long ultimoEnvioMQTT = 0;

// --- CONFIGURAÇÕES WIFI ---
char ssid[] = "t" ; //"" // Substitua pelo nome da sua rede WiFi
char pass[] = "9" ; //"" // Substitua pela senha da sua rede WiFi

// --- HARDWARE ---
#define MOTOR_PIN 25
#define ENC_CLK   26
#define ENC_DT    27
#define ENC_SW    14
#define PWM_CHANNEL 0
#define PWM_FREQ    1000
#define PWM_RES     8
#define FG_PIN 18


U8G2_SH1107_PIMORONI_128X128_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// --- VARIÁVEIS ---
volatile int valor_pwm = 0;   
float temperatura = 25.0;     
int rpm_real = 0;
unsigned long tempoAnteriorSerial = 0;
unsigned long tempoAnteriorBlynk = 0;
volatile unsigned long ultimoDebounce = 0;
volatile int ultimoEstadoCLK;
volatile bool precisaAtualizarOLED = false;
volatile bool travaEmergencia = false; // Variável de trava de emergência
bool cronometroRodando = false;
unsigned long segundosTranscorridos = 0;
unsigned long milisAnteriorCronometro = 0;
int ultimoPWMEnviado = -1; // Para evitar envios redundantes ao Blynk
int ultimoRPMEnviado = -1; // Para evitar envios redundantes ao Blynk
unsigned long ultimoTempoEnviado = 0; // Para evitar envios redundantes ao Blynk

// SINCRONIZAÇÃO BLYNK (APP -> ESP32)

BLYNK_WRITE(V1) {
    int valorRecebido = param.asInt();
    if (valorRecebido != valor_pwm) {
        valor_pwm = valorRecebido;
        precisaAtualizarOLED = true;
    }
}

BLYNK_WRITE(V3) {
    bool estadoBotao = (param.asInt() == 1);
    if (estadoBotao) {
        travaEmergencia = true;
        valor_pwm = 0;
        ledcWrite(PWM_CHANNEL, 0); // 0 é parado no seu motor
        Serial.println("!!! EMERGÊNCIA ATIVA !!!");
    } else {
        travaEmergencia = false;
        ledcWrite(PWM_CHANNEL, 0); // Mantém parado ao destravar por segurança
        Serial.println("Sistema liberado.");
    }
    precisaAtualizarOLED = true;
}

// BOTÃO START/STOP NO BLYNK (V4)
BLYNK_WRITE(V4) {
    int estadoBotaoApp = param.asInt(); 
    
    // O Blynk manda 1 quando você liga o switch no app
    if (estadoBotaoApp == 1) {
        cronometroRodando = true;
        segundosTranscorridos = 0; // Zera ao iniciar
        Serial.println("Cronometro: LIGADO pelo App");
    } else {
        cronometroRodando = false;
        segundosTranscorridos = 0; // Zera ao parar (conforme sua lógica)
        Serial.println("Cronometro: DESLIGADO pelo App");
    }
    
    precisaAtualizarOLED = true;
}

// INTERRUPÇÃO DO ENCODER (MAIS LEVE)

volatile int16_t encoderAcumulado = 0; // Armazena os passos "crus"

void IRAM_ATTR tratarEncoder() {
    // Leitura ultra-rápida dos pinos
    int sCLK = digitalRead(ENC_CLK);
    int sDT  = digitalRead(ENC_DT);
    
    // Lógica de Quadratura: Compara o estado atual com o anterior
    if (sCLK != ultimoEstadoCLK) {
        // Se CLK mudou, verificamos a direção pelo DT
        if (sDT != sCLK) {
            encoderAcumulado++;
        } else {
            encoderAcumulado--;
        }
    }
    ultimoEstadoCLK = sCLK;
}

void atualizarTelaOLED() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "HOMOGENEIZADOR");
    u8g2.drawHLine(0, 15, 128);

    u8g2.setFont(u8g2_font_helvB14_tf);
    u8g2.setCursor(0, 45);
    
    int minutos = segundosTranscorridos / 60;
    int segundos = segundosTranscorridos % 60;

    u8g2.print("Tempo: ");
    if (minutos < 10) u8g2.print("0"); u8g2.print(minutos); 
    u8g2.print(":");
    if (segundos < 10) u8g2.print("0"); u8g2.print(segundos);

    u8g2.setCursor(0, 80);
    int rpm_preview = map(valor_pwm, 0, 255, 0, 6400);
    u8g2.print("RPM: "); u8g2.print(rpm_preview);

    u8g2.drawFrame(0, 110, 120, 10);
    int barWidth = map(valor_pwm, 0, 255, 0, 118);
    u8g2.drawBox(1, 111, barWidth, 8);
    u8g2.sendBuffer();
}

volatile unsigned long contadorPulsosFG = 0;

void IRAM_ATTR contarFG() {
    contadorPulsosFG++;
}

void setup() {
    Serial.begin(115200);
    
    // 1. Inicializa o OLED primeiro (Fundamental!)
    u8g2.begin();
    u8g2.setBusClock(400000);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 50, "Iniciando Sistema..."); // Texto neutro
    u8g2.sendBuffer();

    // 2. Configura o MQTT
    mqttClient.setServer(mqtt_server, 1883);
    
    // 3. Configura o PWM (Lógica Inversa: Motor começa parado)

    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PIN, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0); // Garante que o motor comece PARADO 
    
    // 4. Configura o Encoder
    pinMode(ENC_CLK, INPUT_PULLUP);
    pinMode(ENC_DT, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);
    ultimoEstadoCLK = digitalRead(ENC_CLK);
    attachInterrupt(digitalPinToInterrupt(ENC_CLK), tratarEncoder, CHANGE);

    // 5. Configura o Fio Amarelo (FG)
    pinMode(FG_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FG_PIN), contarFG, FALLING);

    // 6. Configura WiFi e Blynk de forma inteligente (TIMEOUT)
    Serial.println("Tentando conectar WiFi...");
    // Tenta conectar por 10 segundos. Se falhar, segue para o loop()
    WiFi.begin(ssid, pass);
    Blynk.config(BLYNK_AUTH_TOKEN); 
    
    // 7. Finaliza o setup e desenha a tela principal
    atualizarTelaOLED();
    Serial.println("Sistema Pronto!");
}

void reconnectMQTT() {
    // Tenta conectar apenas se o Wi-Fi estiver ok e não estiver conectado ao MQTT
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
        delay(50); // Debounce
        
        cronometroRodando = !cronometroRodando; // Inverte o estado
        
        if (cronometroRodando) {
            segundosTranscorridos = 0;
        } else {
            segundosTranscorridos = 0;
        }

        // --- O PULO DO GATO ---
        // Envia o novo estado para o V4 no Blynk para o botão mudar de cor/texto
        Blynk.virtualWrite(V4, cronometroRodando ? 1 : 0);
        
        precisaAtualizarOLED = true;
        Serial.print("Cronometro via Encoder: ");
        Serial.println(cronometroRodando ? "ON" : "OFF");
    }
    ultimoEstadoBotao = estadoAtualBotao;
}

void loop() {
    // 1. ATUALIZAÇÃO DE REDE
    if (WiFi.status() == WL_CONNECTED) {
        Blynk.run();
        mqttClient.loop();
    }
    // Tentativa de reconexão sem travar o motor
    static unsigned long ultimaTentativaReconexao = 0;
    if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
        if (millis() - ultimaTentativaReconexao > 5000) { // Tenta reconectar a cada 5 segundos
            reconnectMQTT();
            ultimaTentativaReconexao = millis();
        }
    }

    gerenciarBotaoCronometro(); // Verifica o estado do botão a cada loop

    // 2. FILTRO DE SEGURANÇA
    if (travaEmergencia) {
        valor_pwm = 0;
        encoderAcumulado = 0; 
    }
    if (!mqttClient.connected()) {
        reconnectMQTT();
    }
    mqttClient.loop();

    // 3. PROCESSAMENTO DO ENCODER 
    if (encoderAcumulado != 0 && !travaEmergencia) {
        noInterrupts();
        int passos = encoderAcumulado;
        encoderAcumulado = 0;
        interrupts();

        //if (abs(passos) > 1) valor_pwm += (passos * 10);
        //else valor_pwm += (passos * 2);
        valor_pwm += (passos * 5); // Ajuste a sensibilidade aqui (5 é um bom ponto de partida)

        valor_pwm = constrain(valor_pwm, 0, 255);
        precisaAtualizarOLED = true;
    }

    // 4. SAÍDA PARA O MOTOR
    if (travaEmergencia || valor_pwm == 0) {
        ledcWrite(PWM_CHANNEL, 0); 
    } else {
        // Mapeia 1-255 do encoder para 10-255 do PWM (evita zumbido em baixa)
        int pwm_direto = map(valor_pwm, 1, 255, 10, 255); 
        ledcWrite(PWM_CHANNEL, pwm_direto);
    }

    // 5. INTERFACE (OLED) - Resposta rápida ao usuário
    if (precisaAtualizarOLED) {
        atualizarTelaOLED();
        precisaAtualizarOLED = false;
    }

    unsigned long tempoAtual = millis();

    // 6. CÁLCULO DE SENSORES E TELEMETRIA
    if (tempoAtual - tempoAnteriorBlynk >= 1000) { // Ciclo base de 1 segundo
        tempoAnteriorBlynk = tempoAtual;

        // --- LÓGICA DO CRONÔMETRO ---
        if (cronometroRodando && valor_pwm > 0) {
            segundosTranscorridos++;
            precisaAtualizarOLED = true;
        }

        // --- CÁLCULO DO RPM ---
        int rpm_para_envio = map(valor_pwm, 0, 255, 0, 6400);
        
        /*
        // --- ENVIO PARA O BLYNK (1 SEGUNDO SEM TRAVAS) ---
        if (WiFi.status() == WL_CONNECTED) {
            Blynk.virtualWrite(V0, segundosTranscorridos); // Tempo segundo a segundo
            Blynk.virtualWrite(V1, valor_pwm);             // PWM
            Blynk.virtualWrite(V2, rpm_para_envio);        // RPM
            
            ultimoTempoEnviado = segundosTranscorridos;
        }
        */
       
        // --- MODO MQTT (MÁXIMA VELOCIDADE - 1 SEGUNDO) ---
        if (mqttClient.connected()) {
            StaticJsonDocument<128> mqttDoc;
            mqttDoc["tempo"] = segundosTranscorridos;
            mqttDoc["rpm"] = rpm_para_envio;
            mqttDoc["pwm"] = valor_pwm;
            mqttDoc["status"] = travaEmergencia ? "ALERTA" : "OK";
            
            char buffer[128];
            serializeJson(mqttDoc, buffer);
            mqttClient.publish("if/homogeneizador/dados", buffer);
            mqttClient.publish("if/homogeneizador/nema/cmd", String(valor_pwm).c_str()); // Envia o PWM para o NEMA 17
        }

        // --- MODO BLYNK ECONÔMICO (SEM CHART) ---
        if (WiFi.status() == WL_CONNECTED) {
            
            // 1. PWM e RPM: Só envia se o valor MUDAR (Economia extrema)
            if (valor_pwm != ultimoPWMEnviado) {
                Blynk.virtualWrite(V1, valor_pwm);
                Blynk.virtualWrite(V2, rpm_para_envio);
                ultimoPWMEnviado = valor_pwm;
            }

            // 2. TEMPO: Só envia a cada 5 segundos (Suficiente para leitura numérica)
            if (cronometroRodando && segundosTranscorridos % 5 == 0) {
                Blynk.virtualWrite(V0, segundosTranscorridos);
            } 
            // Garante o envio do 0 ao resetar
            else if (segundosTranscorridos == 0 && ultimoTempoEnviado != 0) {
                Blynk.virtualWrite(V0, 0);
                ultimoTempoEnviado = 0;
            }
        }
    }

    // 7. MONITOR SERIAL (A cada 500ms)
    if (tempoAtual - tempoAnteriorSerial >= 500) { 
        tempoAnteriorSerial = tempoAtual;

        // Note que aqui removemos o map(6400) e usamos a lógica do sensor
        Serial.print("> STATUS: ");
        Serial.print(temperatura, 1); Serial.print("C | PWM: "); 
        Serial.print(valor_pwm); Serial.println();
    }
}
