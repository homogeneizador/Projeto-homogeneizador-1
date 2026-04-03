#define BLYNK_TEMPLATE_ID   "T"
#define BLYNK_TEMPLATE_NAME "H"
#define BLYNK_AUTH_TOKEN    "P"

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
char ssid[] = "" // Substitua pelo nome da sua rede WiFi
char pass[] = "" // Substitua pela senha da sua rede WiFi

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


// SINCRONIZAÇÃO BLYNK (APP -> ESP32)

BLYNK_WRITE(V1) {
    int valorRecebido = param.asInt();
    if (valorRecebido != valor_pwm) {
        valor_pwm = valorRecebido;
        precisaAtualizarOLED = true;
    }
}

// BOTÃO DE EMERGÊNCIA (V3)
BLYNK_WRITE(V3) {
    travaEmergencia = (param.asInt() == 1);
    if (travaEmergencia == 1) {
        valor_pwm = 0; // Para o motor imediatamente
        precisaAtualizarOLED = true;

        //Força o desligamento imediato no pino (lógica invertida do motor)
        ledcWrite(PWM_CHANNEL, 255); // Sinal máximo para desligar o motor

        Serial.println("EMERGENCIA ATIVADA: MOTOR PARADO!!!!");

        // Opcional: Envia uma notificação para o app
        Blynk.logEvent("emergencia_ativada", "Motor parado por emergência!");
    }
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
    u8g2.drawStr(0, 12, "MODO HIBRIDO (IOT)");
    u8g2.drawHLine(0, 15, 128);

    u8g2.setFont(u8g2_font_helvB14_tf);
    u8g2.setCursor(0, 45);
    u8g2.print("TEMP: "); u8g2.print(temperatura, 1); u8g2.print(" C");

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
    while (!mqttClient.connected()) {
        Serial.print("Tentando conexão MQTT...");
        // Cria um ID de cliente único usando o MAC do ESP32
        String clientId = "ESP32-Homog-";
        clientId += String(random(0xffff), HEX);
        
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("conectado ao MQTT!");
        } else {
            Serial.print("falha, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" tentando novamente em 5s");
            delay(5000);
        }
    }
}

void loop() {
    // 1. ATUALIZAÇÃO DE REDE
    Blynk.run(); 

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
    if (valor_pwm == 0 || travaEmergencia) {
        ledcWrite(PWM_CHANNEL, 0); // Garante desligado
    } else {
        // Mapeia o valor do encoder (1-255) para uma faixa que o motor entenda.
        // Se o motor pula para o máximo muito rápido, diminua o 200 para 150.
        int pwm_suave = map(valor_pwm, 1, 255, 10, 255); 
        ledcWrite(PWM_CHANNEL, pwm_suave);
    } 

    // 5. INTERFACE (OLED) - Resposta rápida ao usuário
    if (precisaAtualizarOLED) {
        atualizarTelaOLED();
        precisaAtualizarOLED = false;
    }

    unsigned long tempoAtual = millis();

    // 6. CÁLCULO DE SENSORES E TELEMETRIA (A cada 1 segundo)
    if (tempoAtual - tempoAnteriorBlynk >= 1000) {
        tempoAnteriorBlynk = tempoAtual;
        
        // --- CÁLCULO DO RPM REAL (FG SIGNAL) ---
        noInterrupts();
        unsigned long pulsos = contadorPulsosFG;
        contadorPulsosFG = 0; // Reseta para o próximo segundo
        interrupts();

        // Cálculo para o B2418 (ajuste o divisor se necessário)
        int rpm_real = (pulsos * 60) / 6; 
        if (valor_pwm == 0) rpm_real = 0; // Limpa ruído parado

        // --- LÓGICA DE TEMPERATURA ---
        if (valor_pwm > 0) {
            if (temperatura < 38.0) temperatura += 0.5;
            else if (temperatura < 40.0) temperatura += 0.1;
        } else {
            if (temperatura > 25.0) temperatura -= 0.1;
        }

        // Força atualização do OLED para mostrar o RPM real
        precisaAtualizarOLED = true;

        // --- ATUALIZAÇÃO BLYNK ---
        Blynk.virtualWrite(V0, temperatura); 
        Blynk.virtualWrite(V1, valor_pwm);
        Blynk.virtualWrite(V2, rpm_real); // Enviando o valor lido do sensor

        // --- PUBLICAÇÃO MQTT ---
        if (mqttClient.connected()) {
            StaticJsonDocument<128> mqttDoc;
            mqttDoc["temp"] = (float)((int)(temperatura * 10)) / 10.0;
            mqttDoc["rpm"] = rpm_real;
            mqttDoc["pwm"] = valor_pwm;
            mqttDoc["trava"] = travaEmergencia ? 1 : 0;
            
            char buffer[128];
            serializeJson(mqttDoc, buffer);
            mqttClient.publish("alguma_coisa", buffer);  //mudar para o nome desejável
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
