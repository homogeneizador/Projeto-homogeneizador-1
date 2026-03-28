#define BLYNK_TEMPLATE_ID   "TMPL2S15vGwG_"
#define BLYNK_TEMPLATE_NAME "Homogeneizador"
#define BLYNK_AUTH_TOKEN    "PeyZusk1HQHYqSeYs6N8wAs8A4EhifPl"

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
char ssid[] = "tp_link_5200_2_4";
char pass[] = "9ENAE674DVSG672HJBS3";

// --- HARDWARE ---
#define MOTOR_PIN 25
#define ENC_CLK   26
#define ENC_DT    27
#define ENC_SW    14
#define PWM_CHANNEL 0
#define PWM_FREQ    5000
#define PWM_RES     8
#define FG_PIN 18


U8G2_SH1107_PIMORONI_128X128_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// --- VARIÁVEIS ---
volatile int valor_pwm = 0;   
float temperatura = 25.0;     
int rpm = 0;
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
    // 1. ATUALIZAÇÃO DE REDE (Prioridade Máxima)
    // Recebe o comando do botão de emergência do servidor
    Blynk.run(); 

    // 2. FILTRO DE SEGURANÇA (Trava de Emergência)
    // Se a trava estiver ativa, forçamos o PWM a zero antes de qualquer cálculo
    if (travaEmergencia) {
        valor_pwm = 0;
        encoderAcumulado = 0; // Descarta giros acidentais no KY-040 físico
    }
    if (!mqttClient.connected()) {
        reconnectMQTT();
    }
    mqttClient.loop();

    // 3. PROCESSAMENTO DO ENCODER 
    // Só entra aqui se a emergência NÃO estiver travada
    if (encoderAcumulado != 0 && !travaEmergencia) {
        noInterrupts();
        int passos = encoderAcumulado;
        encoderAcumulado = 0;
        interrupts();

        // Lógica de aceleração que já validamos
        if (abs(passos) > 1) valor_pwm += (passos * 10);
        else valor_pwm += (passos * 2);

        valor_pwm = constrain(valor_pwm, 0, 255);
        precisaAtualizarOLED = true;
    }

    // 4. SAÍDA PARA O MOTOR (Hardware)
    // Aplica a lógica invertida (255 - valor_pwm), se utilizar o optoacoplador, utilize o valor_pwm
    // int sinal_invertido = 255 - valor_pwm;
    ledcWrite(PWM_CHANNEL, valor_pwm); // Envia o sinal PWM para o motor (lógica direta, pois o hardware é invertido)

    // 5. INTERFACE E SENSORES (OLED e Telemetria)
    if (precisaAtualizarOLED) {
        atualizarTelaOLED();
        precisaAtualizarOLED = false;
    }
    unsigned long tempoAtual = millis();

    // Sincronização Geral (Blynk + MQTT + Temperatura) a cada 1s
    if (tempoAtual - tempoAnteriorBlynk >= 1000) {
        tempoAnteriorBlynk = tempoAtual;
        
        // --- LÓGICA DE TEMPERATURA ---
        if (valor_pwm > 0) {
            if (temperatura < 38.0) temperatura += 0.5;
            else if (temperatura < 40.0) temperatura += 0.1;
            else {
                float oscilacao = (random(-20, 20) / 100.0); 
                temperatura = 40.0 + oscilacao;
            }
        } else {
            if (temperatura > 25.0) temperatura -= 0.1;
        }

        // --- ATUALIZAÇÃO BLYNK ---
        Blynk.virtualWrite(V0, temperatura); 
        Blynk.virtualWrite(V1, valor_pwm);
        Blynk.virtualWrite(V2, map(valor_pwm, 0, 255, 0, 6400));

        // --- PUBLICAÇÃO MQTT ---
        if (mqttClient.connected()) {
            StaticJsonDocument<128> mqttDoc;
            mqttDoc["temp"] = (float)((int)(temperatura * 10)) / 10.0;
            mqttDoc["rpm"] = map(valor_pwm, 0, 255, 0, 6400);
            mqttDoc["trava"] = travaEmergencia ? 1 : 0;
            
            char buffer[128];
            serializeJson(mqttDoc, buffer);
            mqttClient.publish("ifsudestemg/homogeneizador/dados", buffer);
        }
    }

    // 6. MONITOR SERIAL (VS CODE) - A cada 500ms
    if (tempoAtual - tempoAnteriorSerial >= 500) { 
        tempoAnteriorSerial = tempoAtual;

        int rpm_atual = map(valor_pwm, 0, 255, 0, 6400);

        // Mensagem Amigável para o Humano ler
        Serial.print("> STATUS: ");
        Serial.print(temperatura, 1); Serial.print("C | ");
        Serial.print(rpm_atual); Serial.print(" RPM | ");
        Serial.print("PWM: "); Serial.println(valor_pwm);

        // JSON para o Computador/Python ler (Opcional)
        StaticJsonDocument<128> doc;
        doc["temp"] = (float)((int)(temperatura * 10)) / 10.0;
        doc["rpm"] = rpm_atual;
        doc["pwm"] = valor_pwm;
        serializeJson(doc, Serial);
        Serial.println(); // Pula linha para o próximo pacote
    }

}
