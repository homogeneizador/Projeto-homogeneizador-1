# Projeto-homogeneizador-1

# 🌀 Homogeneizador de Amostras - Módulo de Controle

> **⚠️ ATENÇÃO: CONFIGURAÇÃO INICIAL OBRIGATÓRIA**
> 
> Antes de realizar o upload do código para o ESP32, você **deve** atualizar as credenciais de rede no arquivo principal (`.ino` ou `.cpp`). Sem isso, o sistema não acessará o Blynk nem o Dashboard MQTT.
>
> Procure pelas seguintes linhas no início do código:
> ```cpp
> char ssid[] = "NOME_DA_SUA_REDE"; // Substitua pelo nome do Wi-Fi
> char pass[] = "SENHA_DA_REDE";     // Substitua pela senha do Wi-Fi
> ```

## 🌐 Monitoramento Web (Live Dashboard)

O projeto conta com um dashboard estático hospedado via **GitHub Pages**, que permite o acompanhamento em tempo real sem a necessidade de instalação de softwares adicionais.

* **URL do Dashboard:** https://homogeneizador.github.io/Projeto-homogeneizador-1/
* **Broker:** `test.mosquitto.org` (WebSockets)
* **Tópico:** `ifsudestemg/homogeneizador/dados`
---

## 📌 Visão Geral do Projeto
Este projeto faz parte do desenvolvimento de um **Homogeneizador de Leite Materno** para UTIs Neonatais. Minha responsabilidade consiste no **Módulo de Controle e Telemetria**, integrando hardware de precisão com monitoramento via IoT.

### Principais Funcionalidades:
* **Controle Híbrido:** Ajuste de velocidade via Encoder Físico ou App Blynk.
* **Telemetria de Precisão:** Leitura do RPM Real do motor via Fio Amarelo (FG).
* **Segurança Ativa:** Isolamento galvânico total e botão de emergência com prioridade de interrupção.
* **Multicloud:** Envio simultâneo de dados para Blynk e Broker MQTT (Mosquitto).


## 🛠️ Hardware: Guia de Conexão e Pinagem

Este projeto utiliza um **ESP32 DevKit V1** como unidade de processamento central. A arquitetura foi desenhada priorizando a **isolação galvânica** entre o circuito de controle (3.3V) e o circuito de potência (12V) para garantir a integridade do microcontrolador.

### 1. Pinagem de Referência (GPIOs)

| Componente | Função | Pino ESP32 |
| :--- | :--- | :--- |
| **Motor (PWM)** | Controle de Velocidade | GPIO 25 |
| **Encoder CLK** | Pulso de Rotação (Interrupção) | GPIO 26 |
| **Encoder DT** | Direção da Rotação | GPIO 27 |
| **Encoder SW** | Botão de Pressão | GPIO 14 |
| **Display I2C SCL** | Clock de Comunicação | GPIO 22 |
| **Display I2C SDA** | Dados de Comunicação | GPIO 21 |

---

### 2. Esquemas de Ligação

#### 📺 Display OLED (SH1107 - I2C)
Conecte o display ao barramento I2C padrão do ESP32. Utilize a linha de **3.3V** para alimentação.
* **VCC** ➡ 3.3V
* **GND** ➡ GND
* **SCL** ➡ GPIO 22
* **SDA** ➡ GPIO 21

#### 🎡 Encoder Rotativo (KY-040)
O encoder permite o ajuste fino do RPM. Os pinos CLK e DT possuem resistores *pull-up* internos configurados via software.
* **CLK** ➡ GPIO 26
* **DT** ➡ GPIO 27
* **SW** ➡ GPIO 14
* **VCC** ➡ 3.3V
* **GND** ➡ GND

#### ⚙️ Motor Brushless 310 (Com Isolação PC817C)
O motor é controlado via sinal PWM. Para evitar ruídos elétricos, utilizamos um **Optoacoplador PC817C**. 

> **⚠️ IMPORTANTE:** Os terras (GND) do ESP32 e da Fonte 12V **NÃO** devem ser interconectados.

**Lado de Controle (ESP32):**
* **GPIO 25** ➡ Resistor 330Ω ➡ **Pino 1** (Anodo) do PC817.
* **GND ESP32** ➡ **Pino 2** (Catodo) do PC817.

**Lado de Potência (Motor/Fonte 12V):**
* **GND Fonte 12V** ➡ **Pino 3** (Emissor) do PC817.
* **Fio Branco (Motor)** ➡ **Pino 4** (Coletor) do PC817.
* **Resistor 10kΩ (Pull-up)** ➡ Entre o **Pino 4** do PC817 e o **+12V** da Fonte.

**Alimentação Direta do Motor:**
* **Fio Vermelho** ➡ +12V da Fonte.
* **Fio Preto** ➡ GND da Fonte.

### 📊 Adendo: Telemetria de RPM Real (Fio Amarelo - FG)

Para uma medição precisa da rotação (em vez de apenas uma estimativa via PWM), utilizamos um **segundo Optoacoplador PC817C**. Isso isola o sinal de pulso de 12V que sai do motor, convertendo-o para 3.3V legíveis pelo ESP32.

#### **Circuito de Leitura (Isolação de Entrada)**

**Lado do Motor (Sinal FG):**
* **Fio Amarelo (Motor)** ➡ Resistor 1kΩ ➡ **Pino 1** (Anodo) do PC817.
* **GND Fonte 12V** ➡ **Pino 2** (Catodo) do PC817.

**Lado do ESP32 (Entrada de Dados):**
* **GPIO 18** (Sugerido) ➡ **Pino 4** (Coletor) do PC817.
* **GND ESP32** ➡ **Pino 3** (Emissor) do PC817.
* **Configuração de Software:** O pino deve ser configurado como `INPUT_PULLUP` no código para garantir a leitura dos pulsos.

> **Nota Técnica:** O fio amarelo (FG - Frequency Generator) do motor Brushless 310 emite pulsos de tensão baseados na rotação interna. O uso do optoacoplador previne que picos indutivos alcancem as portas lógicas do microcontrolador.

## 💻 Configuração de Ambiente e Software

O firmware foi desenvolvido utilizando o ecossistema **ESP32** e pode ser compilado tanto no **VS Code (PlatformIO)** quanto na **Arduino IDE**.

### 1. Requisitos de Software
* **VS Code** + Extensão **PlatformIO** (Recomendado) ou **Arduino IDE** (v2.0+).
* **Driver USB:** CP210x ou CH340 (dependendo da sua placa ESP32) para comunicação serial.

### 2. Dependências (Bibliotecas)
As seguintes bibliotecas são necessárias. Se estiver usando PlatformIO, elas serão instaladas automaticamente via `platformio.ini`. No Arduino IDE, instale-as pelo **Gerenciador de Bibliotecas**:

| Biblioteca | Versão Recomendada | Função |
| :--- | :--- | :--- |
| **Blynk** | ^1.3.2 | Interface IoT e Controle Mobile |
| **U8g2** | ^2.35.19 | Controle do Display OLED SH1107 |
| **ArduinoJson** | **6.21.3** (Não use v7.x) | Serialização de dados para MQTT e Python |
| **PubSubClient** | ^2.8 | Protocolo de comunicação MQTT |

### 3. Configuração do Arquivo `platformio.ini`
Caso utilize o VS Code, configure seu arquivo de projeto conforme abaixo:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    blynkkk/Blynk @ ^1.3.2
    olikraus/U8g2 @ ^2.35.19
    bblanchon/ArduinoJson @ ^6.21.3
    knolleary/PubSubClient @ ^2.8
```

## 📱 Interface IoT (Blynk)

O controle e a telemetria remota são realizados através da plataforma **Blynk IoT**. Para o funcionamento correto do firmware, o seu Dashboard no Blynk (Web ou Mobile) deve conter os seguintes **Datastreams**:

### Configuração de Datastreams (Virtual Pins)

| Pino Virtual | Nome do Datastream | Tipo de Dado | Valor (Min - Max) | Função |
| :--- | :--- | :--- | :--- | :--- |
| **V0** | Temperatura | Float (Dec. 1) | 0 - 50 | Exibe a temperatura simulada/lida. |
| **V1** | Controle PWM | Integer | 0 - 255 | Slider para controle manual da velocidade. |
| **V2** | Rotação (RPM) | Integer | 0 - 7000 | Exibe o cálculo de RPM Real (Fio Amarelo). |
| **V3** | Emergência | Integer | 0 - 1 | Botão (Switch) para parada imediata. |

---

### 🚀 Funcionalidades do Aplicativo

1. **Monitoramento em Tempo Real:** Visualização da temperatura e do RPM real do motor diretamente no smartphone.
2. **Controle Híbrido:** O Slider no aplicativo sincroniza automaticamente com o Encoder Físico (KY-040). Se você girar o botão no hardware, o Slider no celular se move, e vice-versa.
3. **Segurança Crítica:** O botão de **Emergência (V3)** tem prioridade máxima no código. Uma vez ativado via App, o PWM é zerado no hardware e o processamento do encoder é bloqueado até que a trava seja liberada.
4. **Notificações:** O sistema está configurado para disparar um evento de log (`emergencia`) sempre que a parada brusca for acionada, permitindo auditoria de uso.

> **Dica de Configuração:** No Widget do Slider (V1), habilite a opção **"Send Values on Release"** como "OFF" se desejar um controle fluido e instantâneo, ou "ON" para economizar dados e evitar latência em redes instáveis (como a do campus).

## 🧠 Lógica de Controle e Telemetria

Esta seção detalha o funcionamento interno do firmware e as estratégias de controle adotadas.

### 1. Algoritmo de Controle do Motor (PWM)
O motor Brushless 310 é controlado via sinal **PWM (Pulse Width Modulation)** com resolução de 8 bits (0-255). 
* **Frequência:** 5000Hz (Ideal para evitar ruído audível e garantir torque estável).
* **Duty Cycle:** O sinal é enviado através de um optoacoplador, garantindo que o driver interno do motor receba pulsos limpos de 0-12V.

### 2. Leitura de Feedback (FG - Frequency Generator)
Diferente de sistemas de malha aberta, este projeto utiliza o sinal de feedback do motor para monitoramento real:
* **Fórmula de Cálculo:** $RPM = \frac{Pulsos \times 60}{Polos}$
* **Interrupção:** O sinal é processado via `IRAM_ATTR` para garantir que o microcontrolador não perca pulsos durante as tarefas de rede (Wi-Fi/Blynk).

### 3. Sincronização Híbrida (Hardware + Cloud)
O sistema implementa uma sincronização bidirecional:
* Se o **Encoder Físico** for girado, o valor no **App Blynk** e no **Broker MQTT** é atualizado instantaneamente.
* Se o **Slider no App** for movido, o motor altera a velocidade e o novo valor é refletido no **Display OLED**.

---

## 🚀 Como Operar o Protótipo

1. **Inicialização:** Ao ligar o sistema, o OLED exibirá "MODO HIBRIDO". O sistema tentará se conectar ao Wi-Fi configurado automaticamente.
2. **Controle de Velocidade:**
   * Gire o **Encoder KY-040** para a direita para acelerar e para a esquerda para desacelerar.
   * A aceleração é dinâmica: giros rápidos resultam em incrementos maiores (+10), giros lentos permitem ajuste fino (+2).
3. **Monitoramento:**
   * O display exibe a **Temperatura** (simulada para teste de sensores) e o **RPM Real** lido do motor.
   * Os mesmos dados podem ser acompanhados via Dashboard Blynk ou qualquer cliente MQTT (tópico: `ifsudestemg/homogeneizador/dados`).
4. **Segurança (Parada de Emergência):**
   * Acione o botão virtual **V3** no aplicativo para interromper o processo. O motor parará imediatamente e o controle físico será bloqueado até a liberação via software.

---

## 📈 Possíveis Melhorias (Roadmap)
* Implementação de controle **PID** para manter o RPM constante sob carga.
* Adição de sensor de temperatura real (DS18B20) para monitoramento do leite.
* Interface Web nativa (Webserver) para operação sem dependência de internet externa.
