# 🚀 Robô Balanceador ESP32 - Comunicação WiFi

## 📋 Configuração Inicial

### 1. **Configurar WiFi no ESP32**
✅ **Já configurado com suas credenciais:**
```cpp
const char* ssid = "R3_SATIRO_5G";      // ✅ Sua rede WiFi
const char* password = "Froid@1216PM00"; // ✅ Sua senha WiFi  
const char* pc_ip = "192.168.1.14";     // ✅ Seu IP atual
```

### 2. **Descobrir IP do seu PC**
```bash
# Windows (PowerShell)
ipconfig

# Procure por "Endereço IPv4" da sua rede WiFi
```

### 3. **Instalar dependências Python (opcional)**
```bash
pip install matplotlib
```

## 🔋 **Alimentação por Bateria**

### Opções de Bateria:
- **18650 Li-ion** (3.7V) + Step-up para 5V
- **Power Bank** via cabo USB
- **Baterias AA** (4x1.5V = 6V) no pino VIN

### Conexões:
```
Bateria → ESP32 VIN (ou USB)
ESP32 5V → L298N VCC
ESP32 GND → Bateria GND + L298N GND
```

## 📡 **Monitoramento no PC**

### Opção 1: Receptor Simples (Terminal)
```bash
python receiver.py
```

### Opção 2: Monitor Visual (Gráficos)
```bash
python monitor_visual.py
```

## 🛠️ **Troubleshooting**

### ESP32 não conecta WiFi:
1. Verifique SSID e senha
2. Aproxime ESP32 do roteador
3. Use rede 2.4GHz (não 5GHz)

### PC não recebe dados:
1. Verifique se IP do PC está correto
2. Desative firewall temporariamente
3. Use `ipconfig` para confirmar IP

### Robô oscila muito:
1. Reduza `Ki` (ex: de 5.0 para 2.0)
2. Aumente `Kd` (ex: de 1.0 para 2.0)
3. Verifique calibração do sensor

## 📊 **Dados Enviados**

O ESP32 envia JSON via UDP a cada 100ms:
```json
{
  "timestamp": 12345,
  "angle": -2.45,
  "error": 2.45,
  "output": -123,
  "kp": 2.0,
  "ki": 5.0,
  "kd": 1.0
}
```

## 🔧 **Parâmetros PID**

- **Kp**: Resposta proporcional (padrão: 2.0)
- **Ki**: Resposta integral (padrão: 5.0)
- **Kd**: Resposta derivativa (padrão: 1.0)

**Dica**: Comece com Ki baixo e aumente gradualmente!

## 🎯 **Modo Híbrido**

Este código funciona em **modo híbrido**:
- ✅ **Com WiFi**: Envia dados para PC + Serial Monitor
- ✅ **Sem WiFi**: Funciona offline apenas com Serial Monitor
- ✅ **Com Bateria**: Totalmente portátil

Perfeito para desenvolvimento e uso final! 🚀

## 🔧 Hardware Necessário

### Componentes Principais
- **ESP32 DevKit V1** (ou similar)
- **MPU6050** - Sensor de aceleração e giroscópio
- **Driver de Motor L298N** - Para controle dos motores DC
- **2x Motores DC** com redução (ex: motores de roda)
- **Chassi do robô** (estrutura para montagem)
- **Bateria** (7.4V LiPo recomendada)
- **Rodas** apropriadas para os motores

### Conexões dos Pinos

#### MPU6050 (I2C)
```
MPU6050    →    ESP32
VCC        →    3.3V
GND        →    GND
SDA        →    GPIO 21
SCL        →    GPIO 22
```

#### Driver L298N
```
L298N      →    ESP32
VCC        →    VIN (ou bateria)
GND        →    GND
IN1        →    GPIO 19 (Motor 1)
IN2        →    GPIO 18 (Motor 1)
ENA        →    GPIO 5  (PWM Motor 1)
IN3        →    GPIO 17 (Motor 2) tx2 
IN4        →    GPIO 16 (Motor 2)rx2
ENB        →    GPIO 4  (PWM Motor 2)
```

#### Motores
```
Motor 1    →    OUT1 e OUT2 (L298N)
Motor 2    →    OUT3 e OUT4 (L298N)
```

## 🚀 Como Usar

### 1. Compilar e Carregar o Código

```bash
# Compilar o projeto
python -m platformio run

# Carregar no ESP32 (certifique-se de que está conectado via USB)
python -m platformio run --target upload
```

### 2. Conectar à Interface Web

1. **Ligue o ESP32** - ele criará um ponto de acesso WiFi
2. **Conecte seu dispositivo** ao WiFi:
   - **Nome da rede**: `Robo-Balanceador`
   - **Senha**: `12345678`
3. **Abra o navegador** e acesse: `http://192.168.4.1`

### 3. Interface de Controle

A interface web oferece:

#### 🎛️ Controles PID
- **Kp (Proporcional)**: 0 a 100 (padrão: 10.0)
- **Ki (Integral)**: 0 a 20 (padrão: 0.5)  
- **Kd (Derivativo)**: 0 a 50 (padrão: 1.0)

#### 📊 Gráficos em Tempo Real
- **Ângulo**: Inclinação atual do robô (graus)
- **Erro**: Diferença entre setpoint (0°) e ângulo atual
- **Duty Cycle**: Sinal PWM aplicado aos motores (-255 a +255)

## ⚙️ Ajuste do Controlador PID

### Processo de Sintonia

1. **Comece com valores baixos**:
   - Kp = 5, Ki = 0, Kd = 0

2. **Ajuste o Kp primeiro**:
   - Aumente gradualmente até o robô começar a reagir
   - Se oscilar muito, diminua
   - Se não reagir, aumente

3. **Adicione Kd para estabilidade**:
   - Comece com Kd = Kp/4
   - Ajuste para reduzir oscilações

4. **Adicione Ki se necessário**:
   - Use valores baixos (0.1 - 2.0)
   - Ajuda a eliminar erro steady-state

### Dicas de Ajuste

- **Robô muito lento**: Aumente Kp
- **Robô oscila muito**: Diminua Kp, aumente Kd
- **Robô não volta ao centro**: Aumente Ki (cuidado com overshoot)
- **Resposta instável**: Diminua todos os valores

## 📡 Sistema de Comunicação

### WebSocket
- **Endpoint**: `/ws`
- **Frequência de dados**: 10Hz (100ms)
- **Formato**: JSON com campos `angle`, `error`, `duty`

### Exemplo de Mensagem
```json
{
  "angle": -2.35,
  "error": 2.35, 
  "duty": 45.2
}
```

## 🔧 Características Técnicas

### Controle PID
- **Frequência**: 100Hz (10ms)
- **Setpoint**: 0° (robô na vertical)
- **Saída**: -255 a +255 (duty cycle PWM)

### Sensor Fusion
- **Filtro Complementar**: 98% giroscópio + 2% acelerômetro
- **Taxa de atualização**: 100Hz
- **Faixa do giroscópio**: ±500°/s
- **Faixa do acelerômetro**: ±8g

### PWM
- **Frequência**: 5kHz
- **Resolução**: 8 bits (0-255)
- **Canais**: 2 (um para cada motor)

## 🐛 Resolução de Problemas

### Robô não equilibra
- Verifique as conexões do MPU6050
- Confirme orientação do sensor (eixo Y = inclinação)
- Ajuste os valores PID
- Verifique se os motores giram na direção correta

### Interface web não carrega
- Confirme conexão com WiFi `Robo-Balanceador`
- Acesse `http://192.168.4.1` (não https)
- Verifique se ESP32 está ligado e funcionando

### Motores não respondem
- Verifique conexões do L298N
- Confirme alimentação adequada (7-12V)
- Teste com duty cycle manual no código

### Dados não aparecem no gráfico
- Abra o console do navegador (F12)
- Verifique erros de WebSocket
- Confirme que ESP32 está enviando dados

## 📈 Expansões Possíveis

- **Controle de velocidade** (setpoint diferente de 0°)
- **Controle remoto** via interface web
- **Logging de dados** para análise offline
- **Filtro Kalman** para melhor sensor fusion
- **Auto-tuning PID** usando algoritmos adaptativos

## 📝 Estrutura do Código

```
src/main.cpp           - Código principal
platformio.ini         - Configuração do projeto
README.md             - Esta documentação
```

### Funções Principais
- `updateIMU()` - Lê sensores e calcula ângulo
- `moveMotors()` - Controla ambos os motores
- `handleWebSocketMessage()` - Processa comandos web
- `setup()` - Inicialização do sistema
- `loop()` - Loop principal (PID + comunicação)

---

**Autor**: Sistema de Controle Self-Balance  
**Data**: Novembro 2025  
**Versão**: 1.0