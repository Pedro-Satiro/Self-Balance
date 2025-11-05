# Robô Self-Balance com Controle PID

Este projeto implementa um robô auto-balanceador usando ESP32, sensor MPU6050 e controle PID com interface web para ajuste de parâmetros em tempo real.

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
IN3        →    GPIO 17 (Motor 2)
IN4        →    GPIO 16 (Motor 2)
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