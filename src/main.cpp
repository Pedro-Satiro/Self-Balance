#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <WebServer.h>

// =================================================================
//  1. CONFIGURAÇÕES DE AJUSTE (TUNING) - EDITE AQUI!
// =================================================================

// --- Configurações WiFi ---de WiFi
const char* password = "icomputacaoufal"; 
const char* ssid = "IC-ALUNOS";    
// const char* ssid = "satiro";      
// const char* password = "teste123987"; 
const char* pc_ip = "192.168.1.14";     // ✅ IP do seu PC (encontrado via ipconfig)
const int udp_port = 8888;              // Porta UDP para envio

// Estes são os seus ganhos. Mude estes valores para ajustar o robô.
double Kp = 0.0;
double Ki = 0.0;  // Reduzido de 15.0 para evitar oscilações
double Kd = 0.0;


double pid_setpoint = 0.0;  // Posição de equilíbrio (ajustar se necessário)

// Offset de calibração do ângulo (ajustar para posição vertical real)
double angle_offset = 0.0;  // Começar com zero e calibrar via web

// =================================================================
//  2. PINOS E HARDWARE 
// =================================================================

// --- Pinos de Controlo do L298N ---
const int M1_IN1 = 19;
const int M1_IN2 = 18;
const int M1_ENA_PWM = 5;
const int M2_IN3 = 17;
const int M2_IN4 = 16;
const int M2_ENB_PWM = 4;

// --- Pinos I2C do MPU-6050 ---
const int MPU_SDA = 21;
const int MPU_SCL = 22;

// --- Configuração do PWM (ledc) ---
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8; // 8 bits = 0-255
const int PWM_CHANNEL_1 = 0;
const int PWM_CHANNEL_2 = 1;

// --- Objetos de Hardware ---
Adafruit_MPU6050 mpu;

// --- Objetos WiFi ---
WiFiUDP udp;
WebServer server(80);  // Servidor web na porta 80
bool wifi_connected = false;

// =================================================================
//  3. VARIÁVEIS GLOBAIS DO PID E SENSOR
// =================================================================

// --- Variáveis Globais do PID ---
double angle_pitch = 0.0;   // O ângulo atual lido do sensor
double pid_output = 0.0;    // A saída do PID (velocidade do motor)

// --- Variáveis de Tensão dos Motores ---
double motor1_voltage = 0.0; // Tensão aplicada ao motor 1 (0-12V)
double motor2_voltage = 0.0; // Tensão aplicada ao motor 2 (0-12V)
int motor1_pwm = 0;         // PWM atual do motor 1 (0-255)
int motor2_pwm = 0;         // PWM atual do motor 2 (0-255)

// --- Variáveis do PID Customizado ---
double last_error = 0.0;      // Erro anterior (para derivativo)
double integral = 0.0;        // Acumulador do termo integral
double output_min = -200.0;   // Limite mínimo de saída
double output_max = 200.0;    // Limite máximo de saída
unsigned long last_pid_time = 0; // Tempo da última execução do PID

// --- Variáveis de Sensor Fusion (Filtro Complementar) ---
float accelAngleY = 0.0;
float gyroY = 0.0;
unsigned long last_loop_time = 0;

// Intervalo do loop de controlo (10ms = 100Hz)
const int PID_LOOP_INTERVAL_MS = 10;
// Intervalo para imprimir no monitor (100ms = 10Hz)
const int PRINT_INTERVAL_MS = 100;
unsigned long last_print_time = 0;

// =================================================================
//  4. FUNÇÃO PID CUSTOMIZADA
// =================================================================

// Função PID customizada com anti-windup avançado
double computePID(double input) {
  unsigned long now = millis();
  double dt = (now - last_pid_time) / 1000.0; // Delta time em segundos
  
  // Evita divisão por zero na primeira execução
  if (last_pid_time == 0 || dt <= 0) {
    last_pid_time = now;
    return 0.0;
  }
  
  // Calcula o erro atual (INVERTIDO para lógica correta de balanceamento)
  // Se robô inclina para frente (+), erro positivo → acelera para frente
  double error = input - pid_setpoint;
  
  // === TERMO PROPORCIONAL ===
  double proportional = Kp * error;
  
  // === TERMO INTEGRAL ===
  integral += error * dt;
  
  // Anti-windup: limita a integral para evitar saturação
  double integral_limit = 50.0; // Limite fixo para evitar windup
  if (integral > integral_limit) integral = integral_limit;
  if (integral < -integral_limit) integral = -integral_limit;
  
  // Calcula o termo integral
  double integral_term = Ki * integral;
  
  // === TERMO DERIVATIVO ===
  double derivative = (error - last_error) / dt;
  double derivative_term = Kd * derivative;
  
  // === SAÍDA TOTAL ===
  double output = proportional + integral_term + derivative_term;
  
  // Limita a saída final
  if (output > output_max) output = output_max;
  if (output < output_min) output = output_min;
  
  // Anti-windup: se a saída saturou, reduz a integral
  if ((output >= output_max && error > 0) || (output <= output_min && error < 0)) {
    integral *= 0.8; // Reduz a integral em 20%
  }
  
  // Atualiza variáveis para próxima iteração
  last_error = error;
  last_pid_time = now;
  
  return output;
}

// Função para resetar o PID (útil para recalibração)
void resetPID() {
  integral = 0.0;
  last_error = 0.0;
  last_pid_time = 0;
}

// =================================================================
//  5. FUNÇÕES DE CONTROLO DO ROBÔ
// =================================================================

// Função para conectar WiFi (não bloqueia se falhar)
void connectWiFi() {
  Serial.print("Conectando ao WiFi");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println("\n✅ WiFi conectado!");
    Serial.print("IP do ESP32: ");
    Serial.println(WiFi.localIP());
    Serial.print("Enviando dados para: ");
    Serial.print(pc_ip);
    Serial.print(":");
    Serial.println(udp_port);
    udp.begin(udp_port);
  } else {
    wifi_connected = false;
    Serial.println("\n❌ WiFi falhou. Continuando offline...");
  }
}

// Função para enviar dados via UDP
void sendDataUDP(float angle, float error, float output) {
  if (!wifi_connected) return;
  
  // Cria JSON com dados
  StaticJsonDocument<200> doc;
  doc["timestamp"] = millis();
  doc["angle"] = angle;
  doc["error"] = error;
  doc["output"] = output;
  doc["kp"] = Kp;
  doc["ki"] = Ki;
  doc["kd"] = Kd;
  
  // Serializa para string
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Envia via UDP
  udp.beginPacket(pc_ip, udp_port);
  udp.print(jsonString);
  udp.endPacket();
}

// Função para configurar servidor web
void setupWebServer() {
  server.on("/", []() {
    String html = "<!DOCTYPE html><html><head><title>Robo Balanceador</title><meta charset='UTF-8'>";
    html += "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>";
    html += "<style>body{font-family:Arial;margin:20px;background:#f0f0f0;}";
    html += ".container{max-width:1200px;margin:0 auto;background:white;padding:20px;border-radius:10px;}";
    html += ".status{padding:10px;margin:10px 0;border-radius:5px;background:#d4edda;color:#155724;}";
    html += ".controls{display:flex;gap:20px;margin:20px 0;flex-wrap:wrap;}";
    html += ".control-group{flex:1;min-width:200px;padding:15px;background:#f8f9fa;border-radius:5px;}";
    html += ".slider-container{margin:10px 0;}";
    html += ".slider{width:100%;margin:5px 0;}";
    html += ".value{font-weight:bold;color:#007bff;}";
    html += ".charts{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:20px;margin:20px 0;}";
    html += ".chart-container{background:#f8f9fa;padding:15px;border-radius:5px;}";
    html += "canvas{max-height:300px;}";
    html += "@media(max-width:768px){.charts{grid-template-columns:1fr 1fr;}.controls{flex-direction:column;}}";
    html += "</style></head><body><div class='container'>";
    html += "<h1>🚀 Robo Balanceador ESP32</h1>";
    html += "<div class='status'>WiFi Conectado - IP: " + WiFi.localIP().toString() + "</div>";
    
    html += "<div class='controls'>";
    html += "<div class='control-group'><h3>⚙️ Controle PID</h3>";
    html += "<div style='margin:10px 0;'>Kp: <span id='kp-value' class='value'>" + String(Kp, 1) + "</span>";
    html += "<input type='number' id='kp-input' min='0' max='10000' step='0.1' value='" + String(Kp, 1) + "' style='width:120px;padding:8px;border:1px solid #ccc;border-radius:3px;margin-left:10px;' placeholder='Digite Kp e pressione Enter'>";
    html += "</div>";
    html += "<div style='margin:10px 0;'>Ki: <span id='ki-value' class='value'>" + String(Ki, 1) + "</span>";
    html += "<input type='number' id='ki-input' min='0' max='10000' step='0.1' value='" + String(Ki, 1) + "' style='width:120px;padding:8px;border:1px solid #ccc;border-radius:3px;margin-left:10px;' placeholder='Digite Ki e pressione Enter'>";
    html += "</div>";
    html += "<div style='margin:10px 0;'>Kd: <span id='kd-value' class='value'>" + String(Kd, 1) + "</span>";
    html += "<input type='number' id='kd-input' min='0' max='10000' step='0.1' value='" + String(Kd, 1) + "' style='width:120px;padding:8px;border:1px solid #ccc;border-radius:3px;margin-left:10px;' placeholder='Digite Kd e pressione Enter'>";
    html += "</div>";
    html += "</div>";
    
    html += "<div class='control-group'><h3>🎯 Calibração</h3>";
    html += "<div style='margin:10px 0;'>Offset: <span id='offset-value' class='value'>" + String(angle_offset, 1) + "</span>";
    html += "<input type='number' id='offset-input' min='-180' max='180' step='0.01' value='" + String(angle_offset, 1) + "' style='width:120px;padding:8px;border:1px solid #ccc;border-radius:3px;margin-left:10px;' placeholder='Digite offset e pressione Enter'>";
    html += "</div>";
    html += "<button onclick='calibrateNow()' style='width:100%;padding:10px;background:#28a745;color:white;border:none;border-radius:5px;cursor:pointer;margin-top:10px;'>📐 Calibrar Agora (Posição Vertical)</button>";
    html += "</div>";
    
    html += "<div class='control-group'><h3>📊 Dados Atuais</h3>";
    html += "<div id='current-data'>Carregando...</div></div>";
    html += "</div>";
    
    html += "<div class='charts'>";
    html += "<div class='chart-container'><h4>📐 Ângulo (°)</h4><canvas id='angleChart'></canvas></div>";
    html += "<div class='chart-container'><h4>❌ Erro (°)</h4><canvas id='errorChart'></canvas></div>";
    html += "<div class='chart-container'><h4>⚡ Duty Cycle</h4><canvas id='dutyChart'></canvas></div>";
    html += "<div class='chart-container'><h4>🔋 Tensão Motores (V)</h4><canvas id='voltageChart'></canvas></div>";
    html += "</div>";
    
    html += "<script>";
    html += "const maxPoints=50;";
    html += "const timeLabels=Array.from({length:maxPoints},(_,i)=>i-maxPoints+1);";
    
    html += "const angleChart=new Chart(document.getElementById('angleChart'),{";
    html += "type:'line',data:{labels:timeLabels,datasets:[{label:'Ângulo',data:new Array(maxPoints).fill(0),borderColor:'#007bff',tension:0.1}]},";
    html += "options:{responsive:true,scales:{y:{min:-45,max:45}},plugins:{legend:{display:false}}}});";
    
    html += "const errorChart=new Chart(document.getElementById('errorChart'),{";
    html += "type:'line',data:{labels:timeLabels,datasets:[{label:'Erro',data:new Array(maxPoints).fill(0),borderColor:'#dc3545',tension:0.1}]},";
    html += "options:{responsive:true,scales:{y:{min:-45,max:45}},plugins:{legend:{display:false}}}});";
    
    html += "const dutyChart=new Chart(document.getElementById('dutyChart'),{";
    html += "type:'line',data:{labels:timeLabels,datasets:[{label:'Duty',data:new Array(maxPoints).fill(0),borderColor:'#28a745',tension:0.1}]},";
    html += "options:{responsive:true,scales:{y:{min:-255,max:255}},plugins:{legend:{display:false}}}});";
    
    html += "const voltageChart=new Chart(document.getElementById('voltageChart'),{";
    html += "type:'line',data:{labels:timeLabels,datasets:[";
    html += "{label:'Motor1',data:new Array(maxPoints).fill(0),borderColor:'#ff6384',tension:0.1},";
    html += "{label:'Motor2',data:new Array(maxPoints).fill(0),borderColor:'#36a2eb',tension:0.1}";
    html += "]},options:{responsive:true,scales:{y:{min:-8,max:8}},plugins:{legend:{display:true}}}});";
    
    html += "function updateCharts(angle,error,duty,volt1,volt2){";
    html += "angleChart.data.datasets[0].data.shift();angleChart.data.datasets[0].data.push(angle);angleChart.update('none');";
    html += "errorChart.data.datasets[0].data.shift();errorChart.data.datasets[0].data.push(error);errorChart.update('none');";
    html += "dutyChart.data.datasets[0].data.shift();dutyChart.data.datasets[0].data.push(duty);dutyChart.update('none');";
    html += "voltageChart.data.datasets[0].data.shift();voltageChart.data.datasets[0].data.push(volt1);";
    html += "voltageChart.data.datasets[1].data.shift();voltageChart.data.datasets[1].data.push(volt2);voltageChart.update('none');}";
    
    html += "function updatePID(param,value){";
    html += "fetch('/setPID?'+param+'='+value,{method:'GET'}).catch(err=>console.error('Erro PID:',err));}";
    
    html += "function fastUpdatePID(param,value){";
    html += "const xhr=new XMLHttpRequest();xhr.open('GET','/setPID?'+param+'='+value,true);xhr.send();}";
    
    html += "function calibrateNow(){";
    html += "fetch('/calibrate').then(response=>response.text()).then(data=>{";
    html += "document.getElementById('offset-value').textContent=data;";
    html += "document.getElementById('offset-slider').value=data;";
    html += "document.getElementById('offset-input').value=data;";
    html += "alert('Calibração realizada! Offset: '+data+'°');";
    html += "}).catch(error=>console.error('Erro na calibração:',error));}";
    
    html += "function syncKp(value){";
    html += "document.getElementById('kp-value').textContent=value;";
    html += "fastUpdatePID('kp',value);}";
    
    html += "function syncKi(value){";
    html += "document.getElementById('ki-value').textContent=value;";
    html += "fastUpdatePID('ki',value);}";
    
    html += "function syncKd(value){";
    html += "document.getElementById('kd-value').textContent=value;";
    html += "fastUpdatePID('kd',value);}";
    
    html += "function syncOffset(value){";
    html += "document.getElementById('offset-value').textContent=value;";
    html += "updatePID('offset',value);}";
    
    html += "document.getElementById('kp-input').addEventListener('keypress',function(e){if(e.key==='Enter'){syncKp(this.value);}});";
    html += "document.getElementById('ki-input').addEventListener('keypress',function(e){if(e.key==='Enter'){syncKi(this.value);}});";
    html += "document.getElementById('kd-input').addEventListener('keypress',function(e){if(e.key==='Enter'){syncKd(this.value);}});";
    html += "document.getElementById('offset-input').addEventListener('keypress',function(e){if(e.key==='Enter'){syncOffset(this.value);}});";
    html += "function updateData(){";
    html += "fetch('/data').then(response=>response.json()).then(data=>{";
    html += "document.getElementById('current-data').innerHTML=";
    html += "'<strong>Ângulo:</strong> '+data.angle.toFixed(2)+'°<br>'+";
    html += "'<strong>Erro:</strong> '+data.error.toFixed(2)+'°<br>'+";
    html += "'<strong>Duty:</strong> '+data.output.toFixed(0)+'<br>'+";
    html += "'<strong>PID:</strong> Kp='+data.kp+' Ki='+data.ki+' Kd='+data.kd+'<br>'+";
    html += "'<strong>Offset:</strong> '+data.offset.toFixed(1)+'°';";
    html += "updateCharts(data.angle,data.error,data.output,data.motor1_voltage,data.motor2_voltage);";
    html += "}).catch(error=>console.error('Erro:',error));}";
    html += "setInterval(updateData,100);updateData();";
    html += "</script></div></body></html>";
    
    server.send(200, "text/html", html);
  });

  server.on("/data", []() {
    double error = angle_pitch - pid_setpoint;  // Consistente com lógica PID
    
    StaticJsonDocument<300> doc;
    doc["angle"] = angle_pitch;
    doc["error"] = error;
    doc["output"] = pid_output;
    doc["kp"] = Kp;
    doc["ki"] = Ki;
    doc["kd"] = Kd;
    doc["offset"] = angle_offset;
    doc["motor1_voltage"] = motor1_voltage;
    doc["motor2_voltage"] = motor2_voltage;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  server.on("/setPID", []() {
    bool changed = false;
    
    if (server.hasArg("kp")) {
      float new_kp = server.arg("kp").toFloat();
      // Só atualiza se o valor for válido (não zero a menos que explicitamente definido)
      if (new_kp >= 0 && (new_kp > 0 || server.arg("kp") == "0" || server.arg("kp") == "0.0")) {
        Kp = new_kp;
        changed = true;
        Serial.print("Kp atualizado para: "); Serial.println(Kp);
      }
    }
    if (server.hasArg("ki")) {
      float new_ki = server.arg("ki").toFloat();
      // Só atualiza se o valor for válido
      if (new_ki >= 0 && (new_ki > 0 || server.arg("ki") == "0" || server.arg("ki") == "0.0")) {
        Ki = new_ki;
        changed = true;
        Serial.print("Ki atualizado para: "); Serial.println(Ki);
      }
    }
    if (server.hasArg("kd")) {
      float new_kd = server.arg("kd").toFloat();
      // Só atualiza se o valor for válido
      if (new_kd >= 0 && (new_kd > 0 || server.arg("kd") == "0" || server.arg("kd") == "0.0")) {
        Kd = new_kd;
        changed = true;
        Serial.print("Kd atualizado para: "); Serial.println(Kd);
      }
    }
    if (server.hasArg("offset")) {
      angle_offset = server.arg("offset").toFloat();
      Serial.print("Offset atualizado para: "); Serial.println(angle_offset);
    }
    
    // Só reseta o PID se algum parâmetro mudou
    if (changed) {
      resetPID();
      Serial.println("PID resetado devido a mudança de parâmetros");
    }
    
    server.send(200, "text/plain", "OK");
  });

  server.on("/calibrate", []() {
    // Calibra automaticamente: assume que posição atual é 0°
    // Remove o offset atual temporariamente para obter leitura "crua"
    float temp_offset = angle_offset;
    angle_offset = 0;
    
    // Aguarda algumas leituras para estabilizar
    delay(100);
    
    // O novo offset é o negativo do ângulo atual (para zerar)
    angle_offset = -angle_pitch + temp_offset;
    
    server.send(200, "text/plain", String(angle_offset, 1));
  });

  server.begin();
}

// Esta função lê o MPU e calcula o ângulo (Filtro Complementar)
void updateIMU() {
  sensors_event_t a, g, temp;
  
  // 🛡️ Proteção contra falhas I2C
  if (!mpu.getEvent(&a, &g, &temp)) {
    Serial.println("⚠️ Falha na leitura do MPU6050!");
    return; // Mantém último valor válido
  }

  // Calcula o tempo desde a última leitura (dt) em segundos
  unsigned long now = millis();
  float dt = (now - last_loop_time) / 1000.0f;
  // last_loop_time é atualizado no loop principal

  // Calcula o ângulo (Pitch) usando a gravidade (Acelerómetro)
  accelAngleY = atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
  
  // Obtém a velocidade angular (Giroscópio)
  gyroY = g.gyro.y * RAD_TO_DEG;
  
  // Fórmula do Filtro Complementar:
  // Confia 98% no Giroscópio (rápido) e corrige 2% com o Acelerómetro (preciso)
  angle_pitch = 0.98 * (angle_pitch + gyroY * dt) + 0.02 * (accelAngleY);
  
  // Aplica offset de calibração
  angle_pitch += angle_offset;
}

// Esta função move AMBOS os motores com a mesma velocidade/direção
void moveMotors(int speed) {
  // Limita a velocidade ao máximo do PWM (255)
  int pwm_duty = abs(speed);
  if (pwm_duty > 255) pwm_duty = 255;
  
  // Armazena os valores PWM atuais
  motor1_pwm = pwm_duty;
  motor2_pwm = pwm_duty;
  
  // Calcula a tensão aplicada (assumindo alimentação de 8V)
  // Tensão = (PWM / 255) * 8V
  motor1_voltage = (motor1_pwm / 255.0) * 8.0;
  motor2_voltage = (motor2_pwm / 255.0) * 8.0;
  
  // Considera a direção (tensão negativa para movimento reverso)
  if (speed < 0) {
    motor1_voltage = -motor1_voltage;
    motor2_voltage = -motor2_voltage;
  }

  if (speed > 0) { // Robô inclinando para FRENTE, motores rodam para FRENTE
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
    // Motor 2 invertido para rodar na mesma direção que Motor 1
    digitalWrite(M2_IN3, LOW);
    digitalWrite(M2_IN4, HIGH);
  } else if (speed < 0) { // Robô inclinando para TRÁS, motores rodam para TRÁS
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    // Motor 2 invertido para rodar na mesma direção que Motor 1
    digitalWrite(M2_IN3, HIGH);
    digitalWrite(M2_IN4, LOW);
  } else { // Parar (Freio)
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN3, LOW);
    digitalWrite(M2_IN4, LOW);
    motor1_voltage = 0.0;
    motor2_voltage = 0.0;
  }

  // Envia o sinal PWM (velocidade) para os DOIS motores
  ledcWrite(PWM_CHANNEL_1, pwm_duty);  // Motor 1
  ledcWrite(PWM_CHANNEL_2, pwm_duty);  // Motor 2
  
  // Debug: imprime o status dos motores ocasionalmente
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 500) {  // A cada 500ms
    Serial.print("PWM M1: "); Serial.print(pwm_duty);
    Serial.print(" | M2: "); Serial.print(pwm_duty);
    Serial.print(" | Speed: "); Serial.println(speed);
    last_debug = millis();
  }
}

// =================================================================
//  5. SETUP (Função de Inicialização)
// =================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n🚀 Iniciando Robô Balanceador (WiFi + Serial)...");
  Serial.println("📝 Configure WiFi no topo do código antes de usar bateria!");

  // --- Conecta WiFi ---
  connectWiFi();

  // --- Configura servidor web ---
  if (wifi_connected) {
    setupWebServer();
    Serial.print("🌐 Servidor web: http://");
    Serial.println(WiFi.localIP());
  }

  // --- Inicializa MPU ---
  Wire.begin(MPU_SDA, MPU_SCL);
  if (!mpu.begin()) {
    Serial.println("!!! FALHA AO ENCONTRAR MPU-6050 !!!");
    Serial.println("Verifique a fiação (Pinos 21, 22).");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("✅ MPU-6050 OK!");

  // --- Inicializa Motores (PWM) ---
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT); pinMode(M2_IN4, OUTPUT);
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(M1_ENA_PWM, PWM_CHANNEL_1);
  ledcAttachPin(M2_ENB_PWM, PWM_CHANNEL_2);
  Serial.println("✅ Motores OK!");

  // --- Inicializa PID Customizado ---
  resetPID();                           // Inicializa variáveis do PID
  last_pid_time = millis();             // Define o tempo inicial do PID
  Serial.println("✅ PID Customizado OK!");
  
  Serial.println("\n--- Loop de Controlo Iniciado ---");
  Serial.println("Ang | Err | Out | Status");

  last_loop_time = millis(); // Define o tempo inicial
  last_print_time = millis();
}

// =================================================================
//  6. LOOP PRINCIPAL (Gere o controlo e imprime dados)
// =================================================================
void loop() {
  unsigned long now = millis();

  // --- Processa requisições web ---
  if (wifi_connected) {
    server.handleClient();
  }

  // --- Loop de Controlo (Rápido, 100Hz) ---
  // Roda o loop de PID apenas a cada 10ms
  if (now - last_loop_time >= PID_LOOP_INTERVAL_MS) {
    // 1. Lê o sensor e calcula o ângulo
    updateIMU();

    // 2. Calcula o PID customizado
    pid_output = computePID(angle_pitch);

    // 3. Move os motores com a potência calculada pelo PID
    moveMotors(pid_output);

    last_loop_time = now; // Atualiza o tempo do loop de controlo
  }

  // --- Loop de Impressão e Envio UDP (Lento, 10Hz) ---
  // Imprime os dados no monitor serial e envia via UDP a cada 100ms
  if (now - last_print_time >= PRINT_INTERVAL_MS) {
    double error =   // Consistente com lógica PID
    
    // Imprime no Serial Monitor
    Serial.printf("%.2f | %.2f | %.0f | %s\n", 
                  angle_pitch, error, pid_output, 
                  wifi_connected ? "WiFi✅" : "Offline");
    
    // Envia via UDP se WiFi conectado
    sendDataUDP(angle_pitch, error, pid_output);

    last_print_time = now; // Atualiza o tempo de impressão
  }
}