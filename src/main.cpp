#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <WebServer.h>

// =================================================================
//  1. CONFIGURAÇÕES DE AJUSTE 
// =================================================================
const char* password = "";
const char* ssid = "R3_SATIRO";     
    
// const char* password = "icomputacaoufal"; 
// const char* ssid = "IC-ALUNOS";     
const char* pc_ip = "192.168.1.14";     
const int udp_port = 8888;

// Ganhos iniciais (serão ajustados via Web)
double Kp = 0.0;   // Ganho proporcional para resposta rápida
double Ki = 0.0;    // Ganho integral pequeno para eliminar erro
double Kd = 0.0;    // Ganho derivativo para amortecimento

// Setpoint (ângulo desejado, 0.0 = vertical)
double pid_setpoint = 0.0;

// Offset inicial (será ajustado via Web)
double angle_offset = 0.0;  // Começar com 0.0 para calibração manual

// =================================================================
//  2. PINOS E HARDWARE 
// =================================================================
const int M1_IN1 = 19;
const int M1_IN2 = 18;
const int M1_ENA_PWM = 5;
const int M2_IN3 = 17;
const int M2_IN4 = 16;
const int M2_ENB_PWM = 4;

const int MPU_SDA = 21;
const int MPU_SCL = 22;

const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8; 
const int PWM_CHANNEL_1 = 0;
const int PWM_CHANNEL_2 = 1;

Adafruit_MPU6050 mpu;

WiFiUDP udp;
WebServer server(80);  
bool wifi_connected = false;

// =================================================================
//  3. VARIÁVEIS GLOBAIS DO PID E SENSOR
// =================================================================

double angle_pitch = 0.0;    // ÂNGULO FINAL (COM OFFSET)
double raw_angle = 0.0;      // ÂNGULO BRUTO FILTRADO (SEM OFFSET)
double pid_output = 0.0;     

double motor1_voltage = 0.0; 
double motor2_voltage = 0.0; 
int motor1_pwm = 0;         
int motor2_pwm = 0;         

double last_error = 0.0;      
double integral = 0.0;        
double output_min = -8.0;     // Limite mínimo: -8V (tensão da bateria)
double output_max = 8.0;      // Limite máximo: +8V (tensão da bateria)     
unsigned long last_pid_time = 0; 

// --- Variáveis de Sensor Fusion (Filtro Complementar) ---
float accelAngleY = 0.0;
float gyroY = 0.0;
unsigned long last_loop_time = 0;

const int PID_LOOP_INTERVAL_MS = 10;
const int PRINT_INTERVAL_MS = 100;
unsigned long last_print_time = 0;

// =================================================================
//  4. FUNÇÃO PID CUSTOMIZADA (CORRIGIDA)
// =================================================================

// Função PID customizada com anti-windup avançado
double computePID(double input) {
  unsigned long now = millis();

  // --- CORREÇÃO DO BUG DO 'dt' ---
  // O 'dt' deve ser o tempo real que passou, em segundos.
  double dt = (now - last_pid_time) / 1000.0;

  // Evita divisão por zero na primeira execução
  if (last_pid_time == 0 || dt <= 0) {
    last_pid_time = now;
    return 0.0;
  }
  // Calcula o erro atual
  double error = pid_setpoint - input;
  // === TERMO PROPORCIONAL ===
  double proportional = Kp * error;

  // Anti-windup (só integra se a saída não estiver saturada)
  if (pid_output >= output_max && error > 0) {
    integral += 0;
  } else if (pid_output <= output_min && error < 0) {
    integral += 0;
  } else {
    integral += error * dt;
  }
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

  // Zera o integral se o erro cruzar o zero
  if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
    integral *= 0.0; 
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

void connectWiFi() {
  Serial.println("\n🔗 Iniciando conexão WiFi...");
  Serial.print("📶 SSID: "); Serial.println(ssid);
  Serial.print("🔑 Senha: "); 
  for(int i = 0; i < strlen(password); i++) Serial.print("*");
  Serial.println();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("⏳ Conectando");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
    
    if (attempts % 10 == 0) {
      Serial.print("\n📊 Status WiFi: ");
      switch(WiFi.status()) {
        case WL_NO_SSID_AVAIL: Serial.print("SSID não encontrado"); break;
        case WL_CONNECT_FAILED: Serial.print("Falha na conexão"); break;
        case WL_CONNECTION_LOST: Serial.print("Conexão perdida"); break;
        case WL_DISCONNECTED: Serial.print("Desconectado"); break;
        default: Serial.print("Conectando..."); break;
      }
      Serial.print(" (Tentativa "); Serial.print(attempts); Serial.println("/30)");
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println("\n✅ WiFi conectado com sucesso!");
    Serial.print("📍 IP do ESP32: "); Serial.println(WiFi.localIP());
    Serial.print("📡 Gateway: "); Serial.println(WiFi.gatewayIP());
    Serial.print("🔗 DNS: "); Serial.println(WiFi.dnsIP());
    Serial.print("📶 RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    Serial.print("📤 Enviando dados UDP para: "); Serial.print(pc_ip); Serial.print(":"); Serial.println(udp_port);
    udp.begin(udp_port);
  } else {
    wifi_connected = false;
    Serial.println("\n❌ Falha na conexão WiFi!");
    Serial.println("🔧 Verifique:");
    Serial.println("   - Nome da rede (SSID)");
    Serial.println("   - Senha do WiFi");
    Serial.println("   - Sinal da rede");
    Serial.println("   - Compatibilidade 2.4GHz");
    Serial.println("⚠️ Continuando em modo offline...");
  }
}

void sendDataUDP(float angle, float error, float output) {
  if (!wifi_connected) return;
  StaticJsonDocument<200> doc;
  doc["timestamp"] = millis();
  doc["angle"] = angle;
  doc["error"] = error;
  doc["output"] = output;
  doc["kp"] = Kp;
  doc["ki"] = Ki;
  doc["kd"] = Kd;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  udp.beginPacket(pc_ip, udp_port);
  udp.print(jsonString);
  udp.endPacket();
}

// servidor web
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
    html += ".charts{display:grid;grid-template-columns:1fr 1fr;gap:20px;margin:20px 0;}";
    html += ".chart-container{background:#f8f9fa;padding:15px;border-radius:5px;}";
    html += "canvas{max-height:300px;}";
    html += "@media(max-width:768px){.charts{grid-template-columns:1fr;}.controls{flex-direction:column;}}";
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
    html += "<p style='margin:10px 0;color:#666;font-size:14px;'>Posicione o robô na posição desejada e clique no botão abaixo para definir como 0°</p>";
    html += "<button onclick='setCurrentAsZero()' style='width:100%;padding:15px;background:#ff6b35;color:white;border:none;border-radius:5px;cursor:pointer;margin:10px 0;font-size:16px;font-weight:bold;'>🎯 DEFINIR POSIÇÃO ATUAL COMO 0°</button>";
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
    
    html += "let chartUpdateCounter = 0;";
    html += "function updateCharts(angle,error,duty,volt1,volt2){";
    html += "angleChart.data.datasets[0].data.shift();angleChart.data.datasets[0].data.push(angle);angleChart.update('none');";
    html += "errorChart.data.datasets[0].data.shift();errorChart.data.datasets[0].data.push(error);errorChart.update('none');";
    html += "dutyChart.data.datasets[0].data.shift();dutyChart.data.datasets[0].data.push(duty);dutyChart.update('none');";
    html += "voltageChart.data.datasets[0].data.shift();voltageChart.data.datasets[0].data.push(volt1);";
    html += "voltageChart.data.datasets[1].data.shift();voltageChart.data.datasets[1].data.push(volt2);voltageChart.update('none');}";
    
    html += "function updatePID(param,value){";
    html += "fetch('/setPID?'+param+'='+value,{method:'GET'}).catch(err=>console.error('Erro PID:',err));}";
    
    html += "function fastUpdatePID(param,value){";
    html += "fetch('/setPID?'+param+'='+value,{method:'GET'}).catch(err=>console.error('Erro PID:',err));}";
    
    html += "function setCurrentAsZero(){";
    html += "fetch('/setZero').then(response=>response.text()).then(data=>{";
    html += "alert('✅ Posição atual definida como 0°!');";
    html += "}).catch(error=>{console.error('Erro:',error);alert('❌ Erro ao calibrar!');});}";
    
    html += "function syncKp(value){";
    html += "document.getElementById('kp-value').textContent=value;";
    html += "fastUpdatePID('kp',value);}";
    
    html += "function syncKi(value){";
    html += "document.getElementById('ki-value').textContent=value;";
    html += "fastUpdatePID('ki',value);}";
    
    html += "function syncKd(value){";
    html += "document.getElementById('kd-value').textContent=value;";
    html += "fastUpdatePID('kd',value);}";
    
    html += "document.getElementById('kp-input').addEventListener('keypress',function(e){if(e.key==='Enter'){syncKp(this.value);}});";
    html += "document.getElementById('ki-input').addEventListener('keypress',function(e){if(e.key==='Enter'){syncKi(this.value);}});";
    html += "document.getElementById('kd-input').addEventListener('keypress',function(e){if(e.key==='Enter'){syncKd(this.value);}});";
    html += "let isFirstLoad = true;";
    html += "function updateData(){";
    html += "fetch('/data').then(response=>response.json()).then(data=>{";
    html += "document.getElementById('current-data').innerHTML=";
    html += "'<strong>Ângulo:</strong> '+data.angle.toFixed(2)+'°<br>'+";
    html += "'<strong>Erro:</strong> '+data.error.toFixed(2)+'°<br>'+";
    html += "'<strong>Duty:</strong> '+data.output.toFixed(0)+'<br>'+";
    html += "'<strong>PID:</strong> Kp='+data.kp+' Ki='+data.ki+' Kd='+data.kd+'<br>'+";
    html += "'<strong>Offset:</strong> '+data.offset.toFixed(1)+'°';";
    html += "document.getElementById('kp-value').textContent=data.kp.toFixed(1);";
    html += "document.getElementById('ki-value').textContent=data.ki.toFixed(1);";
    html += "document.getElementById('kd-value').textContent=data.kd.toFixed(1);";
    html += "if(isFirstLoad){";
    html += "document.getElementById('kp-input').value=data.kp.toFixed(1);";
    html += "document.getElementById('ki-input').value=data.ki.toFixed(1);";
    html += "document.getElementById('kd-input').value=data.kd.toFixed(1);";
    html += "isFirstLoad=false;}";
    html += "chartUpdateCounter++;if(chartUpdateCounter>=2){";
    html += "updateCharts(data.angle,data.error,data.output,data.motor1_voltage,data.motor2_voltage);chartUpdateCounter=0;}";
    html += "}).catch(error=>console.error('Erro:',error));}";
    html += "setInterval(updateData,150);updateData();";
    html += "</script></div></body></html>";
    
    server.send(200, "text/html", html);
  });

  server.on("/data", []() {
    double error = angle_pitch - pid_setpoint;  
    
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
      if (new_kp >= 0 && (new_kp > 0 || server.arg("kp") == "0" || server.arg("kp") == "0.0")) {
        Kp = new_kp;
        changed = true;
      }
    }
    if (server.hasArg("ki")) {
      float new_ki = server.arg("ki").toFloat();
      if (new_ki >= 0 && (new_ki > 0 || server.arg("ki") == "0" || server.arg("ki") == "0.0")) {
        Ki = new_ki;
        changed = true;
      }
    }
    if (server.hasArg("kd")) {
      float new_kd = server.arg("kd").toFloat();
      if (new_kd >= 0 && (new_kd > 0 || server.arg("kd") == "0" || server.arg("kd") == "0.0")) {
        Kd = new_kd;
        changed = true;
      }
    }
    
    if (changed) {
      resetPID();
    }
    
    server.send(200, "text/plain", "OK");
  });

  server.on("/setZero", []() {
    Serial.print("🎯 ANTES - Ângulo: ");
    Serial.print(angle_pitch, 2);
    Serial.print("° | Offset: ");
    Serial.println(angle_offset, 2);
    
    // --- CORREÇÃO DA LÓGICA DE CALIBRAÇÃO ---
    // O novo offset "absorve" o erro atual.
    // new_offset = old_offset - angle_pitch_total_atual
    angle_offset = angle_offset - angle_pitch;
    
    Serial.print("✅ DEPOIS - Novo offset: ");
    Serial.print(angle_offset, 2);
    Serial.println("°");
    
    resetPID();
    server.send(200, "text/plain", "OK");
  });

  server.begin();
}

// =================================================================
//  5.4. FUNÇÃO DA IMU (CORRIGIDA)
// =================================================================

void updateIMU() {
  sensors_event_t a, g, temp;
  
  if (!mpu.getEvent(&a, &g, &temp)) {
    Serial.println("⚠️ Falha na leitura do MPU6050!");
    return;
  }

  unsigned long now = millis();
  float dt = (now - last_loop_time) / 1000.0f;   // Delta time real em segundos
  if (dt <= 0) dt = (float)PID_LOOP_INTERVAL_MS / 1000.0f; // Evita dt 0
  
  // --- CORREÇÃO DO BUG DOS EIXOS ---
  // Para Pitch (frente/trás), usamos Acelerômetro X/Z e Giroscópio Y
  
  // 1. Ângulo do Acelerômetro (Eixos X e Z)
  float raw_accel_angle = atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
  
  // 2. Velocidade angular do Giroscópio (Eixo Y)
  gyroY = g.gyro.y * RAD_TO_DEG; // Estava g.gyro.x
  
  // --- CORREÇÃO DO BUG DO FILTRO/OFFSET ---
  // O filtro complementar opera sobre o 'raw_angle'
  // O 'angle_offset' é somado apenas no final.
  
  // Filtro: 95% giroscópio + 5% acelerômetro
  raw_angle = 0.95 * (raw_angle + gyroY * dt) + 0.05 * (raw_accel_angle);
  
  // O ângulo final (pitch) é o ângulo filtrado + o offset de calibração
  angle_pitch = raw_angle + angle_offset;
  
  // Removemos a "zona morta" e o "filtro de acelerômetro"
  // que estavam no seu código original, pois o filtro complementar já faz isso.
  
  // Limita o ângulo para ±90 graus
  if (angle_pitch > 90.0) angle_pitch = 90.0;
  if (angle_pitch < -90.0) angle_pitch = -90.0;
}


// =================================================================
//  5.5. FUNÇÃO DO MOTOR (SEM MUDANÇAS)
// =================================================================

void moveMotors(double speed) {
  // Converte tensão (speed) para PWM (0-255)
  // speed vem em volts (-8V a +8V), precisa converter para PWM (0-255)
  int pwm_duty = abs(speed) * 255.0 / 8.0;   // Converte V para PWM
  if (pwm_duty > 255) pwm_duty = 255;
  
  // ZONA MORTA: PWM mínimo para superar atrito dos motores
  if (pwm_duty > 0 && pwm_duty < 50) {
    pwm_duty = 50;   // PWM mínimo para mover os motores
  }
  
  motor1_pwm = pwm_duty;
  motor2_pwm = pwm_duty;
  
  // A tensão desejada é o próprio speed (já em volts)
  motor1_voltage = abs(speed);   // Tensão desejada (0 a 8V)
  motor2_voltage = abs(speed);   // Tensão desejada (0 a 8V)
  if (speed < 0) {
    motor1_voltage = -motor1_voltage;
    motor2_voltage = -motor2_voltage;
  }

  // Lógica de direção - ambos motores na mesma direção
  if (speed > 0) {
    // Ambos motores: FRENTE
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN3, HIGH);  // Mesma direção que M1
    digitalWrite(M2_IN4, LOW);   // Mesma direção que M1
  } else if (speed < 0) {
    // Ambos motores: TRÁS
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    digitalWrite(M2_IN3, LOW);   // Mesma direção que M1
    digitalWrite(M2_IN4, HIGH);  // Mesma direção que M1
  } else {
    // Ambos param (Freio)
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN3, LOW);
    digitalWrite(M2_IN4, LOW);
    motor1_voltage = 0.0;
    motor2_voltage = 0.0;
  }

  ledcWrite(PWM_CHANNEL_1, pwm_duty);
  ledcWrite(PWM_CHANNEL_2, pwm_duty);
}

// =================================================================
//  5. SETUP (Função de Inicialização)
// =================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n🚀 Iniciando Robô Balanceador (WiFi + Serial)...");
  Serial.println("📝 Configure WiFi no topo do código antes de usar bateria!");

  connectWiFi();

  if (wifi_connected) {
    setupWebServer();
    Serial.print("🌐 Servidor web: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("⚠️ Modo offline: apenas controle via Serial Monitor");
  }

  Wire.begin(MPU_SDA, MPU_SCL);
  if (!mpu.begin()) {
    Serial.println("!!! FALHA AO ENCONTRAR MPU-6050 !!!");
    Serial.println("Verifique a fiação (Pinos 21, 22).");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("✅ MPU-6050 OK!");

  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT); pinMode(M2_IN4, OUTPUT);
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION); 
  ledcAttachPin(M1_ENA_PWM, PWM_CHANNEL_1);
  ledcAttachPin(M2_ENB_PWM, PWM_CHANNEL_2);
  Serial.println("✅ Motores OK!");

  resetPID();
  last_pid_time = millis();
  Serial.println("✅ PID Customizado OK!");
  
  Serial.println("\n--- Loop de Controlo Iniciado ---");
  Serial.println("Ang | Err | Out | Status");

  last_loop_time = millis();
  last_print_time = millis();
}

// =================================================================
//  6. LOOP PRINCIPAL (Gere o controlo e imprime dados)
// =================================================================
void loop() {
  unsigned long now = millis();

  if (wifi_connected) {
    server.handleClient();
  }

  // Loop de controle principal (Executa a cada ~10ms)
  if (now - last_loop_time >= PID_LOOP_INTERVAL_MS) {
    updateIMU();
    pid_output = computePID(angle_pitch);
    moveMotors(pid_output);
    last_loop_time = now; // Reseta o temporizador do loop de controle
  }

  // Loop de impressão/envio (Executa a cada ~500ms para não atrasar)
  if (now - last_print_time >= 500) {  
    double error = pid_setpoint - angle_pitch;
    
    Serial.printf("Ang:%.1f° | Err:%.1f° | Out:%.0f | %s\n", 
                  angle_pitch, error, pid_output, 
                  wifi_connected ? "WiFi✅" : "Offline");
    
    // Só envie UDP se estiver conectado
    if (wifi_connected) {
      sendDataUDP(angle_pitch, error, pid_output);
    }

    last_print_time = now; // Reseta o temporizador de impressão
  }
}

