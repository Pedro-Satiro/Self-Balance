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
    
// const char* password = "icomputacaoufal"; 
// const char* ssid = "IC-ALUNOS";     
const char* pc_ip = "192.168.1.14";     
const int udp_port = 8888;

// Ganhos iniciais (serão ajustados via Web)
double Kp = 0.0;
double Ki = 0.0;  
double Kd = 0.0;

// Setpoint (ângulo desejado, 0.0 = vertical)
double pid_setpoint = 0.0;

// Offset inicial (será ajustado via Web)
double angle_offset = 1.81;

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

double angle_pitch = 0.0;   
double pid_output = 0.0;    

double motor1_voltage = 0.0; 
double motor2_voltage = 0.0; 
int motor1_pwm = 0;         
int motor2_pwm = 0;         

double last_error = 0.0;      
double integral = 0.0;        
double output_min = -6.0;   
double output_max = 6.0;    
unsigned long last_pid_time = 0; 

// --- Variáveis de Sensor Fusion (Filtro Complementar) ---
float accelAngleY = 0.0;
float gyroY = 0.0;
unsigned long last_loop_time = 0;

const int PID_LOOP_INTERVAL_MS = 10;
const int PRINT_INTERVAL_MS = 100;
unsigned long last_print_time = 0;

// =================================================================
//  4. FUNÇÃO PID CUSTOMIZADA
// =================================================================
double dt = 0.1;

// Função PID customizada com anti-windup avançado
double computePID(double input) {
  unsigned long now = millis();


  // Evita divisão por zero na primeira execução
  if (last_pid_time == 0 || dt <= 0) {
    last_pid_time = now;
    return 0.0;
  }
// Calcula o erro atual
  double error = pid_setpoint - input;
// === TERMO PROPORCIONAL ===
  double proportional = Kp * error;

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

if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
    integral *= 0.0; // Reduz a integral para zero
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
    
    // --- ALTERAÇÃO DO CSS DO GRÁFICO (2x2) ---
    html += ".charts{display:grid;grid-template-columns:1fr 1fr;gap:20px;margin:20px 0;}"; // 2 colunas
    
    html += ".chart-container{background:#f8f9fa;padding:15px;border-radius:5px;}";
    html += "canvas{max-height:300px;}";
    
    // --- ALTERAÇÃO DO CSS (Mobile) ---
    html += "@media(max-width:768px){.charts{grid-template-columns:1fr;}.controls{flex-direction:column;}}"; // 1 coluna no mobile
    
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
        Serial.print("Kp atualizado para: "); Serial.println(Kp);
      }
    }
    if (server.hasArg("ki")) {
      float new_ki = server.arg("ki").toFloat();
      if (new_ki >= 0 && (new_ki > 0 || server.arg("ki") == "0" || server.arg("ki") == "0.0")) {
        Ki = new_ki;
        changed = true;
        Serial.print("Ki atualizado para: "); Serial.println(Ki);
      }
    }
    if (server.hasArg("kd")) {
      float new_kd = server.arg("kd").toFloat();
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
    
    if (changed) {
      resetPID();
      Serial.println("PID resetado devido a mudança de parâmetros");
    }
    
    server.send(200, "text/plain", "OK");
  });

  server.on("/calibrate", []() {
    float temp_offset = angle_offset;
    angle_offset = 0;
    
    delay(100);
    
    angle_offset = -angle_pitch + temp_offset;
    
    server.send(200, "text/plain", String(angle_offset, 1));
  });
  server.begin();
}

void updateIMU() {
  sensors_event_t a, g, temp;
  
  if (!mpu.getEvent(&a, &g, &temp)) {
    Serial.println("⚠️ Falha na leitura do MPU6050!");
    return;
  }

  unsigned long now = millis();
  float dt = 0.1; 
  accelAngleY = atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
  
  gyroY = g.gyro.y * RAD_TO_DEG;
  
  angle_pitch = 0.98 * (angle_pitch + gyroY * dt) + 0.02 * (accelAngleY);
  
  angle_pitch += angle_offset;
}

// =================================================================
//  5.5. FUNÇÃO DO MOTOR (CORRIGIDA)
// =================================================================

void moveMotors(int speed) {
  int pwm_duty = abs(speed);
  if (pwm_duty > 255) pwm_duty = 255;
  
  motor1_pwm = pwm_duty;
  motor2_pwm = pwm_duty;
  
  motor1_voltage = (motor1_pwm / 255.0) * 8.0;
  motor2_voltage = (motor2_pwm / 255.0) * 8.0;
  if (speed < 0) {
    motor1_voltage = -motor1_voltage;
    motor2_voltage = -motor2_voltage;
  }

  // Lógica de direção corrigida
  if (speed > 0) {
    // Ambos para FRENTE
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN3, HIGH); 
    digitalWrite(M2_IN4, LOW);  
  } else if (speed < 0) {
    // Ambos para TRÁS
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    digitalWrite(M2_IN3, LOW);  
    digitalWrite(M2_IN4, HIGH); 
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

  // Loop de impressão/envio (Executa a cada ~100ms)
  if (now - last_print_time >= PRINT_INTERVAL_MS) {
    double error = pid_setpoint - angle_pitch;
    
    Serial.printf("%.2f | %.2f | %.0f | %s\n", 
                  angle_pitch, error, pid_output, 
                  wifi_connected ? "WiFi✅" : "Offline");
    
    // Só envie UDP se estiver conectado
    if (wifi_connected) {
      sendDataUDP(angle_pitch, error, pid_output);
    }

    last_print_time = now; // Reseta o temporizador de impressão
  }
}