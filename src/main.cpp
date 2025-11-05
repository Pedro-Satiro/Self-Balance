#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

// --- Configuração de Rede ---
const char* ssid = "R3FIBRA_PEDRO_SATIRO";
const char* password = "pedro3016";

// --- Configuração MQTT ---
const char* mqtt_server = "192.168.1.100";  // IP do broker MQTT (seu computador)
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_SelfBalance";
const char* mqtt_topic_data = "selfbalance/data";      // Tópico para enviar dados
const char* mqtt_topic_control = "selfbalance/control"; // Tópico para receber comandos

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

// --- Pinos de Controle do L298N (Motor 1) ---
const int M1_IN1 = 19;
const int M1_IN2 = 18;
const int M1_ENA_PWM = 5;

// --- Pinos de Controle do L298N (Motor 2) ---
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

// --- Variáveis Globais do PID ---
double pid_setpoint = 0.0; // Nosso objetivo é 0 graus (reto)
double angle_pitch = 0.0;
double pid_output = 0.0;
double Kp = 10.0, Ki = 0.5, Kd = 1.0; // Valores iniciais, você vai ajustar!

// Objeto PID
PID pid(&angle_pitch, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);

// --- Variáveis de Sensor Fusion (Filtro Complementar) ---
float accelAngleY = 0.0;
float gyroY = 0.0;
unsigned long last_loop_time = 0;

// --- Timers para tarefas (não-bloqueante) ---
unsigned long last_mqtt_send_time = 0;
const int PID_LOOP_INTERVAL_MS = 10; // Roda o PID a 100Hz
const int MQTT_SEND_INTERVAL_MS = 100; // Envia dados via MQTT a 10Hz

// =================================================================
//  Funções MQTT
// =================================================================

void connectToMQTT() {
  while (!mqtt_client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    if (mqtt_client.connect(mqtt_client_id)) {
      Serial.println(" conectado!");
      // Inscreve-se no tópico de controle para receber comandos
      mqtt_client.subscribe(mqtt_topic_control);
      Serial.println("Inscrito no tópico: " + String(mqtt_topic_control));
    } else {
      Serial.print(" falhou, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" tentando novamente em 5 segundos");
      delay(5000);
    }
  }
}

void onMQTTMessage(char* topic, byte* payload, unsigned int length) {
  // Converte payload para string
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Mensagem recebida [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  // Parse JSON para comandos de controle PID
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.print("Erro ao fazer parse do JSON: ");
    Serial.println(error.c_str());
    return;
  }

  // Atualiza parâmetros PID se fornecidos
  if (doc["kp"].is<double>()) {
    Kp = doc["kp"];
    pid.SetTunings(Kp, Ki, Kd);
    Serial.print("Novo Kp: "); Serial.println(Kp);
  }
  if (doc["ki"].is<double>()) {
    Ki = doc["ki"];
    pid.SetTunings(Kp, Ki, Kd);
    Serial.print("Novo Ki: "); Serial.println(Ki);
  }
  if (doc["kd"].is<double>()) {
    Kd = doc["kd"];
    pid.SetTunings(Kp, Ki, Kd);
    Serial.print("Novo Kd: "); Serial.println(Kd);
  }
}

void publishSensorData() {
  // Cria JSON com dados do robô
  JsonDocument doc;
  doc["timestamp"] = millis();
  doc["angle"] = angle_pitch;
  doc["error"] = pid_setpoint - angle_pitch;
  doc["duty_cycle"] = pid_output;
  doc["kp"] = Kp;
  doc["ki"] = Ki;
  doc["kd"] = Kd;
  doc["setpoint"] = pid_setpoint;

  String json_string;
  serializeJson(doc, json_string);
  
  // Publica no tópico MQTT
  if (mqtt_client.publish(mqtt_topic_data, json_string.c_str())) {
    // Serial.println("Dados publicados: " + json_string);
  } else {
    Serial.println("Falha ao publicar dados MQTT");
  }
}


// =================================================================
//  Funções de Controle do Robô
// =================================================================

void updateIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calcula o delta time (dt) em segundos
  unsigned long now = millis();
  float dt = (now - last_loop_time) / 1000.0f;
  last_loop_time = now;

  // --- Filtro Complementar ---
  // Acel: Calcula o ângulo Y (pitch) baseado na gravidade
  accelAngleY = atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
  
  // Gyro: Integra a velocidade angular
  gyroY = g.gyro.y * RAD_TO_DEG;
  
  // Fórmula Mágica: 0.98 (confia no gyro) + 0.02 (corrige com o acel)
  angle_pitch = 0.98 * (angle_pitch + gyroY * dt) + 0.02 * (accelAngleY);
}

// Função para mover os DOIS motores com o mesmo sinal
// speed: -255 (Trás) a 255 (Frente)
void moveMotors(int speed) {
  // Satura a velocidade no limite do PWM (0-255)
  int pwm_duty = abs(speed);
  if (pwm_duty > 255) {
    pwm_duty = 255;
  }

  if (speed > 0) {
    // Para Frente
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN3, HIGH);
    digitalWrite(M2_IN4, LOW);
  } else if (speed < 0) {
    // Para Trás
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    digitalWrite(M2_IN3, LOW);
    digitalWrite(M2_IN4, HIGH);
  } else {
    // Parar (Freio)
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN3, LOW);
    digitalWrite(M2_IN4, LOW);
  }

  // Envia o sinal PWM para os DOIS motores
  ledcWrite(PWM_CHANNEL_1, pwm_duty);
  ledcWrite(PWM_CHANNEL_2, pwm_duty);
}


// =================================================================
//  Lógica MQTT (substituindo WebSocket)
// =================================================================

// =================================================================
//  SETUP
// =================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nIniciando Robô Balanceador...");

  // --- Inicializa MPU ---
  Wire.begin(MPU_SDA, MPU_SCL);
  if (!mpu.begin()) {
    Serial.println("Falha ao encontrar o sensor MPU-6050!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("MPU-6050 OK!");

  // --- Inicializa Motores (PWM) ---
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(M1_ENA_PWM, PWM_CHANNEL_1);
  ledcAttachPin(M2_ENB_PWM, PWM_CHANNEL_2);
  Serial.println("Motores OK!");

  // --- Inicializa PID ---
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(PID_LOOP_INTERVAL_MS);
  pid.SetOutputLimits(-255, 255); // Limita o Duty Cycle
  Serial.println("PID OK!");

  // --- Inicializa WiFi (Modo Station) ---
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi conectado! IP: ");
  Serial.println(WiFi.localIP());

  // --- Inicializa MQTT ---
  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(onMQTTMessage);
  connectToMQTT();
  Serial.println("MQTT OK!");

  last_loop_time = millis();
}


// =================================================================
//  LOOP PRINCIPAL
// =================================================================
void loop() {
  unsigned long now = millis();

  // Mantém conexão MQTT ativa
  if (!mqtt_client.connected()) {
    connectToMQTT();
  }
  mqtt_client.loop();

  // --- Loop de Controle (Rápido) ---
  if (now - last_loop_time >= PID_LOOP_INTERVAL_MS) {
    // 1. Lê o sensor e calcula o ângulo
    updateIMU();

    // 2. Calcula o PID (pid_output será atualizado)
    pid.Compute();

    // 3. Move os motores com o valor do PID
    moveMotors(pid_output);

    last_loop_time = now;
  }

  // --- Loop de Comunicação MQTT (Mais Lento) ---
  if (now - last_mqtt_send_time >= MQTT_SEND_INTERVAL_MS) {
    last_mqtt_send_time = now;
    publishSensorData();
  }
}