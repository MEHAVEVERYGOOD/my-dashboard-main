#include <WiFiS3.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h> // [เพิ่มใหม่] โหลด Library PID

// --- 1. ตั้งค่า WiFi และ MQTT ---
const char* ssid = "1T1M";
const char* password = "12345678";
const char* mqtt_server = "broker.hivemq.com";

// --- 2. กำหนดพินอุปกรณ์เดิม ---
const int XKC_1_PIN = 2;      
const int XKC_2_PIN = 3;      
const int PUMP_PIN = 4;       
const int TEMP_PIN = 5;       
const int TRIG_PIN = 6;  
const int ECHO_PIN = 7;
const int MOTOR_IN1 = 8;      
const int MOTOR_IN2 = 9;      
const int MIX_PUMP_PIN = 10;  

// พินสำหรับ Relay x2 และ DS18B20
const int EXTRA_RELAY_1_PIN = 11; // Heater 1
const int EXTRA_RELAY_2_PIN = 12; // Heater 2
const int TEMP2_PIN = 13;         

OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
OneWire oneWire2(TEMP2_PIN);
DallasTemperature sensors2(&oneWire2);
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastTempMsg = 0;
int lastSentState = -1;
int lastSentPump = -1;
bool isGrease = false; 

float currentDistance = 100.0; 
const float SAFETY_LIMIT = 4.0; 
unsigned long greaseStartTime = 0; 
bool isTiming = false; 
const unsigned long AUTO_DELAY_MS = 5000; 
bool isMixPumpRunning = false;
unsigned long mixPumpStartTime = 0;
const unsigned long MIX_PUMP_DURATION = 5000; 

// ==========================================
// [เพิ่มใหม่] ตัวแปรสำหรับระบบ PID Heater
// ==========================================
// ล็อคอุณหภูมิเป้าหมายไว้ที่ 58 องศา (เพื่อให้แกว่งอยู่ช่วง 55-60)
double Setpoint1 = 58.0, Input1 = 25.0, Output1 = 0.0;
double Setpoint2 = 58.0, Input2 = 25.0, Output2 = 0.0;

// ค่าจูน PID (สามารถปรับแต่งได้ถ้ามันร้อนเกินหรือร้อนช้าไป)
double Kp = 30.0, Ki = 0.5, Kd = 2.0; 

// สร้างออบเจกต์ PID
PID myPID1(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000; // หน้าต่างเวลาทำงาน 5 วินาที (5000 ms)
unsigned long windowStartTime;
// ==========================================

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

void sendPumpStatus() {
  int pinVal = digitalRead(PUMP_PIN);
  int logicalStatus = (pinVal == LOW) ? 1 : 0; 
  client.publish("myiot/pump_real_status", String(logicalStatus).c_str());
  lastSentPump = logicalStatus;
}

void callback(char* topic, byte* payload, unsigned int length) {
  char message = (char)payload[0];
  String currentTopic = String(topic);
  
  if (currentTopic == "myiot/control") {
    if (message == '1' && isGrease && currentDistance > SAFETY_LIMIT) { digitalWrite(PUMP_PIN, LOW); sendPumpStatus(); } 
    else if (message == '0') { digitalWrite(PUMP_PIN, HIGH); isTiming = false; sendPumpStatus(); }
  }
  else if (currentTopic == "myiot/control_motor") {
    if (message == '1') { digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW); client.publish("myiot/motor_status", "1"); } 
    else if (message == '0') { digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW); client.publish("myiot/motor_status", "0"); }
  }
  else if (currentTopic == "myiot/control_mix_pump") {
    if (message == '1' && !isMixPumpRunning) { digitalWrite(MIX_PUMP_PIN, LOW); isMixPumpRunning = true; mixPumpStartTime = millis(); client.publish("myiot/mix_pump_status", "1"); }
  }
  // หมายเหตุ: โค้ดรับคำสั่ง Manual Relay 1, 2 เดิมยังมีอยู่ แต่จะถูก PID 
  // ควบคุมทับในฟังก์ชัน Loop ทันที (เพราะระบบ PID กลายเป็นระบบอัตโนมัติแล้ว)
}

void setup() {
  Serial.begin(9600);
  pinMode(XKC_1_PIN, INPUT); pinMode(XKC_2_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
   
  pinMode(PUMP_PIN, OUTPUT); digitalWrite(PUMP_PIN, HIGH); 
  pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW);
  pinMode(MIX_PUMP_PIN, OUTPUT); digitalWrite(MIX_PUMP_PIN, HIGH); 
  
  pinMode(EXTRA_RELAY_1_PIN, OUTPUT); digitalWrite(EXTRA_RELAY_1_PIN, HIGH);
  pinMode(EXTRA_RELAY_2_PIN, OUTPUT); digitalWrite(EXTRA_RELAY_2_PIN, HIGH);
   
  sensors.begin();
  sensors2.begin(); 
  
  // ==========================================
  // [เพิ่มใหม่] เริ่มต้นระบบ PID
  // ==========================================
  windowStartTime = millis();
  
  // ตั้งขีดจำกัด Output ของ PID ให้ไม่เกิน 5000ms (เท่ากับ WindowSize)
  myPID1.SetOutputLimits(0, WindowSize);
  myPID2.SetOutputLimits(0, WindowSize);
  
  // สั่งให้ PID เริ่มทำงาน
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  // ==========================================

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) { reconnect(); }
  client.loop();

  // --- [เพิ่มใหม่] ส่วนทำงานของ Time-Proportioning PID (ทำงานทุกรอบ Loop) ---
  unsigned long nowMillis = millis();
  
  // ถ้านับเวลาครบ 1 หน้าต่าง (5 วินาที) ให้เลื่อนกรอบเวลาใหม่
  if (nowMillis - windowStartTime > WindowSize) { 
    windowStartTime += WindowSize;
  }
  
  // คุม Relay ตัวที่ 1 (Heater 1) แบบ Active LOW
  if (Output1 > (nowMillis - windowStartTime)) {
    digitalWrite(EXTRA_RELAY_1_PIN, LOW); // เปิด Heater
  } else {
    digitalWrite(EXTRA_RELAY_1_PIN, HIGH); // ปิด Heater
  }

  // คุม Relay ตัวที่ 2 (Heater 2) แบบ Active LOW
  if (Output2 > (nowMillis - windowStartTime)) {
    digitalWrite(EXTRA_RELAY_2_PIN, LOW); // เปิด Heater
  } else {
    digitalWrite(EXTRA_RELAY_2_PIN, HIGH); // ปิด Heater
  }
  // ----------------------------------------------------------------------

  if (isMixPumpRunning) {
    if (millis() - mixPumpStartTime >= MIX_PUMP_DURATION) {
      digitalWrite(MIX_PUMP_PIN, HIGH); isMixPumpRunning = false; client.publish("myiot/mix_pump_status", "0"); 
    }
  }

  int s1 = digitalRead(XKC_1_PIN), s2 = digitalRead(XKC_2_PIN), currentState = 0;
  if (s1 == LOW && s2 == LOW) { currentState = 0; isGrease = false; } 
  else if (s1 != s2) { currentState = 1; isGrease = true; } 
  else if (s1 == HIGH && s2 == HIGH) { currentState = 2; isGrease = false; }

  if (currentState != lastSentState) { client.publish("myiot/grease_status", String(currentState).c_str()); lastSentState = currentState; }

  static unsigned long lastDistRead = 0;
  if (millis() - lastDistRead > 500) { 
    lastDistRead = millis();
    currentDistance = readUltrasonic();
    client.publish("myiot/tank_distance", String(currentDistance, 1).c_str());
  }

  bool isPumpRunning = (digitalRead(PUMP_PIN) == LOW);
  if (isPumpRunning) { if (!isGrease || currentDistance < SAFETY_LIMIT) { digitalWrite(PUMP_PIN, HIGH); isTiming = false; sendPumpStatus(); } }

  if (isGrease && !isPumpRunning && currentDistance > SAFETY_LIMIT) {
      if (!isTiming) { greaseStartTime = millis(); isTiming = true; } 
      else { if (millis() - greaseStartTime >= AUTO_DELAY_MS) { digitalWrite(PUMP_PIN, LOW); sendPumpStatus(); isTiming = false; } }
  } else { if (isTiming) isTiming = false; }

  // --- อ่านอุณหภูมิ และคำนวณ PID ใหม่ ทุกๆ 5 วินาที ---
  unsigned long now = millis();
  if (now - lastTempMsg > 5000) {
    lastTempMsg = now;
    
    // อุณหภูมิตัวที่ 1
    sensors.requestTemperatures(); 
    float tempC = sensors.getTempCByIndex(0);
    if(tempC != DEVICE_DISCONNECTED_C) {
      client.publish("myiot/temp", String(tempC, 1).c_str());
      // [เพิ่มใหม่] ส่งค่าเข้า PID และสั่งให้คำนวณ
      Input1 = tempC;
      myPID1.Compute();
    }
    
    // อุณหภูมิตัวที่ 2
    sensors2.requestTemperatures(); 
    float temp2C = sensors2.getTempCByIndex(0);
    if(temp2C != DEVICE_DISCONNECTED_C) {
      client.publish("myiot/temp2", String(temp2C, 1).c_str());
      // [เพิ่มใหม่] ส่งค่าเข้า PID และสั่งให้คำนวณ
      Input2 = temp2C;
      myPID2.Compute();
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("GreaseR4_Unique_ID_123", NULL, NULL, "myiot/board_status", 1, true, "offline")) { 
      client.publish("myiot/board_status", "online", true);
      client.subscribe("myiot/control"); 
      client.subscribe("myiot/control_motor"); 
      client.subscribe("myiot/control_mix_pump"); 
      client.subscribe("myiot/control_ex_relay1"); 
      client.subscribe("myiot/control_ex_relay2"); 
    } 
    else { delay(5000); }
  }
}