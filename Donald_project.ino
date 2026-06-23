#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include "Adafruit_MPU6050_Modified.h" 

// -----------------------------------------
// NETWORK & MQTT CONFIGURATION
// -----------------------------------------
const char* ssid = "Jesus is Lord";
const char* password = "ROBOTICS";
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_telemetry_topic = "conrad/brace/telemetry";
const char* mqtt_command_topic = "conrad/brace/command";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer downloadServer(80);

// -----------------------------------------
// PIN DEFINITIONS & MUX
// -----------------------------------------
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define SPI_SCK_PIN  4
#define SPI_MISO_PIN 5
#define SPI_MOSI_PIN 6
#define SD_CS_PIN    7
#define FSR1_PIN 0
#define FSR2_PIN 1
#define VIB_MOTOR_PIN 2
#define LED_PIN       3
#define TCA9548A_ADDR 0x70

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RTC_DS3231 rtc;
Adafruit_MPU6050 mpu1, mpu2, mpu3;

// -----------------------------------------
// SYSTEM STATE, TIMING & ALGORITHMS
// -----------------------------------------
const int FSR_WORN_THRESHOLD = 300; 
int fsrDebounce = 0; // Prevents "flapping" between worn/removed

const float POSTURE_TOLERANCE_RAD = 0.35; // ~20 Degrees of bend allowed
const unsigned long POSTURE_DELAY_MS = 5000; // 5 seconds of sustained bad posture required
unsigned long postureTimer = 0;
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 1000; 

// Hardware Inversion Flags (Change to -1.0 if mounted upside down)
const float DIR_MPU1 = 1.0; 
const float DIR_MPU2 = 1.0; 
const float DIR_MPU3 = 1.0;

int lastFsr1 = 0, lastFsr2 = 0;
bool isWorn = false, badPosture = false, isCompliant = false;
String lastTime = "Waiting...";
String dateOnly = "2026-01-01";
int currentDay = -1;
unsigned long totalSecondsWorn = 0;

// MPU Tilt Variables & Persistent Offsets
float m1_p = 0, m1_r = 0, offset_m1_p = 0, offset_m1_r = 0; 
float m2_p = 0, m2_r = 0, offset_m2_p = 0, offset_m2_r = 0; 
float m3_p = 0, m3_r = 0, offset_m3_p = 0, offset_m3_r = 0;

bool oledOk = false, rtcOk = false, sdOk = false;
bool mpu1Ok = false, mpu2Ok = false, mpu3Ok = false;

// -----------------------------------------
// HELPER FUNCTIONS
// -----------------------------------------
void tcaSelect(uint8_t bus) {
  if (bus > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR); Wire.write(1 << bus); Wire.endTransmission(); delay(10); 
}

void calculateTilt(sensors_event_t& a, float& pitch, float& roll, float offset_p, float offset_r, float dir) {
  pitch = (atan2(a.acceleration.y, a.acceleration.z) * dir) - offset_p;
  roll = (atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * dir) - offset_r;
}

void loadCalibration() {
  if (!sdOk) return;
  File f = SD.open("/calib.txt", FILE_READ);
  if (f) {
    offset_m1_p = f.readStringUntil(',').toFloat(); offset_m1_r = f.readStringUntil('\n').toFloat();
    offset_m2_p = f.readStringUntil(',').toFloat(); offset_m2_r = f.readStringUntil('\n').toFloat();
    offset_m3_p = f.readStringUntil(',').toFloat(); offset_m3_r = f.readStringUntil('\n').toFloat();
    f.close();
  }
}

void saveCalibration() {
  if (!sdOk) return;
  SD.remove("/calib.txt");
  File f = SD.open("/calib.txt", FILE_WRITE);
  if (f) {
    f.print(offset_m1_p, 4); f.print(","); f.println(offset_m1_r, 4);
    f.print(offset_m2_p, 4); f.print(","); f.println(offset_m2_r, 4);
    f.print(offset_m3_p, 4); f.print(","); f.println(offset_m3_r, 4);
    f.close();
  }
}

void calibratePosture() {
  sensors_event_t a, g, temp;
  if(mpu1Ok) { tcaSelect(2); mpu1.getEvent(&a, &g, &temp); offset_m1_p = atan2(a.acceleration.y, a.acceleration.z) * DIR_MPU1; offset_m1_r = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * DIR_MPU1; }
  if(mpu2Ok) { tcaSelect(3); mpu2.getEvent(&a, &g, &temp); offset_m2_p = atan2(a.acceleration.y, a.acceleration.z) * DIR_MPU2; offset_m2_r = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * DIR_MPU2; }
  if(mpu3Ok) { tcaSelect(4); mpu3.getEvent(&a, &g, &temp); offset_m3_p = atan2(a.acceleration.y, a.acceleration.z) * DIR_MPU3; offset_m3_r = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * DIR_MPU3; }
  saveCalibration();
  digitalWrite(LED_PIN, HIGH); delay(500); digitalWrite(LED_PIN, LOW);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) message += (char)payload[i];
  if (String(topic) == mqtt_command_topic && message.indexOf("calibrate") >= 0) calibratePosture();
}

void updateDisplay(const char* timeStr, bool compliant, unsigned long totalSecs, bool isOnline) {
  tcaSelect(1); display.clearDisplay(); 
  display.fillRect(0, 0, SCREEN_WIDTH, 14, WHITE); display.setTextColor(BLACK, WHITE);
  if (isOnline) { display.fillRect(2, 8, 2, 2, BLACK); display.fillRect(5, 6, 2, 4, BLACK); display.fillRect(8, 4, 2, 6, BLACK); display.fillRect(11, 2, 2, 8, BLACK); } 
  else { display.setCursor(2, 3); display.print("OFF"); }
  
  int16_t x1, y1; uint16_t w, h; display.getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(SCREEN_WIDTH - w - 2, 3); display.print(timeStr);

  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 20); display.print("FSR1: "); display.print(lastFsr1);
  display.print(" | FSR2: "); display.println(lastFsr2);
  
  unsigned long hrs = totalSecs / 3600; unsigned long mins = (totalSecs % 3600) / 60; unsigned long secs = totalSecs % 60;
  char timeFmt[25]; snprintf(timeFmt, sizeof(timeFmt), "%02lu h %02lu m %02lu s", hrs, mins, secs); 
  display.setCursor(0, 32); display.print("Worn: "); display.println(timeFmt);

  if(!isWorn) { display.drawRect(0, 46, SCREEN_WIDTH, 18, WHITE); display.setTextColor(WHITE, BLACK); display.setTextSize(2); display.setCursor(24, 48); display.println("REMOVED"); }
  else if(compliant) { display.fillRect(0, 46, SCREEN_WIDTH, 18, WHITE); display.setTextColor(BLACK, WHITE); display.setTextSize(2); display.setCursor(12, 48); display.println("COMPLIANT"); } 
  else { display.drawRect(0, 46, SCREEN_WIDTH, 18, WHITE); display.setTextColor(WHITE, BLACK); display.setTextSize(2); display.setCursor(28, 48); display.println("ALERT"); }
  
  display.setTextSize(1); display.display();
}

// -----------------------------------------
// SETUP
// -----------------------------------------
void setup() {
  Serial.begin(115200); delay(1000); 

  pinMode(FSR1_PIN, INPUT); pinMode(FSR2_PIN, INPUT);
  pinMode(VIB_MOTOR_PIN, OUTPUT); pinMode(LED_PIN, OUTPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  tcaSelect(0); rtcOk = rtc.begin(); if (rtcOk && rtc.lostPower()) rtc.adjust(DateTime(__DATE__, __TIME__));
  tcaSelect(1); oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  
  if(oledOk) { display.clearDisplay(); display.setTextColor(WHITE); display.setCursor(0, 0); display.println("Booting..."); display.display(); }

  tcaSelect(2); mpu1Ok = mpu1.begin(0x68, &Wire, 0);
  tcaSelect(3); mpu2Ok = mpu2.begin(0x68, &Wire, 1);
  tcaSelect(4); mpu3Ok = mpu3.begin(0x68, &Wire, 2);
  
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
  sdOk = SD.begin(SD_CS_PIN);
  loadCalibration(); 

  WiFi.begin(ssid, password);
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttCallback);

  downloadServer.on("/ping", HTTP_GET, []() {
    downloadServer.sendHeader("Access-Control-Allow-Origin", "*");
    downloadServer.send(200, "text/plain", "ok");
  });

  downloadServer.on("/download", HTTP_GET, []() {
    downloadServer.sendHeader("Access-Control-Allow-Origin", "*");
    if (!downloadServer.hasArg("date")) { downloadServer.send(400, "text/plain", "Missing date"); return; }
    String filename = "/" + downloadServer.arg("date") + ".csv";
    File file = SD.open(filename, FILE_READ);
    if (!file) { downloadServer.send(404, "text/plain", "Log file not found."); return; }
    downloadServer.streamFile(file, "text/csv"); file.close();
  });
  downloadServer.begin();
}

// -----------------------------------------
// MAIN LOOP
// -----------------------------------------
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    downloadServer.handleClient();
    static bool mdnsStarted = false;
    if(!mdnsStarted) { MDNS.begin("conradbrace"); mdnsStarted = true; }

    if (!mqttClient.connected()) {
      static unsigned long lastMqttAttempt = 0;
      if (millis() - lastMqttAttempt > 5000) {
        lastMqttAttempt = millis();
        String clientId = "ESP32Brace-" + String(random(0xffff), HEX);
        if(mqttClient.connect(clientId.c_str())) mqttClient.subscribe(mqtt_command_topic); 
      }
    } else mqttClient.loop();
  }

  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = millis();

    // TIME & DAILY RESET LOGIC
    char timeDisplay[10] = "--:--";
    if (rtcOk) {
      tcaSelect(0); DateTime now = rtc.now();
      sprintf(timeDisplay, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
      char fullStr[25]; char dateStr[15];
      sprintf(fullStr, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
      sprintf(dateStr, "%04d-%02d-%02d", now.year(), now.month(), now.day());
      lastTime = String(fullStr); dateOnly = String(dateStr);
      
      // Reset wear time at midnight
      if (currentDay != -1 && now.day() != currentDay) totalSecondsWorn = 0; 
      currentDay = now.day();
    }

    // FSR DEBOUNCE LOGIC
    lastFsr1 = analogRead(FSR1_PIN);
    lastFsr2 = analogRead(FSR2_PIN);
    if (lastFsr1 > FSR_WORN_THRESHOLD || lastFsr2 > FSR_WORN_THRESHOLD) {
      fsrDebounce++;
      if (fsrDebounce > 2) { isWorn = true; fsrDebounce = 2; }
    } else {
      fsrDebounce--;
      if (fsrDebounce <= 0) { isWorn = false; fsrDebounce = 0; }
    }
    
    if(isWorn) totalSecondsWorn++; 

    sensors_event_t a, g, temp;
    if(mpu1Ok) { tcaSelect(2); mpu1.getEvent(&a, &g, &temp); calculateTilt(a, m1_p, m1_r, offset_m1_p, offset_m1_r, DIR_MPU1); }
    if(mpu2Ok) { tcaSelect(3); mpu2.getEvent(&a, &g, &temp); calculateTilt(a, m2_p, m2_r, offset_m2_p, offset_m2_r, DIR_MPU2); }
    if(mpu3Ok) { tcaSelect(4); mpu3.getEvent(&a, &g, &temp); calculateTilt(a, m3_p, m3_r, offset_m3_p, offset_m3_r, DIR_MPU3); }

    // DYNAMIC POSTURE ALGORITHM (Checks true angles, not raw gravity)
    bool currentBend = (abs(m1_p) > POSTURE_TOLERANCE_RAD || abs(m2_p) > POSTURE_TOLERANCE_RAD || abs(m3_p) > POSTURE_TOLERANCE_RAD ||
                        abs(m1_r) > POSTURE_TOLERANCE_RAD || abs(m2_r) > POSTURE_TOLERANCE_RAD || abs(m3_r) > POSTURE_TOLERANCE_RAD);
    
    if (currentBend) {
      if (postureTimer == 0) postureTimer = millis(); 
      if (millis() - postureTimer >= POSTURE_DELAY_MS) badPosture = true; 
    } else {
      postureTimer = 0; 
      badPosture = false;
    }

    isCompliant = (isWorn && !badPosture);

    if (!isCompliant && isWorn) { 
      digitalWrite(LED_PIN, HIGH); digitalWrite(VIB_MOTOR_PIN, HIGH); delay(150); digitalWrite(VIB_MOTOR_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, LOW); digitalWrite(VIB_MOTOR_PIN, LOW);
    }

    if (oledOk) updateDisplay(timeDisplay, isCompliant, totalSecondsWorn, mqttClient.connected());

    if (mqttClient.connected()) {
      String json = "{";
      json += "\"fsr1\":" + String(lastFsr1) + ",\"fsr2\":" + String(lastFsr2) + ",";
      json += "\"worn\":" + String(isWorn ? "true" : "false") + ",";
      json += "\"compliant\":" + String(isCompliant ? "true" : "false") + ",";
      json += "\"wear_time\":" + String(totalSecondsWorn) + ",";
      json += "\"mpu1_pitch\":" + String(m1_p, 4) + ", \"mpu1_roll\":" + String(m1_r, 4) + ",";
      json += "\"mpu2_pitch\":" + String(m2_p, 4) + ", \"mpu2_roll\":" + String(m2_r, 4) + ",";
      json += "\"mpu3_pitch\":" + String(m3_p, 4) + ", \"mpu3_roll\":" + String(m3_r, 4);
      json += "}";
      mqttClient.publish(mqtt_telemetry_topic, json.c_str());
    }

    if (sdOk) {
      String filename = "/" + dateOnly + ".csv";
      bool fileExists = SD.exists(filename);
      File dataFile = SD.open(filename, FILE_APPEND);
      if (dataFile) {
        if(!fileExists) dataFile.println("Timestamp,FSR1,FSR2,Worn,Compliant,M1_Pitch,M1_Roll,M2_Pitch,M2_Roll,M3_Pitch,M3_Roll");
        dataFile.print(lastTime); dataFile.print(","); dataFile.print(lastFsr1); dataFile.print(","); dataFile.print(lastFsr2); dataFile.print(",");
        dataFile.print(isWorn); dataFile.print(","); dataFile.print(isCompliant); dataFile.print(",");
        dataFile.print(m1_p,4); dataFile.print(","); dataFile.print(m1_r,4); dataFile.print(",");
        dataFile.print(m2_p,4); dataFile.print(","); dataFile.print(m2_r,4); dataFile.print(",");
        dataFile.print(m3_p,4); dataFile.print(","); dataFile.println(m3_r,4); 
        dataFile.close();
      }
    }
  }
}