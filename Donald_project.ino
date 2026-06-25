/*
 * Smart TLSO Compliance Monitor - v2.0
 * Conrad Robotics | Author: Awele Bonaventure Ugbah
 *
 * KEY UPGRADE: Gravity-Vector Calibration
 * =========================================
 * Previous approach: scalar atan2 offset subtraction per sensor
 *   → BROKEN when sensors have different physical orientations
 *   → Pitch/roll axes don't correspond between sensors
 *
 * New approach: Gravity Reference Vector Remapping
 *   → At calibration, store the full 3-axis gravity unit vector
 *     [Ax, Ay, Az] (normalised) for each sensor in "good posture"
 *   → At runtime, compute the delta rotation between the live
 *     gravity vector and the reference vector using cross-product/
 *     dot-product. This gives a TRUE tilt angle regardless of
 *     sensor mounting orientation.
 *   → A single "body-frame" convention is then defined so all three
 *     sensors report movement in the same direction:
 *       +pitch = patient bending FORWARD
 *       +roll  = patient leaning to the RIGHT
 *   → Each sensor stores a sign-pair {pitchSign, rollSign} that is
 *     determined automatically during calibration by applying a
 *     known small perturbation — OR set manually via constants.
 */

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
const char* ssid          = "Jesus is Lord";
const char* password      = "ROBOTICS";
const char* mqtt_server   = "broker.hivemq.com";
const char* mqtt_telemetry_topic = "conrad/brace/telemetry";
const char* mqtt_command_topic   = "conrad/brace/command";

WiFiClient    espClient;
PubSubClient  mqttClient(espClient);
WebServer     downloadServer(80);

// -----------------------------------------
// PIN DEFINITIONS & MUX
// -----------------------------------------
#define I2C_SDA_PIN   8
#define I2C_SCL_PIN   9
#define SPI_SCK_PIN   4
#define SPI_MISO_PIN  5
#define SPI_MOSI_PIN  6
#define SD_CS_PIN     7
#define FSR1_PIN      0
#define FSR2_PIN      1
#define VIB_MOTOR_PIN 2
#define LED_PIN       3
#define TCA9548A_ADDR 0x70

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RTC_DS3231       rtc;
Adafruit_MPU6050 mpu1, mpu2, mpu3;

// -----------------------------------------
// POSTURE & TIMING PARAMETERS
// -----------------------------------------
const int   FSR_WORN_THRESHOLD  = 300;
const float POSTURE_TOLERANCE_RAD = 0.26f; // ~15 degrees
const unsigned long POSTURE_DELAY_MS  = 5000;
const unsigned long UPDATE_INTERVAL   = 100; // 10Hz for smoother 3D model

// -----------------------------------------
// BODY-FRAME SIGN CONVENTION
// (set to +1 or -1 for each sensor so that
//  a physical forward-bend reads POSITIVE
//  pitch on every sensor)
//
// HOW TO DETERMINE THESE:
//  Flash firmware, open Serial monitor at 115200.
//  Bend the brace FORWARD while wearing it.
//  Check which sensor's delta_pitch goes negative.
//  Set that sensor's PITCH_SIGN to -1.
//  Do the same for roll (lean RIGHT = positive).
// -----------------------------------------
const float MPU1_PITCH_SIGN = 1.0f;
const float MPU1_ROLL_SIGN  = 1.0f;
const float MPU2_PITCH_SIGN = 1.0f;   // <-- adjust if MPU2 is inverted
const float MPU2_ROLL_SIGN  = 1.0f;
const float MPU3_PITCH_SIGN = 1.0f;
const float MPU3_ROLL_SIGN  = 1.0f;

// -----------------------------------------
// CALIBRATION DATA STRUCTURE
// Stores the reference gravity unit vector
// and the live-reading tilt deltas.
// -----------------------------------------
struct SensorCal {
  // Reference gravity unit vector captured at calibration
  float ref_ax, ref_ay, ref_az;  // normalised to magnitude 1.0
  bool  valid;                    // true once calibrated

  // Live computed tilt relative to calibration pose (radians)
  float delta_pitch;
  float delta_roll;
};

SensorCal cal1, cal2, cal3;

// -----------------------------------------
// SYSTEM STATE
// -----------------------------------------
int   fsrDebounce = 0;
int   lastFsr1 = 0, lastFsr2 = 0;
bool  isWorn = false, badPosture = false, isCompliant = false;
bool  oledOk = false, rtcOk = false, sdOk = false;
bool  mpu1Ok = false, mpu2Ok = false, mpu3Ok = false;

String lastTime  = "Waiting...";
String dateOnly  = "2026-01-01";
int    currentDay = -1;

unsigned long totalSecondsWorn = 0;
unsigned long postureTimer     = 0;
unsigned long lastUpdate       = 0;

// -----------------------------------------
// HELPER: I2C MUX SELECT
// -----------------------------------------
void tcaSelect(uint8_t bus) {
  if (bus > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << bus);
  Wire.endTransmission();
  delay(5);
}

// -----------------------------------------
// CORE MATH: Gravity-vector tilt extraction
//
// Given:
//   ref  = [rx, ry, rz] normalised reference gravity (good posture)
//   live = [ax, ay, az] normalised live gravity
//
// We want to find the rotation FROM ref TO live.
// The axis of rotation is the cross product (ref × live).
// The angle is acos(ref · live).
//
// We then project that rotation onto the body-frame
// pitch (forward-back) and roll (left-right) axes.
//
// Body frame (defined by calibration):
//   pitch axis = horizontal axis perpendicular to gravity in the
//                sagittal plane  ≈ world X
//   roll  axis = horizontal axis perpendicular to gravity in the
//                coronal plane   ≈ world Y
//
// This is valid for small-to-medium angles (<90°), which is
// all we need for a spinal brace.
// -----------------------------------------
void computeTilt(const SensorCal& cal,
                 float ax, float ay, float az,  // raw (will be normalised)
                 float pitchSign, float rollSign,
                 float& out_pitch, float& out_roll) {

  // Normalise live vector
  float mag = sqrt(ax*ax + ay*ay + az*az);
  if (mag < 0.01f) { out_pitch = 0; out_roll = 0; return; }
  float lx = ax/mag, ly = ay/mag, lz = az/mag;

  // If not calibrated, output zeros
  if (!cal.valid) { out_pitch = 0; out_roll = 0; return; }

  float rx = cal.ref_ax, ry = cal.ref_ay, rz = cal.ref_az;

  // --- Cross product: rotation axis = ref × live ---
  float cx = ry*lz - rz*ly;
  float cy = rz*lx - rx*lz;
  float cz = rx*ly - ry*lx;

  // --- Dot product: cos(angle) ---
  float dot = rx*lx + ry*ly + rz*lz;
  dot = constrain(dot, -1.0f, 1.0f);
  float angle = acos(dot);  // total tilt angle in radians

  // --- Decompose into pitch and roll ---
  // The cross product vector (cx, cy, cz) is the rotation axis scaled by sin(angle).
  // We project it onto our body pitch and roll axes.
  //
  // Body convention:
  //   The reference gravity vector defines "down".
  //   We build two horizontal body axes perpendicular to "down":
  //     pitch_axis = normalise(ref × [0,0,1]) if ref is not already [0,0,1]
  //                  else use [1,0,0]
  //     roll_axis  = normalise(ref × pitch_axis)
  //
  // Then:  sin(pitch) ≈ dot(cross_product_axis, pitch_axis)
  //        sin(roll)  ≈ dot(cross_product_axis, roll_axis)
  //
  // For small angles, sin(θ) ≈ θ, so these ARE the angles.

  // Build pitch axis: horizontal, forward direction
  // Use world-up [0,1,0] (Y-up convention) to define horizontal
  float wx = 0, wy = 1, wz = 0;  // world up guess
  // If ref is close to [0,1,0], use [1,0,0] instead
  if (fabs(ry) > 0.9f) { wx = 1; wy = 0; wz = 0; }

  // pitch_axis = normalise(world_up × ref)  -> points "forward"
  float pax = wy*rz - wz*ry;
  float pay = wz*rx - wx*rz;
  float paz = wx*ry - wy*rx;
  float pmag = sqrt(pax*pax + pay*pay + paz*paz);
  if (pmag < 0.01f) { out_pitch = 0; out_roll = 0; return; }
  pax /= pmag; pay /= pmag; paz /= pmag;

  // roll_axis  = normalise(ref × pitch_axis) -> points "right"
  float rax = ry*paz - rz*pay;
  float ray = rz*pax - rx*paz;
  float raz = rx*pay - ry*pax;
  float rmag = sqrt(rax*rax + ray*ray + raz*raz);
  if (rmag < 0.01f) { out_pitch = 0; out_roll = 0; return; }
  rax /= rmag; ray /= rmag; raz /= rmag;

  // Project the rotation axis onto pitch and roll axes
  // (cross product vector already encodes sin(angle))
  float raw_pitch = cx*pax + cy*pay + cz*paz;
  float raw_roll  = cx*rax + cy*ray + cz*raz;

  // Apply body-frame sign convention
  out_pitch = raw_pitch * pitchSign;
  out_roll  = raw_roll  * rollSign;

  // Debug output (comment out in production)
  Serial.printf("  tilt_angle=%.3f pitch=%.3f roll=%.3f\n", angle, out_pitch, out_roll);
}

// -----------------------------------------
// CALIBRATION: Capture reference vectors
// -----------------------------------------
void captureReference(SensorCal& cal, Adafruit_MPU6050& mpu, bool mpuOk) {
  if (!mpuOk) return;

  // Average 20 samples to get a stable reference
  float sumX = 0, sumY = 0, sumZ = 0;
  const int N = 20;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += a.acceleration.x;
    sumY += a.acceleration.y;
    sumZ += a.acceleration.z;
    delay(10);
  }
  float ax = sumX/N, ay = sumY/N, az = sumZ/N;
  float mag = sqrt(ax*ax + ay*ay + az*az);
  if (mag < 0.01f) return;

  cal.ref_ax = ax / mag;
  cal.ref_ay = ay / mag;
  cal.ref_az = az / mag;
  cal.valid  = true;

  Serial.printf("  CAL ref=[%.3f, %.3f, %.3f]\n", cal.ref_ax, cal.ref_ay, cal.ref_az);
}

void calibratePosture() {
  Serial.println("Calibrating posture - patient must stand still...");

  if (mpu1Ok) { tcaSelect(2); captureReference(cal1, mpu1, mpu1Ok); }
  if (mpu2Ok) { tcaSelect(3); captureReference(cal2, mpu2, mpu2Ok); }
  if (mpu3Ok) { tcaSelect(4); captureReference(cal3, mpu3, mpu3Ok); }

  saveCalibration();

  // Visual confirmation
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(200);
    digitalWrite(LED_PIN, LOW);  delay(200);
  }
  Serial.println("Calibration complete.");
}

// -----------------------------------------
// SD: Save / Load Calibration
// -----------------------------------------
void saveCalibration() {
  if (!sdOk) return;
  SD.remove("/calib.txt");
  File f = SD.open("/calib.txt", FILE_WRITE);
  if (!f) return;
  // Format: rx,ry,rz per line for each sensor
  f.printf("%.6f,%.6f,%.6f\n", cal1.ref_ax, cal1.ref_ay, cal1.ref_az);
  f.printf("%.6f,%.6f,%.6f\n", cal2.ref_ax, cal2.ref_ay, cal2.ref_az);
  f.printf("%.6f,%.6f,%.6f\n", cal3.ref_ax, cal3.ref_ay, cal3.ref_az);
  f.printf("%d,%d,%d\n", cal1.valid?1:0, cal2.valid?1:0, cal3.valid?1:0);
  f.close();
  Serial.println("Calibration saved to SD.");
}

void loadCalibration() {
  if (!sdOk) return;
  File f = SD.open("/calib.txt", FILE_READ);
  if (!f) { Serial.println("No calibration file found."); return; }

  cal1.ref_ax = f.readStringUntil(',').toFloat();
  cal1.ref_ay = f.readStringUntil(',').toFloat();
  cal1.ref_az = f.readStringUntil('\n').toFloat();

  cal2.ref_ax = f.readStringUntil(',').toFloat();
  cal2.ref_ay = f.readStringUntil(',').toFloat();
  cal2.ref_az = f.readStringUntil('\n').toFloat();

  cal3.ref_ax = f.readStringUntil(',').toFloat();
  cal3.ref_ay = f.readStringUntil(',').toFloat();
  cal3.ref_az = f.readStringUntil('\n').toFloat();

  String validLine = f.readStringUntil('\n');
  cal1.valid = (validLine.charAt(0) == '1');
  cal2.valid = (validLine.charAt(2) == '1');
  cal3.valid = (validLine.charAt(4) == '1');

  f.close();
  Serial.printf("Calibration loaded. Valid: %d %d %d\n", cal1.valid, cal2.valid, cal3.valid);
}

// -----------------------------------------
// MQTT CALLBACK
// -----------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  if (String(topic) == mqtt_command_topic && message.indexOf("calibrate") >= 0) {
    calibratePosture();
  }
}

// -----------------------------------------
// OLED DISPLAY
// -----------------------------------------
void updateDisplay(const char* timeStr, bool compliant, unsigned long totalSecs, bool isOnline) {
  tcaSelect(1);
  display.clearDisplay();
  display.fillRect(0, 0, SCREEN_WIDTH, 14, WHITE);
  display.setTextColor(BLACK, WHITE);

  if (isOnline) {
    display.fillRect(2, 8, 2, 2, BLACK); display.fillRect(5, 6, 2, 4, BLACK);
    display.fillRect(8, 4, 2, 6, BLACK); display.fillRect(11, 2, 2, 8, BLACK);
  } else {
    display.setCursor(2, 3); display.print("OFF");
  }

  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(SCREEN_WIDTH - w - 2, 3); display.print(timeStr);

  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 20);
  display.print("FSR1: "); display.print(lastFsr1);
  display.print(" | FSR2: "); display.println(lastFsr2);

  unsigned long hrs  = totalSecs / 3600;
  unsigned long mins = (totalSecs % 3600) / 60;
  unsigned long secs = totalSecs % 60;
  char timeFmt[25];
  snprintf(timeFmt, sizeof(timeFmt), "%02luh %02lum %02lus", hrs, mins, secs);
  display.setCursor(0, 32); display.print("Worn: "); display.println(timeFmt);

  if (!isWorn) {
    display.drawRect(0, 46, SCREEN_WIDTH, 18, WHITE);
    display.setTextColor(WHITE, BLACK); display.setTextSize(2);
    display.setCursor(24, 48); display.println("REMOVED");
  } else if (compliant) {
    display.fillRect(0, 46, SCREEN_WIDTH, 18, WHITE);
    display.setTextColor(BLACK, WHITE); display.setTextSize(2);
    display.setCursor(12, 48); display.println("COMPLIANT");
  } else {
    display.drawRect(0, 46, SCREEN_WIDTH, 18, WHITE);
    display.setTextColor(WHITE, BLACK); display.setTextSize(2);
    display.setCursor(28, 48); display.println("ALERT");
  }
  display.setTextSize(1);
  display.display();
}

// -----------------------------------------
// SETUP
// -----------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(FSR1_PIN, INPUT);
  pinMode(FSR2_PIN, INPUT);
  pinMode(VIB_MOTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Channel 0: RTC
  tcaSelect(0); rtcOk = rtc.begin();
  if (rtcOk && rtc.lostPower()) rtc.adjust(DateTime(__DATE__, __TIME__));

  // Channel 1: OLED
  tcaSelect(1); oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk) {
    display.clearDisplay(); display.setTextColor(WHITE);
    display.setCursor(0, 0); display.println("Booting v2.0...");
    display.println("Gravity-Cal System");
    display.display();
  }

  // Channels 2-4: MPUs
  tcaSelect(2); mpu1Ok = mpu1.begin(0x68, &Wire, 0);
  if (mpu1Ok) { mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ); }

  tcaSelect(3); mpu2Ok = mpu2.begin(0x68, &Wire, 1);
  if (mpu2Ok) { mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ); }

  tcaSelect(4); mpu3Ok = mpu3.begin(0x68, &Wire, 2);
  if (mpu3Ok) { mpu3.setFilterBandwidth(MPU6050_BAND_21_HZ); }

  Serial.printf("MPUs: %d %d %d\n", mpu1Ok, mpu2Ok, mpu3Ok);

  // SD & load saved calibration
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
  sdOk = SD.begin(SD_CS_PIN);
  loadCalibration();

  // WiFi & MQTT
  WiFi.begin(ssid, password);
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttCallback);

  // Local web server for SD download
  downloadServer.on("/ping", HTTP_GET, []() {
    downloadServer.sendHeader("Access-Control-Allow-Origin", "*");
    downloadServer.send(200, "text/plain", "ok");
  });
  downloadServer.on("/download", HTTP_GET, []() {
    downloadServer.sendHeader("Access-Control-Allow-Origin", "*");
    if (!downloadServer.hasArg("date")) { downloadServer.send(400, "text/plain", "Missing date"); return; }
    String filename = "/" + downloadServer.arg("date") + ".csv";
    File file = SD.open(filename, FILE_READ);
    if (!file) { downloadServer.send(404, "text/plain", "Log not found."); return; }
    downloadServer.streamFile(file, "text/csv"); file.close();
  });
  downloadServer.begin();

  if (oledOk) {
    display.clearDisplay(); display.setCursor(0, 0);
    display.println(cal1.valid ? "Cal: LOADED OK" : "Cal: NOT FOUND");
    display.println("Wear brace & press");
    display.println("'Zero Baseline'");
    display.println("to calibrate.");
    display.display();
  }
}

// -----------------------------------------
// MAIN LOOP
// -----------------------------------------
void loop() {
  // --- WiFi / MQTT housekeeping ---
  if (WiFi.status() == WL_CONNECTED) {
    downloadServer.handleClient();
    static bool mdnsStarted = false;
    if (!mdnsStarted) { MDNS.begin("conradbrace"); mdnsStarted = true; }

    if (!mqttClient.connected()) {
      static unsigned long lastMqttAttempt = 0;
      if (millis() - lastMqttAttempt > 5000) {
        lastMqttAttempt = millis();
        String clientId = "ESP32Brace-" + String(random(0xffff), HEX);
        if (mqttClient.connect(clientId.c_str()))
          mqttClient.subscribe(mqtt_command_topic);
      }
    } else {
      mqttClient.loop();
    }
  }

  if (millis() - lastUpdate < UPDATE_INTERVAL) return;
  lastUpdate = millis();

  // --- RTC: Time & Daily Reset ---
  char timeDisplay[10] = "--:--";
  if (rtcOk) {
    tcaSelect(0);
    DateTime now = rtc.now();
    sprintf(timeDisplay, "%02d:%02d", now.hour(), now.minute());
    char fullStr[25], dateStr[15];
    sprintf(fullStr, "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    sprintf(dateStr, "%04d-%02d-%02d", now.year(), now.month(), now.day());
    lastTime = String(fullStr);
    dateOnly = String(dateStr);
    if (currentDay != -1 && now.day() != currentDay) totalSecondsWorn = 0;
    currentDay = now.day();
  }

  // --- FSR Debounce: Is brace worn? ---
  lastFsr1 = analogRead(FSR1_PIN);
  lastFsr2 = analogRead(FSR2_PIN);
  bool fsrActive = (lastFsr1 > FSR_WORN_THRESHOLD || lastFsr2 > FSR_WORN_THRESHOLD);
  if (fsrActive) {
    fsrDebounce = min(fsrDebounce + 1, 3);
    if (fsrDebounce >= 3) isWorn = true;
  } else {
    fsrDebounce = max(fsrDebounce - 1, 0);
    if (fsrDebounce == 0) isWorn = false;
  }
  if (isWorn) totalSecondsWorn++;

  // --- Read MPUs and compute tilt via gravity-vector remapping ---
  sensors_event_t a, g, temp;

  if (mpu1Ok) {
    tcaSelect(2);
    mpu1.getEvent(&a, &g, &temp);
    computeTilt(cal1,
                a.acceleration.x, a.acceleration.y, a.acceleration.z,
                MPU1_PITCH_SIGN, MPU1_ROLL_SIGN,
                cal1.delta_pitch, cal1.delta_roll);
  }
  if (mpu2Ok) {
    tcaSelect(3);
    mpu2.getEvent(&a, &g, &temp);
    computeTilt(cal2,
                a.acceleration.x, a.acceleration.y, a.acceleration.z,
                MPU2_PITCH_SIGN, MPU2_ROLL_SIGN,
                cal2.delta_pitch, cal2.delta_roll);
  }
  if (mpu3Ok) {
    tcaSelect(4);
    mpu3.getEvent(&a, &g, &temp);
    computeTilt(cal3,
                a.acceleration.x, a.acceleration.y, a.acceleration.z,
                MPU3_PITCH_SIGN, MPU3_ROLL_SIGN,
                cal3.delta_pitch, cal3.delta_roll);
  }

  // --- Posture Algorithm ---
  // Check if any sensor deviates beyond tolerance from calibrated pose
  bool currentBend = (
    fabs(cal1.delta_pitch) > POSTURE_TOLERANCE_RAD ||
    fabs(cal1.delta_roll)  > POSTURE_TOLERANCE_RAD ||
    fabs(cal2.delta_pitch) > POSTURE_TOLERANCE_RAD ||
    fabs(cal2.delta_roll)  > POSTURE_TOLERANCE_RAD ||
    fabs(cal3.delta_pitch) > POSTURE_TOLERANCE_RAD ||
    fabs(cal3.delta_roll)  > POSTURE_TOLERANCE_RAD
  );

  if (currentBend) {
    if (postureTimer == 0) postureTimer = millis();
    if (millis() - postureTimer >= POSTURE_DELAY_MS) badPosture = true;
  } else {
    postureTimer = 0;
    badPosture   = false;
  }

  isCompliant = (isWorn && !badPosture);

  // --- Physical Feedback ---
  if (!isCompliant && isWorn) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(VIB_MOTOR_PIN, HIGH);
    delay(100);
    digitalWrite(VIB_MOTOR_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(VIB_MOTOR_PIN, LOW);
  }

  // --- OLED ---
  if (oledOk) updateDisplay(timeDisplay, isCompliant, totalSecondsWorn, mqttClient.connected());

  // --- MQTT Telemetry ---
  // Sends delta_pitch and delta_roll in unified body-frame radians.
  // The dashboard can now use these values DIRECTLY to drive the 3D model.
  if (mqttClient.connected()) {
    char json[512];
    snprintf(json, sizeof(json),
      "{"
      "\"fsr1\":%d,\"fsr2\":%d,"
      "\"worn\":%s,"
      "\"compliant\":%s,"
      "\"wear_time\":%lu,"
      "\"mpu1_pitch\":%.4f,\"mpu1_roll\":%.4f,"
      "\"mpu2_pitch\":%.4f,\"mpu2_roll\":%.4f,"
      "\"mpu3_pitch\":%.4f,\"mpu3_roll\":%.4f,"
      "\"calibrated\":%s"
      "}",
      lastFsr1, lastFsr2,
      isWorn ? "true" : "false",
      isCompliant ? "true" : "false",
      totalSecondsWorn,
      cal1.delta_pitch, cal1.delta_roll,
      cal2.delta_pitch, cal2.delta_roll,
      cal3.delta_pitch, cal3.delta_roll,
      (cal1.valid && cal2.valid && cal3.valid) ? "true" : "false"
    );
    mqttClient.publish(mqtt_telemetry_topic, json);
  }

  // --- SD Logging ---
  if (sdOk) {
    String filename = "/" + dateOnly + ".csv";
    bool fileExists = SD.exists(filename);
    File dataFile = SD.open(filename, FILE_APPEND);
    if (dataFile) {
      if (!fileExists) {
        dataFile.println("Timestamp,FSR1,FSR2,Worn,Compliant,"
                         "M1_Pitch,M1_Roll,M2_Pitch,M2_Roll,M3_Pitch,M3_Roll");
      }
      dataFile.print(lastTime);       dataFile.print(",");
      dataFile.print(lastFsr1);       dataFile.print(",");
      dataFile.print(lastFsr2);       dataFile.print(",");
      dataFile.print(isWorn);         dataFile.print(",");
      dataFile.print(isCompliant);    dataFile.print(",");
      dataFile.print(cal1.delta_pitch, 4); dataFile.print(",");
      dataFile.print(cal1.delta_roll,  4); dataFile.print(",");
      dataFile.print(cal2.delta_pitch, 4); dataFile.print(",");
      dataFile.print(cal2.delta_roll,  4); dataFile.print(",");
      dataFile.print(cal3.delta_pitch, 4); dataFile.print(",");
      dataFile.println(cal3.delta_roll, 4);
      dataFile.close();
    }
  }
}