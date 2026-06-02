#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>

// Use your custom local library!
#include "Adafruit_MPU6050_Modified.h" 

// -----------------------------------------
// WIFI CONFIGURATION
// -----------------------------------------
const char* ssid = "CONRAD10";
const char* password = "ROBOTICS";
WebServer server(80);
bool serverStarted = false; // Tracks if the background WiFi has connected

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

// -----------------------------------------
// COMPONENT CONFIGURATION
// -----------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RTC_DS3231 rtc;
Adafruit_MPU6050 mpu1, mpu2, mpu3;

// -----------------------------------------
// SYSTEM STATE & TIMING
// -----------------------------------------
const int FSR_WORN_THRESHOLD = 500; 
const float POSTURE_TOLERANCE_G = 0.5; 
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 1000; 

// Live Data Variables
int lastFsr1 = 0, lastFsr2 = 0;
float lastMpu1X = 0, lastMpu2X = 0, lastMpu3X = 0;
bool isWorn = false, badPosture = false, isCompliant = false;
String lastTime = "Waiting...";
bool oledOk = false, rtcOk = false, sdOk = false;
bool mpu1Ok = false, mpu2Ok = false, mpu3Ok = false;

// -----------------------------------------
// HELPER FUNCTIONS
// -----------------------------------------
void tcaSelect(uint8_t bus) {
  if (bus > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << bus);
  Wire.endTransmission();
  delay(10); 
}

// -----------------------------------------
// WEB SERVER HTML & AJAX
// -----------------------------------------
void handleRoot() {
  String html = R"=====(
<!DOCTYPE html><html><head><title>Live Brace Monitor</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
  body{font-family:Arial; padding:20px; background:#f4f4f4;}
  table{width:100%; max-width:600px; background:#fff; border-collapse:collapse; margin-top:20px;}
  th, td{padding:12px; border:1px solid #ddd; text-align:left;}
  th{background:#007BFF; color:white;}
  .ok{color:green; font-weight:bold;} .fail{color:red; font-weight:bold;}
</style>
<script>
  setInterval(function() {
    fetch('/data').then(response => response.json()).then(data => {
      document.getElementById('time').innerText = data.time;
      document.getElementById('fsr1').innerText = "ADC: " + data.fsr1;
      document.getElementById('fsr2').innerText = "ADC: " + data.fsr2;
      document.getElementById('mpu1').innerText = "X: " + data.mpu1 + " g";
      document.getElementById('mpu2').innerText = "X: " + data.mpu2 + " g";
      document.getElementById('mpu3').innerText = "X: " + data.mpu3 + " g";
      document.getElementById('worn').innerText = data.worn;
      document.getElementById('posture').innerText = data.posture;
      document.getElementById('compliant').innerText = data.compliant;
      document.getElementById('compliant').className = data.compliant == 'COMPLIANT' ? 'ok' : 'fail';
    });
  }, 1000); 
</script>
</head><body>
  <h2>Live Spinal Brace Dashboard</h2>
  <p><strong>Last Updated:</strong> <span id='time'>Loading...</span></p>
  <table>
    <tr><th>Component</th><th>Live Data</th></tr>
    <tr><td>MPU6050 #1 (Upper)</td><td id='mpu1'>Loading...</td></tr>
    <tr><td>MPU6050 #2 (Mid)</td><td id='mpu2'>Loading...</td></tr>
    <tr><td>MPU6050 #3 (Lower)</td><td id='mpu3'>Loading...</td></tr>
    <tr><td>FSR #1</td><td id='fsr1'>Loading...</td></tr>
    <tr><td>FSR #2</td><td id='fsr2'>Loading...</td></tr>
    <tr><th>System State</th><th>Result</th></tr>
    <tr><td>Brace Worn</td><td id='worn'>Loading...</td></tr>
    <tr><td>Posture</td><td id='posture'>Loading...</td></tr>
    <tr><td>Overall Compliance</td><td id='compliant'>Loading...</td></tr>
  </table>
</body></html>
  )=====";
  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{";
  json += "\"time\":\"" + lastTime + "\",";
  json += "\"fsr1\":" + String(lastFsr1) + ",";
  json += "\"fsr2\":" + String(lastFsr2) + ",";
  json += "\"mpu1\":\"" + String(lastMpu1X, 2) + "\",";
  json += "\"mpu2\":\"" + String(lastMpu2X, 2) + "\",";
  json += "\"mpu3\":\"" + String(lastMpu3X, 2) + "\",";
  json += "\"worn\":\"" + String(isWorn ? "YES" : "NO") + "\",";
  json += "\"posture\":\"" + String(badPosture ? "INCORRECT" : "GOOD") + "\",";
  json += "\"compliant\":\"" + String(isCompliant ? "COMPLIANT" : "ALERT") + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// -----------------------------------------
// SETUP
// -----------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000); // Give power time to stabilize, but NO while(!Serial)

  pinMode(FSR1_PIN, INPUT);
  pinMode(FSR2_PIN, INPUT);
  pinMode(VIB_MOTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(VIB_MOTOR_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // 1. Initialize RTC on Channel 0
  tcaSelect(0);
  rtcOk = rtc.begin();
  if (rtcOk && rtc.lostPower()) rtc.adjust(DateTime(__DATE__, __TIME__));

  // 2. Initialize OLED on Channel 1
  tcaSelect(1);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if(oledOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Brace Booting...");
    display.display();
  }

  // 3. Initialize MPUs on Channels 2, 3, 4
  tcaSelect(2); mpu1Ok = mpu1.begin(0x68, &Wire, 0);
  tcaSelect(3); mpu2Ok = mpu2.begin(0x68, &Wire, 1);
  tcaSelect(4); mpu3Ok = mpu3.begin(0x68, &Wire, 2);

  // 4. Init SD
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
  sdOk = SD.begin(SD_CS_PIN);
  if (sdOk) {
    File dataFile = SD.open("/datalog.csv", FILE_APPEND);
    if (dataFile) {
      if(dataFile.size() == 0) dataFile.println("Timestamp,FSR1,FSR2,MPU1,MPU2,MPU3,Compliant");
      dataFile.close();
    }
  }

  // 5. Start WiFi Connection in the Background (No While Loop!)
  WiFi.begin(ssid, password);
}

// -----------------------------------------
// MAIN LOOP
// -----------------------------------------
void loop() {
  
  // -- ASYNCHRONOUS WIFI CHECK --
  if (WiFi.status() == WL_CONNECTED) {
    if (!serverStarted) {
      // WiFi just connected! Start the Web Server
      server.on("/", handleRoot);
      server.on("/data", handleData);
      server.begin();
      serverStarted = true;
    }
    // Handle incoming web traffic
    server.handleClient();
  } else {
    // If WiFi drops, reset the server flag so it restarts when WiFi returns
    serverStarted = false; 
  }

  // -- SENSOR & LOGIC UPDATE (Every 1 Second) --
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = millis();

    // Read RTC
    if (rtcOk) {
      tcaSelect(0); 
      DateTime now = rtc.now();
      char timeStr[20];
      sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
              now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
      lastTime = String(timeStr);
    }

    // Read FSRs
    lastFsr1 = analogRead(FSR1_PIN);
    lastFsr2 = analogRead(FSR2_PIN);
    isWorn = (lastFsr1 > FSR_WORN_THRESHOLD && lastFsr2 > FSR_WORN_THRESHOLD);

    // Read MPUs
    sensors_event_t a, g, temp;
    if(mpu1Ok) { tcaSelect(2); mpu1.getEvent(&a, &g, &temp); lastMpu1X = a.acceleration.x; }
    if(mpu2Ok) { tcaSelect(3); mpu2.getEvent(&a, &g, &temp); lastMpu2X = a.acceleration.x; }
    if(mpu3Ok) { tcaSelect(4); mpu3.getEvent(&a, &g, &temp); lastMpu3X = a.acceleration.x; }

    badPosture = false;
    if (abs(lastMpu1X) > POSTURE_TOLERANCE_G || abs(lastMpu2X) > POSTURE_TOLERANCE_G || abs(lastMpu3X) > POSTURE_TOLERANCE_G) {
      badPosture = true;
    }

    isCompliant = (isWorn && !badPosture);

    // Provide physical feedback
    if (!isCompliant) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(VIB_MOTOR_PIN, HIGH);
      delay(100); 
      digitalWrite(VIB_MOTOR_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, LOW);
      digitalWrite(VIB_MOTOR_PIN, LOW);
    }

    // Update OLED Screen directly
    if (oledOk) {
      tcaSelect(1); 
      display.clearDisplay();
      display.setCursor(0, 0);
      
      // Top row dynamically shows WiFi Status
      if (serverStarted) {
        display.print("IP:"); display.println(WiFi.localIP());
      } else {
        display.println("WiFi: Connecting...");
      }
      
      display.setCursor(0, 12);
      display.print("FSR1:"); display.print(lastFsr1);
      display.print(" FSR2:"); display.println(lastFsr2);
      display.setCursor(0, 24);
      display.print("Brace: "); display.println(isWorn ? "WORN" : "REMOVED");
      display.setCursor(0, 36);
      display.print("Posture: "); display.println(badPosture ? "BAD" : "GOOD");
      display.setCursor(0, 48);
      display.print("Status: "); display.println(isCompliant ? "OK" : "ALERT");
      display.display();
    }

    // Log to SD Card
    if (sdOk) {
      File dataFile = SD.open("/datalog.csv", FILE_APPEND);
      if (dataFile) {
        dataFile.print(lastTime); dataFile.print(",");
        dataFile.print(lastFsr1); dataFile.print(",");
        dataFile.print(lastFsr2); dataFile.print(",");
        dataFile.print(lastMpu1X); dataFile.print(",");
        dataFile.print(lastMpu2X); dataFile.print(",");
        dataFile.print(lastMpu3X); dataFile.print(",");
        dataFile.println(isCompliant);
        dataFile.close();
      }
    }
  }
}