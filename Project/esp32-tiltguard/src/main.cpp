#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Kalman.h>

// UART pins from ESP32-S2 to FRDM
#define UART_TX_PIN 18  // ESP32-S2 TX -> FRDM RX
#define UART_RX_PIN 17  // ESP32-S2 RX -> FRDM TX

// I2C pins to MPU-6050 (GY-521)
#define I2C_SDA_PIN 33
#define I2C_SCL_PIN 35

// IMU Raw -> Roll/Pitch
#define sqr(x) x * x
#define hypotenuse(x, y) sqrt(sqr(x) + sqr(y))

// ================== MPU-6050 ==================
static const uint8_t MPU_ADDR = 0x68; // AD0 low
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_WHO_AM_I    = 0x75;
// WHO_AM_I expected values:
// 0x68: Standard MPU-6050 (AD0 low)
// 0x69: Standard MPU-6050 (AD0 high)
// 0x70: Seen on some clones / MPU-6000 style variants or mislabeled chips; treat as compatible if basic registers respond

Adafruit_MPU6050 mpu; // Using default Wire now

double ax, ay, az;
double gx, gy, gz;
double roll, pitch;
Kalman kalmanX, kalmanY;
uint32_t timer = 0;
float kalPitch = 0, kalRoll = 0;
// ================== ESP32-side IMU tilt flags (spec: ESP sends only boolean tilt status) ==================
volatile bool imuTilt = false;        // true if |kalPitch| or |kalRoll| exceeds tilt threshold
volatile bool imuTiltPitch = false;   // true if pitch exceeded threshold
volatile bool imuTiltRoll = false;    // true if roll exceeded threshold

// Forward-declare FRDM status struct so updateImuTiltFlags (and others) can use it
struct FrdmStatus {
  bool armed;
  bool alarm;
  int thrTilt;   // degrees
  int thrLight;
  int light;
  String lastAlert;
};
// Global instance (will be defined/initialized later) - define default values here
FrdmStatus frdm;

// Forward prototype for sendLineToFrdm (implemented below) so tasks can call it
static void sendLineToFrdm(const String& line);

// Forward declarations for low-level I2C helpers used early in imuSetup
static bool i2cWriteByte(uint8_t reg, uint8_t val); // defined later
static bool i2cReadBytes(uint8_t reg, uint8_t* buf, size_t n); // defined later

void kalmanSetup()
{
	kalmanX.setAngle(roll);
	kalmanY.setAngle(pitch);
	timer = micros();
	kalmanX.setQbias(0.002f);
	kalmanY.setQbias(0.002f);
}

void imuSetup() {
  // Initialize default Wire bus on chosen pins
  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin();
  // Some MPU6050 boards are sensitive to clock speed and timing; slow down and settle
  Wire.setClock(100000); // 100 kHz
  delay(150);

  // Quick WHO_AM_I diagnostic before using the library
  uint8_t who = 0xFF; bool whoOk = i2cReadBytes(REG_WHO_AM_I, &who, 1);
  if (whoOk) {
    Serial.printf("WHO_AM_I=0x%02X (expect 0x68)\n", who);
  } else {
    Serial.println("WHO_AM_I read failed (I2C error)");
  }

  // Try waking the device (clear sleep)
  if (!i2cWriteByte(REG_PWR_MGMT_1, 0x00)) {
    Serial.println("PWR_MGMT_1 write failed (wake)");
  }
  delay(100);

  // Retry WHO_AM_I after wake
  whoOk = i2cReadBytes(REG_WHO_AM_I, &who, 1);
  if (whoOk) Serial.printf("WHO_AM_I(after wake)=0x%02X\n", who);

  // First attempt to init via Adafruit driver
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("Failed to init via Adafruit driver (attempt 1); retrying with soft-reset...");
    // Soft reset, then wake again
    if (!i2cWriteByte(REG_PWR_MGMT_1, 0x80)) {
      Serial.println("Device reset write failed");
    }
    delay(150);
    i2cWriteByte(REG_PWR_MGMT_1, 0x00);
    delay(150);
    // Final retry
    if (!mpu.begin(MPU_ADDR, &Wire)) {
      Serial.printf("Second init failed (WHO_AM_I=0x%02X). Evaluating fallback...\n", who);
      bool supported = (who == 0x68 || who == 0x69 || who == 0x70);
      if (!supported) {
        Serial.println("WHO_AM_I not recognized as compatible -> IMU disabled.");
        return;
      }
      // For 0x70 (or 0x69 if AD0 high but address set to 0x68), continue with raw register fallback
      Serial.println("Proceeding with raw register fallback (manual reads). Kalman & driver features limited.");
      // Minimal wake already done; no library config. We'll rely on imuReadOnce failing gracefully if sensor not true-compatible.
      return; // Early return: indicates library not initialized; tasks should check an init flag if needed.
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void imuReadOnce() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  // Compute roll and pitch (in degrees)
  roll = atan(ay/hypotenuse(ax, az)) * 180.0 / PI;
  pitch = atan2(-ax, az) * 180.0 / PI;
}

void getRollPitchK() {
  imuReadOnce();
  double dt = (double)(micros() - timer) / 1000000;
	timer = micros();
	kalRoll = kalmanX.getAngle(roll, gx, dt);
	kalPitch = kalmanY.getAngle(pitch, gy, dt);
}

// Update IMU tilt flags based on current Kalman-filtered angles and user tilt threshold (degrees)
static void updateImuTiltFlags() {
  float thr = (float)frdm.thrTilt;
  bool overPitch = fabsf(kalPitch) > thr;
  bool overRoll  = fabsf(kalRoll)  > thr;
  imuTiltPitch = overPitch;
  imuTiltRoll  = overRoll;
  imuTilt = (overPitch || overRoll);
}


void scanI2C() {
    byte address;
    int nDevices = 0;
    Serial.println("Scanning...");

    for (address = 1; address < 127; address++) {
  Wire.beginTransmission(address);
  if (Wire.endTransmission() == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
  } else if (Wire.endTransmission() == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    } else {
        Serial.println("Scan complete.\n");
    }
}

static bool i2cWriteByte(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cReadBytes(uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)MPU_ADDR, (int)n);
  if (got != n) return false;
  for (size_t i=0;i<n;i++) buf[i] = Wire.read();
  return true;
}

void imuTask(void* pv) {
  for (;;) {
    getRollPitchK();
    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}

void imuReportTask(void* pv) {
  // Periodically send IMU state to FRDM
  for (;;) {
    // SPEC: ESP32 sends ONLY boolean IMU tilt status to FRDM (no raw values)
    // true if |kalPitch| or |kalRoll| exceeds the user-set tilt threshold
    updateImuTiltFlags();
    sendLineToFrdm(String("IMU_TILT=") + (imuTilt ? "1" : "0"));
    vTaskDelay(pdMS_TO_TICKS(250)); // ~4 Hz reporting
  }
}

// ================== UART to FRDM ==================
HardwareSerial& FRDM = Serial1;  // use Serial1 with custom pins

// Send a line to FRDM (adds newline) - sends commands to FRDM when user interacts with web UI
// ARM, DISARM, TH_TILT=xxx, TH_LIGHT=xxx
static void sendLineToFrdm(const String& line) {
  FRDM.println(line);
}

// ================== UART from FRDM ==================
// (FrdmStatus declared earlier to allow IMU code to reference thresholds)
String lastStatusLine;

// Parse a line received from FRDM
static void parseFrdmLine(const String& line) {
  lastStatusLine = line;
  if (line.startsWith("STATUS ")) {
    // SPEC: FRDM -> ESP32: report armed, alarm, light (human-readable line)
    // Example: STATUS armed=1 alarm=0 light=380
    // Tolerant parser: will ignore extra fields if present (e.g., thr_*), but only stores required ones
    frdm.armed = line.indexOf("armed=1") >= 0;
    frdm.alarm = line.indexOf("alarm=1") >= 0;
    int p;
    if ((p = line.indexOf("light=")) >= 0) frdm.light = line.substring(p+6).toInt();
    // Optional: still accept thr_light if FRDM echoes it; user tilt threshold is owned by ESP32 side
    if ((p = line.indexOf("thr_light=")) >= 0) frdm.thrLight = line.substring(p+10).toInt();
  } else if (line.startsWith("ALERT ")) {
    frdm.lastAlert = line;
  }
}

// UART receive task
void uartRxTask(void* pv) {
  String buf;
  for (;;) {
    while (FRDM.available()) {
      char c = (char)FRDM.read();
      if (c == '\n') {
        buf.trim();
        if (buf.length()) parseFrdmLine(buf);
        buf = "";
      } else if (c != '\r') {
        buf += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


// ================== WiFi AP config ==================
static const char* WIFI_SSID = "TiltGuard";
static const char* WIFI_PASS = "tiltguard123";  // min 8 chars for WPA2

// ================== Web server ==================
WebServer server(80);
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Tilt & Light Guard</title>
  <style>
    body{font-family:sans-serif;margin:20px}
    button{margin:6px;padding:10px 16px}
    .row{margin:10px 0}
    #status{white-space:pre;font-family:monospace;background:#f6f6f6;padding:8px}
  </style>
</head>
<body>
  <h2>Tilt & Light Guard</h2>
  <div class="row">
    <button onclick="cmd('arm')">ARM</button>
    <button onclick="cmd('disarm')">DISARM</button>
  </div>
  <div class="row">
    Tilt threshold (deg): <input id="tht" type="range" min="0" max="90" value="30" oninput="st(this)"> <span id="thtVal">30</span>
  </div>
  <div class="row">
    Light threshold: <input id="thl" type="range" min="0" max="1023" value="500" oninput="sl(this)"> <span id="thlVal">500</span>
  </div>
  <pre id="status">loading...</pre>
<script>
function cmd(name){
  fetch('/'+name,{method:'POST'}).then(r=>r.text()).then(t=>console.log(t));
}
function st(el){document.getElementById('thtVal').innerText=el.value;fetch('/set?th_tilt='+el.value,{method:'POST'})}
function sl(el){document.getElementById('thlVal').innerText=el.value;fetch('/set?th_light='+el.value,{method:'POST'})}
function poll(){
  const url = '/status?t=' + Date.now(); // cache-bust per request
  fetch(url,{cache:'no-store'})
    .then(r=>r.json())
    .then(j=>{
  document.getElementById('status').textContent=JSON.stringify(j,null,2);
    })
    .catch(e=>{ console.log('poll error', e); })
    .finally(()=>setTimeout(poll,500)); // ~2 Hz
}
setTimeout(poll,300);
</script>
</body>
</html>
)HTML";

void handleIndex(){ server.send_P(200, "text/html", INDEX_HTML); }
void handleNotFound(){ server.send(404, "text/plain", "Not Found"); }

// From client (web) to server (esp32) which then sends to FRDM
void handleArm(){ sendLineToFrdm("ARM"); server.send(200, "text/plain", "OK"); }
void handleDisarm(){ sendLineToFrdm("DISARM"); server.send(200, "text/plain", "OK"); }
void handleSet(){
  bool ok=false;
  if (server.hasArg("th_tilt")) {
    int v = server.arg("th_tilt").toInt();
    // SPEC: Tilt threshold is owned by ESP32 side only; do not send to FRDM
    frdm.thrTilt=v; ok=true; // store user tilt threshold locally
  }
  if (server.hasArg("th_light")) {
    int v = server.arg("th_light").toInt();
    // SPEC: Light threshold is sent to FRDM over UART
    sendLineToFrdm(String("TH_LIGHT=")+v);
    frdm.thrLight=v; ok=true;
  }
  server.send(ok?200:400, "text/plain", ok?"OK":"NOOP");
}

// From FDRM to server (esp32) to client (web)
void handleStatus(){
  String json = "{";
  json += "\"armed\":" + String(frdm.armed?1:0) + ",";
  json += "\"alarm\":" + String(frdm.alarm?1:0) + ",";
  json += "\"thr_tilt\":" + String(frdm.thrTilt) + ",";
  json += "\"thr_light\":" + String(frdm.thrLight) + ",";
  json += "\"light\":" + String(frdm.light) + ",";
  // // ESP32-side telemetry required by spec
  // json += "\"ax\":" + String(ax,3) + ",";
  // json += "\"ay\":" + String(ay,3) + ",";
  // json += "\"az\":" + String(az,3) + ",";
  // json += "\"gx\":" + String(gx,3) + ",";
  // json += "\"gy\":" + String(gy,3) + ",";
  // json += "\"gz\":" + String(gz,3) + ",";
  // json += "\"roll\":" + String(roll,3) + ",";
  // json += "\"pitch\":" + String(pitch,3) + ",";
  json += "\"kalRoll\":" + String(kalRoll,3) + ",";
  json += "\"kalPitch\":" + String(kalPitch,3) + ",";
  // IMU tilt boolean and cause (pitch/roll/both/none)
  updateImuTiltFlags();
  json += "\"imuTilt\":" + String(imuTilt ? 1:0) + ",";
  String cause = (!imuTilt ? "none" : (imuTiltPitch && imuTiltRoll ? "both" : (imuTiltPitch ? "pitch" : "roll")));
  json += "\"imuTiltCause\":\"" + cause + "\",";
  json += "\"lastStatus\":\"" + lastStatusLine + "\"";
  json += "}";
  // Debug: log each status request and provide no-cache headers so clients don't freeze on cached responses
  Serial.println("/status requested -> replying JSON");
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.send(200, "application/json", json);
}


// ================== setup/loop ==================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32-S2 TiltGuard starting...");
  Serial.println("BOOT OK");

  // Initialize default FRDM status values (used by IMU tilt threshold and UI)
  frdm.armed = false;
  frdm.alarm = false;
  frdm.thrTilt = 30;   // default tilt threshold (degrees)
  frdm.thrLight = 500;
  frdm.light = 0;
  frdm.lastAlert = String("");

  // WiFi AP for simplest demo
  WiFi.mode(WIFI_AP);
  bool ap = WiFi.softAP(WIFI_SSID, WIFI_PASS);
  if (ap) Serial.printf("AP up: %s  IP: %s\n", WIFI_SSID, WiFi.softAPIP().toString().c_str());

  // UART to FRDM
  FRDM.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // I2C to MPU-6050 using default Wire
  imuSetup();          // calls Wire.begin(...)
  kalmanSetup();       // initialize Kalman filters
  getRollPitchK();     // take first reading
  // scanI2C();

  // Web server routes (ensure server actually runs; these were commented out)
  server.on("/", HTTP_GET, handleIndex);
  server.on("/arm", HTTP_POST, handleArm);
  server.on("/disarm", HTTP_POST, handleDisarm);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/status", HTTP_GET, handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();

  // Tasks (re-enable for periodic IMU + UART handling)
  xTaskCreatePinnedToCore(uartRxTask, "uartRx", 4096, nullptr, 1, nullptr, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 1, nullptr, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(imuReportTask, "imuRpt", 3072, nullptr, 1, nullptr, ARDUINO_RUNNING_CORE);
}

void loop() {
  server.handleClient();
  delay(5);
  // getRollPitchK();  
  static uint32_t lastBeat=0; if(millis()-lastBeat>1000){ lastBeat=millis(); Serial.println("heartbeat"); }
}
