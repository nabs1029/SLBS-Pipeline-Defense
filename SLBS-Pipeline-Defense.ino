#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// -------- PIN DEFINITIONS --------
#define OKM_SWITCH 12  // 1km switch (Normally Closed)
#define TKM_SWITCH 13  // 2km switch (Normally Open)
#define RELAY_PIN 11  // Relay control for bypass flow

// -------- LCD SETTINGS --------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// -------- SENSOR SETTINGS --------
const int SENSOR_PIN = A0;
const float VREF = 5.0;
const int ADC_MAX = 1023;
const float dividerScale = 1.45;

float zeroOffset = 0.0;
float pressure_kPa = 0.0;

// -------- GSM SETTINGS --------
SoftwareSerial sim800(7, 8); // RX, TX
String phone_number = "+916238109059"; // Replace with your number

// -------- PRESSURE RANGES --------
const float NORMAL_LOW = 107.5;
const float NORMAL_HIGH = 111;
const float LEAK_LOW = 102;
const float LEAK_HIGH = 106;
const float BLOCK_LOW = 113;
const float BLOCK_HIGH = 127;
const float DANGER_LIMIT = 128;

// -------- STATE TRACKING --------
String lastStatus = "NORMAL";
unsigned long statusStart = 0;
bool msgSent = false;
unsigned long blockStartTime = 0;
bool relayActive = false;

// -------- SMOOTHING FILTER --------
const int SMOOTH_SAMPLES = 10;
float pressureBuffer[SMOOTH_SAMPLES];
int bufferIndex = 0;
bool bufferFilled = false;

// ------------------------------------------------
void setup() {
  Serial.begin(9600);
  sim800.begin(9600);
  Wire.begin();
  lcd.init();
  lcd.backlight();

  pinMode(OKM_SWITCH, INPUT_PULLUP); // Normally Closed
  pinMode(TKM_SWITCH, INPUT_PULLUP); // Normally Open
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  lcd.setCursor(0, 0);
  lcd.print("Pipe Pressure:");
  delay(500);

  // ---- GSM INIT ----
  sendCommand("AT");
  sendCommand("AT+CMGF=1");  // SMS text mode

  // ---- Calibration ----
  Serial.println("Calibrating sensor...");
  delay(1000);
  zeroOffset = getAverageVoltage();
  Serial.print("Zero offset: ");
  Serial.println(zeroOffset, 3);
}

// ------------------------------------------------
void loop() {
  float Vout = getAverageVoltage();
  float Vsensor = Vout * dividerScale;
  float Veffective = Vsensor - zeroOffset;
  if (Veffective < 0) Veffective = 0;
  float rawPressure = Veffective * 500.0;

  // ---- Apply Moving Average Filter ----
  addToBuffer(rawPressure);
  pressure_kPa = getSmoothedPressure();

  String status = getPipeStatus(pressure_kPa);

  // ---- LCD Display ----
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(pressure_kPa, 1);
  lcd.print("kPa ");
  lcd.print(status);

  // ---- Serial Output ----
  Serial.print("Pressure: ");
  Serial.print(pressure_kPa, 2);
  Serial.print(" kPa | Status: ");
  Serial.println(status);

  unsigned long now = millis();

  // ---- Status Change Handling ----
  if (status != lastStatus) {
    lastStatus = status;
    statusStart = now;
    msgSent = false;
  }

  // ---- Switch Readings ----
  bool oneKmTriggered = (digitalRead(OKM_SWITCH) == HIGH); // NC: HIGH means open/triggered
  bool twoKmTriggered = (digitalRead(TKM_SWITCH) == HIGH); // NO: HIGH means triggered

  // ---- Detect Block and Handle ----
  if (status == "BLOCK") {
    if (blockStartTime == 0) blockStartTime = now;

    // If block persists more than 5 seconds -> Activate bypass relay
    if (!relayActive && (now - blockStartTime >= 5000)) {
      digitalWrite(RELAY_PIN, HIGH);
      relayActive = true;
      Serial.println("Bypass Relay Activated due to persistent BLOCK condition");
    }

    // Send message once after 3 seconds stable
    if (!msgSent && (now - statusStart >= 3000)) {
      if (oneKmTriggered) {
        sendSMS("Possible block at 1km distance", pressure_kPa);
      } else if (twoKmTriggered) {
        sendSMS("Possible block at 2km distance", pressure_kPa);
      } else {
        sendSMS("Block detected in pipe", pressure_kPa);
      }
      msgSent = true;
    }
  } 
  else {
    blockStartTime = 0; // reset
    if (relayActive && (status == "NORMAL" || status == "LEAK" || status == "FAIL")) {
      digitalWrite(RELAY_PIN, LOW);
      relayActive = false;
      Serial.println("Bypass Relay Deactivated (Pressure Normal)");
    }

    // Send messages for LEAK or EXPLODE
    if (!msgSent && (now - statusStart >= 3000)) {
      if (status == "LEAK" || status == "EXPLODE") {
        sendSMS(status, pressure_kPa);
        msgSent = true;
      }
    }
  }

  delay(200);
}

// ------------------------------------------------
// Stable status logic (no UNKNOWN)
String getPipeStatus(float p) {
  static String stableStatus = "NORMAL";

  if (p >= DANGER_LIMIT)
    stableStatus = "EXPLODE";
  else if (p >= BLOCK_LOW && p <= BLOCK_HIGH)
    stableStatus = "BLOCK";
  else if (p >= LEAK_LOW && p <= LEAK_HIGH)
    stableStatus = "LEAK";
  else if (p >= NORMAL_LOW && p <= NORMAL_HIGH)
    stableStatus = "NORMAL";
  else if (p < 64)
    stableStatus = "FAIL";
  // else → retain last stable status

  return stableStatus;
}

// ------------------------------------------------
float getAverageVoltage() {
  long total = 0;
  const int samples = 10;
  for (int i = 0; i < samples; i++) {
    total += analogRead(SENSOR_PIN);
    delay(10);
  }
  float avg = total / (float)samples;
  return (avg * VREF) / ADC_MAX;
}

// ------------------------------------------------
// Add pressure sample to smoothing buffer
void addToBuffer(float pressure) {
  pressureBuffer[bufferIndex] = pressure;
  bufferIndex = (bufferIndex + 1) % SMOOTH_SAMPLES;
  if (bufferIndex == 0) bufferFilled = true;
}

// ------------------------------------------------
// Compute smoothed pressure value
float getSmoothedPressure() {
  int count = bufferFilled ? SMOOTH_SAMPLES : bufferIndex;
  if (count == 0) return 0;
  float total = 0;
  for (int i = 0; i < count; i++) total += pressureBuffer[i];
  return total / count;
}

// ------------------------------------------------
void sendSMS(String status, float pressure) {
  String message = "Pipe Status: " + status +
                   " | Pressure: " + String(pressure, 1) + " kPa";
  Serial.println("Sending SMS: " + message);

  sim800.print("AT+CMGS=\"");
  sim800.print(phone_number);
  sim800.println("\"");
  delay(300);
  sim800.print(message);
  delay(200);
  sim800.write(26); // CTRL+Z
  delay(3000);

  Serial.println("Message sent!");
}

// ------------------------------------------------
void sendCommand(String cmd) {
  sim800.println(cmd);
  unsigned long start = millis();
  while (millis() - start < 1000) {
    if (sim800.available()) {
      Serial.write(sim800.read());
    }
  }
}
