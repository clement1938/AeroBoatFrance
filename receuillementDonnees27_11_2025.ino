/*
  Acquisition :
  - 6 ADC
  - VL53L0X
  - Pitot MS4525DO
  - GPS (SoftwareSerial)
  - IMU WT61C (UART Serial1, avec JY901)
  - Ultrason HC-SR04 (Trig 5, Echo 6)
  Sortie SD CSV
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <JY901.h>

/* =====================================================
   =================== GPS ==============================
   =====================================================*/
const uint8_t GPS_RX = 2;
const uint8_t GPS_TX = 3;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

TinyGPSPlus gps;
double lastLat = 0, lastLon = 0;
bool lastGpsValid = false;
uint32_t gpsTimeSec = 0;
bool gpsTimeValid = false;

/* =====================================================
   =================== IMU WT61C (avec JY901) ==========
   =====================================================*/
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float roll = 0, pitch = 0, yaw = 0;
bool imuValid = false;

/* =====================================================
   =================== VL53 =============================
   =====================================================*/
Adafruit_VL53L0X lox;
volatile int lastVL53 = -1;

/* =====================================================
   =================== PITOT ============================
   =====================================================*/
#define PITOT_ADDRESS 0x28

const float P_MIN = -1.0f;
const float P_MAX = 1.0f;
const float PSI_TO_PA = 6894.757f;

float pitotPressure = 0;
float pitotTemperature = 0;
float pitotOffsetPa = 0;

/* =====================================================
   =================== Ultrason =========================
   =====================================================*/
const byte TRIG_PIN = 5;
const byte ECHO_PIN = 6;
const unsigned long US_TIMEOUT = 25000UL;
float lastUltrason_mm = NAN;

/* =====================================================
   =================== ADC + SD + Time ==================
   =====================================================*/
const int pinIn[6] = {A1, A2, A3, A4, A5, A6};
const unsigned long period_us = 100000; // 10 Hz

File myFile;
char filename[20];
int file_id = 0;

unsigned long Time = 0;
unsigned long lastFileSwitch = 0;
const unsigned long file_period_ms = 5000;

unsigned long compteur = 0; // Compteur pour le timing précis

/* =====================================================
   =============== PITOT FUNCTIONS ======================
   =====================================================*/

// --- Lecture simple ---
void readPitot() {
  Wire.requestFrom(PITOT_ADDRESS, 4);
  if (Wire.available() == 4) {
    uint16_t p_raw = (Wire.read() & 0x3F) << 8;
    p_raw |= Wire.read();
    uint16_t t_raw = (Wire.read() << 3) | (Wire.read() >> 5);

    float p_psi = ((float)p_raw - 1638.0f) * (P_MAX-P_MIN) / (14745.0f-1638.0f) + P_MIN;
    pitotPressure = p_psi * PSI_TO_PA - pitotOffsetPa;

    pitotTemperature = ((float)t_raw * 0.0977f) - 50.0f;
  }
}

// --- Calibration pitot ---
float calibratePitotOffset(int samples) {
  float sum = 0, sumSq = 0;
  int n = 0;

  Serial.println("Calibration Pitot...");

  for (int i = 0; i < samples; i++) {
    Wire.requestFrom(PITOT_ADDRESS, 4);
    if (Wire.available() != 4) { delay(5); continue; }

    uint16_t p_raw = (Wire.read() & 0x3F) << 8;
    p_raw |= Wire.read();
    Wire.read(); Wire.read();

    float p_psi = ((float)p_raw - 1638.0f) * (P_MAX-P_MIN) / (14745.0f-1638.0f) + P_MIN;
    float p_pa = p_psi * PSI_TO_PA;

    sum += p_pa;
    sumSq += p_pa * p_pa;
    n++;

    delay(5);
  }

  if (n < 20) {
    Serial.println("Erreur calibration Pitot !");
    return 0;
  }

  float mean = sum / n;
  float var  = (sumSq / n) - mean*mean;

  Serial.print("Pitot offset = "); Serial.print(mean); Serial.print(" Pa, variance=");
  Serial.println(var);

  if (var > 5.0) Serial.println("Pitot instable pendant calibration !");

  return mean;
}

/* =====================================================
   =================== VL53 =============================
   =====================================================*/
void updateVL53() {
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  if (m.RangeStatus != 4) lastVL53 = m.RangeMilliMeter;
}

/* =====================================================
   =================== Ultrason =========================
   =====================================================*/
void updateUltrason() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long pulse = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT);

  lastUltrason_mm = (pulse == 0) ? NAN : (pulse * 0.340) / 2.0;
}

/* =====================================================
   =================== IMU WT61C (JY901) =================
   =====================================================*/
void updateIMU() {
  JY901.receiveSerialData();
  accX = JY901.getAccX();
  accY = JY901.getAccY();
  accZ = JY901.getAccZ();
  gyroX = JY901.getGyroX();
  gyroY = JY901.getGyroY();
  gyroZ = JY901.getGyroZ();
  roll  = JY901.getRoll();
  pitch = JY901.getPitch();
  yaw   = JY901.getYaw();
  imuValid = !(accX == 0 && accY == 0 && accZ == 0 && gyroX == 0 && gyroY == 0 && gyroZ == 0);
}

/* =====================================================
   =================== GPS ==============================
   =====================================================*/
void updateGPS() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  if (gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
    lastGpsValid = true;
  } else lastGpsValid = false;

  if (gps.time.isValid()) {
    gpsTimeSec = gps.time.hour()*3600 +
                 gps.time.minute()*60 +
                 gps.time.second();
    gpsTimeValid = true;
  }
}

/* =====================================================
   =================== FILE =============================
   =====================================================*/
void openNewFile() {
  if (myFile) { myFile.flush(); myFile.close(); }

  sprintf(filename, "rec_%03d.csv", file_id++);
  myFile = SD.open(filename, FILE_WRITE);

  myFile.println("time_us,gps_sec,raw1,raw2,raw3,raw4,raw5,raw6,"
                 "vl53_mm,pitot_pa,air_speed,ultrason_mm,"
                 "accX,accY,accZ,gyroX,gyroY,gyroZ,roll,pitch,yaw,"
                 "lat,lon,gps_valid");

  lastFileSwitch = millis();
  Time = micros();
  compteur = 0;
}

/* =====================================================
   =================== SETUP ============================
   =====================================================*/
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // IMU WT61C
  JY901.attach(Serial1); // Attache l'IMU à Serial1

  gpsSerial.begin(4800);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  analogReference(INTERNAL1V1);

  SD.begin(53);

  Wire.begin();
  Wire.setClock(400000);

  lox.begin();
  lox.startRangeContinuous(20);

  // --- Calibration Pitot ---
  delay(500);
  pitotOffsetPa = calibratePitotOffset(200);

  openNewFile();
}

/* =====================================================
   =================== LOOP =============================
   =====================================================*/
void loop() {
  unsigned long now = micros();

  if (millis() - lastFileSwitch >= file_period_ms)
    openNewFile();

  // Timing précis avec compteur
  if (now - Time >= compteur * period_us) {
    compteur++;

  updateGPS();
  updateIMU();
  updateVL53();
  readPitot();
  updateUltrason();

    // ADC
    uint16_t raw[6];
    for (int i=0; i<6; i++) {
      analogRead(pinIn[i]);
      raw[i] = analogRead(pinIn[i]);
    }

    // Airspeed
    float airDensity = 1.225;
    float airSpeed = (pitotPressure >= 0)
                    ? sqrt(2*pitotPressure/airDensity)
                    : -sqrt(2*(-pitotPressure)/airDensity);

    // Write CSV
    myFile.print(now); myFile.print(",");
    myFile.print(gpsTimeValid ? gpsTimeSec : 0); myFile.print(",");

    for(int i=0;i<6;i++){ myFile.print(raw[i]); myFile.print(","); }

    myFile.print(lastVL53); myFile.print(",");
    myFile.print(pitotPressure); myFile.print(",");
    myFile.print(airSpeed); myFile.print(",");
    myFile.print(lastUltrason_mm); myFile.print(",");

    myFile.print(accX); myFile.print(",");
    myFile.print(accY); myFile.print(",");
    myFile.print(accZ); myFile.print(",");
    myFile.print(gyroX); myFile.print(",");
    myFile.print(gyroY); myFile.print(",");
    myFile.print(gyroZ); myFile.print(",");
    myFile.print(roll); myFile.print(",");
    myFile.print(pitch); myFile.print(",");
    myFile.print(yaw); myFile.print(",");

    if (lastGpsValid) {
      myFile.print(lastLat,6); myFile.print(",");
      myFile.print(lastLon,6); myFile.print(",");
      myFile.print(1);
    } else {
      myFile.print(",,0");
    }

    myFile.println();
    myFile.flush();
  }
}
