#include <TinyGPS++.h>
#include<SoftwareSerial.h>
#include <JY901.h>
#include <JY901_dfs.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// plus tard : ajouter pitot

const byte TRIGGER_PIN = 5; 
const byte ECHO_PIN = 3;
static const int RXPin = 2, TXPin = 10;
static const uint32_t GPSBaud = 4800;
//const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
//const float SOUND_SPEED = 340.0 / 1000;
double ailerons = 0;
double profondeur = 0; 
double derive=0;
double moteur=0;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

File myFile;

void setup() {
  Serial.begin(115200);
  /* Initialise les broches */
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN, INPUT);
  JY901.attach(Serial);
  ss.begin(GPSBaud);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) { Serial.println("initialization failed!");
    return;}
  Serial.println("initialization done.");
  pinMode(7 , INPUT);
  pinMode(9,INPUT);
  pinMode(6,INPUT);
  pinMode(8,INPUT);
  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
  delay (100);
  myFile = SD.open("test.csv", FILE_WRITE);
  if (myFile) {
  myFile.println("commande ailerons,commande dérive,commande profondeur,commande moteur,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,AngleX,Angley,AngleZ,lattitude,longitude,heurs,minutes,secondes,centisecondes");  // Distance du sol (mm),
  delay(100);
  myFile.close();
  }
  else { Serial.println("pb");
  analogWrite(A0,255);
  delay (1000);}
  analogWrite(A0,0);
  analogWrite(A1,255);
}

void loop() {
  
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read())){
       myFile = SD.open("test.csv", FILE_WRITE);
  if (myFile) {
  ailerons = pulseIn(7 , HIGH);
  myFile.print(ailerons);
  myFile.print(",");
  profondeur = pulseIn(9 , HIGH);
  myFile.print(profondeur);
  myFile.print(",");
  derive = pulseIn(6 , HIGH);
  myFile.print(derive);
  myFile.print(",");
  moteur = pulseIn(2 , HIGH);
  myFile.print(moteur);
  myFile.print(",");
  
  /*  Ancien code lors de l'utilisation du capteur ultrason
  
  // 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  // 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe)
  long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);
   
  // 3. Calcul la distance à partir du temps mesuré
  float distance_mm = measure / 2.0 * SOUND_SPEED;
   
  // Affiche les résultats en mm, cm et m
  myFile.print(distance_mm);
  myFile.print(",");
  JY901.receiveSerialData(); 
  */
  
  myFile.print(JY901.getAccX());
  myFile.print(",");
  myFile.print(JY901.getAccY());
  myFile.print(",");
  myFile.print(JY901.getAccZ());
  myFile.print(",");
	
  myFile.print(JY901.getGyroX());
  myFile.print(",");
  myFile.print(JY901.getGyroY());
  myFile.print(",");
  myFile.print(JY901.getGyroZ());
  myFile.print(",");

  myFile.print(JY901.getRoll());
  myFile.print(",");
  myFile.print(JY901.getPitch());
  myFile.print(",");
  myFile.print(JY901.getYaw());
  myFile.println(",");

  
  if (gps.location.isValid())
  {
    myFile.print(gps.location.lat(), 6);
    myFile.print(F(","));
    myFile.print(gps.location.lng(), 6);
  }
  else
  {
    myFile.print(F("INVALID"));
  }
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) myFile.print(F("0"));
    myFile.print(gps.time.hour());
    myFile.print(F(","));
    if (gps.time.minute() < 10) myFile.print(F("0"));
    myFile.print(gps.time.minute());
    myFile.print(F(","));
    if (gps.time.second() < 10) myFile.print(F("0"));
    myFile.print(gps.time.second());
    myFile.print(F(","));
    if (gps.time.centisecond() < 10) myFile.print(F("0"));
    myFile.print(gps.time.centisecond());
  }
  else
  {
    myFile.print(F("INVALID"));
  }
  }
  else { myFile.println("pb");
  analogWrite(A0,255);
  }
  analogWrite(A0,0);
  analogWrite(A1,255);
      myFile.close();
}

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
    }
 
}


