#include <Arduino.h>

#include <Wire.h>

#include <BMI088.h>

#include <Adafruit_BMP3XX.h>

#include <lyralibr1.h>

#include <SPIFlash.h>
#include <SD.h>

#include <RF24.h>

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

Adafruit_BMP3XX bmp;

SPIFlash flash(FLASHCS,0xEF16);

RF24 radio(CSBRKOUT,PC14);

void configpins(){
  pinMode(BUZZERPIN,OUTPUT);
  pinMode(REDLED,OUTPUT);
  pinMode(GREENLED,OUTPUT);
  pinMode(BLUELED,OUTPUT);
  pinMode(FLASHCS,OUTPUT);
  pinMode(SDCS,OUTPUT);
  pinMode(PB2,OUTPUT);
  pinMode(P1EN,OUTPUT);
  digitalWrite(FLASHCS,HIGH);
  digitalWrite(SDCS,HIGH);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("\n\nrestart");

  configpins();
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
}

