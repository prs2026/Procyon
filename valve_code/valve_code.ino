#include "Servo.h"

const int oxvalvepin = 12;
const int fuelvalvepin = 10;
const int fillvalvepin = 9;

Servo fuelvalve;
Servo oxvalve;
Servo fillvalve;

const int fillclosed = 165;
const int oxclosed = 147;
const int fuelclosed = 155;

const int fillopen = 100;
const int oxopen = 90;
const int fuelopen = 85;

void setup() {
  // put your setup code here, to run once:
 fuelvalve.attach(fuelvalvepin);
 oxvalve.attach(oxvalvepin);
 fillvalve.attach(fillvalvepin);
 
 fuelvalve.write(fuelclosed);
 oxvalve.write(oxclosed);
 fillvalve.write(fillclosed);
 
 pinMode(A1,INPUT_PULLUP);
 pinMode(A3,INPUT_PULLUP);

 Serial.begin(9600);
}

void loop() {
  // fill valve code
  if(analogRead(A1) < 100){
    fillvalve.write(fillopen);
    Serial.println("fillopen");
  }
  else{
    fillvalve.write(fillclosed);
  }


  // main valve code
   if(analogRead(A3) < 100){
    fuelvalve.write(fuelopen);
    oxvalve.write(oxopen);
    Serial.println("mainopen");
  }
  else{
    fuelvalve.write(fuelclosed);
    oxvalve.write(oxclosed);
  }
  // put your main code here, to run repeatedly:
  
}
