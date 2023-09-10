#include <Arduino.h>
#include <quats.h>
#include "generallib.h"
#include <Wire.h>
#include <Lyrav2sensors.h>

TwoWire i2c(I2C1_SDA,I2C1_SCL);

MPCORE MP;

NAVCORE NAV;


Quaternion base(1,0,0,0);
Quaternion base2(0,1,0,0);
Quaternion basetonorm(0,1,1,0);

void printquat(Quaternion q1){
    char result[65];
    sprintf(result,"w = %d.%d,x = %d.%di, y = %d.%dj, z = %d.%dk",int(q1.w*1000)/1000,int(q1.w*1000)%1000,int(q1.x*1000)/1000,int(q1.x*1000)%1000,int(q1.y*1000)/1000,int(q1.y*1000)%1000,int(q1.z*1000)/1000,int(q1.z*1000)%1000);
    Serial.println(result);
}



void setup() {
  MP.setuppins();
  MP.beep();
  // put your setup code here, to run once:
  

  i2c.begin();
  

  scani2c(i2c);

}

void setup1(){

}

void loop() {
  
  digitalWrite(LEDRED,LOW);
  delay(1000);
  digitalWrite(LEDRED,HIGH);
  delay(1000);
  Serial.println("loop");
  
  
  // put your main code here, to run repeatedly:
}



void loop1(){

}