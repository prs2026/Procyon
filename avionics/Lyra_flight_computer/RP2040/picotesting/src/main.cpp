#include <Arduino.h>
#include <quats.h>

#define LED_BUILTINn 25

Quaternion base(0,0,0,1);
Quaternion base2(0,0,1,0);

void printquat(Quaternion q1){
    char result[45];
    sprintf(result,"w = %d,x = %di, y = %dj, z = %dk");
    Serial.println(result);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTINn,OUTPUT);
  digitalWrite(LED_BUILTINn, HIGH);
  Serial.begin(115200);
  Serial.println("\n\nrestart");
  
}

void loop() {
  digitalWrite(LED_BUILTINn,LOW);
  delay(1000);
  digitalWrite(LED_BUILTINn,HIGH);
  delay(1000);
  Serial.println("loop");
  printquat(base);
  // put your main code here, to run repeatedly:
}