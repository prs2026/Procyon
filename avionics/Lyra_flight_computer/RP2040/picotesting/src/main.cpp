#include <Arduino.h>
#include <quats.h>
#include "generallib.h"





Quaternion base(1,0,0,0);
Quaternion base2(0,1,0,0);
Quaternion basetonorm(0,1,1,0);

void printquat(Quaternion q1){
    char result[65];
    sprintf(result,"w = %d.%d,x = %d.%di, y = %d.%dj, z = %d.%dk",int(q1.w*1000)/1000,int(q1.w*1000)%1000,int(q1.x*1000)/1000,int(q1.x*1000)%1000,int(q1.y*1000)/1000,int(q1.y*1000)%1000,int(q1.z*1000)/1000,int(q1.z*1000)%1000);
    Serial.println(result);
}

Quaternion rotate(Quaternion torotate, Quaternion axis,float theta){ // rotate torotate quaterion around quaterion axis by theta
    theta = radians(theta)/2;
    // construct the rotation quaternion from the input axis and theta values
    Quaternion result;
    Quaternion q(1,0,0,0);
    q.w = cos(theta);
    q.x = axis.x*sin(theta);
    q.y = axis.y*sin(theta);
    q.z = axis.z*sin(theta);

    Serial.println("quaternion to rotate by");
    printquat(q);
    // rotate the original quaternion by the rotatino quaternion
    Quaternion _q = q.conjugate();
    result = (q*torotate)*_q;
    return result;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LEDRED,OUTPUT);
  pinMode(LEDGREEN,OUTPUT);
  pinMode(LEDBLUE,OUTPUT);

  digitalWrite(LEDRED, LOW);
  digitalWrite(LEDGREEN, HIGH);
  digitalWrite(LEDBLUE, HIGH);
  Serial.begin(115200);
  Serial.println("\n\nrestart");
  /*
  basetonorm.normalize();
  
  Quaternion addedquat = base+base2;
  Quaternion multedquat = base*basetonorm;
  Quaternion scalarmult = base*4;
  Quaternion rotatedquat = rotate(base,base2,45);
  */
  //printquat(base2);
  //printquat(basetonorm);
  //printquat(addedquat);
  //printquat(multedquat);
  //printquat(scalarmult);
  //printquat(rotatedquat);
  
}

void loop() {
  
  digitalWrite(LEDRED,LOW);
  delay(1000);
  digitalWrite(LEDRED,HIGH);
  delay(1000);
  Serial.println("loop");
  
  
  // put your main code here, to run repeatedly:
}