#include <Arduino.h>
#define LEDPIN PC13

#define REDLED PA3
#define GREENLED PA2
#define BLUELED PA1

void setup() {
  pinMode(LEDPIN, OUTPUT);
  pinMode(REDLED,OUTPUT);
  pinMode(GREENLED,OUTPUT);
  pinMode(BLUELED,OUTPUT);

  digitalWrite(LEDPIN,LOW);
  digitalWrite(REDLED,LOW);
  digitalWrite(GREENLED,LOW);
  digitalWrite(BLUELED,LOW);
  // put your setup code here, to run once:
  
}

void loop() {
  digitalWrite(GREENLED,HIGH);
  delay(1000);
  digitalWrite(GREENLED,LOW);
  delay(1000);
  // put your main code here, to run repeatedly:
}
