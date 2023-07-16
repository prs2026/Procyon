#include <Arduino.h>
#include <SPI.h>

#define MOSI PB5
#define MISO PB4
#define SCK PB3

const int radiocs = PC14;
const int radioce = PC15;

const int pyro1 = PB10;

bool trylaunch = false;


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(pyro1, OUTPUT);
  digitalWrite(pyro1, LOW);
}

void broadcast(int data) {

}

void loop() {

  if (Serial.available() > 0 )
  {
  byte data = Serial.read();
  Serial.print("Echo: ");
  Serial.println(data);

  switch (data)
  {
  case 108:
    broadcast(108);
    trylaunch = true;
    Serial.println("launcharmed");
    break;
  
  case 109:
    broadcast(109);
    break;

  default:
    break;
  }
  }
  if (trylaunch == true)
  {
    digitalWrite(pyro1, HIGH);
    delay(100);
    digitalWrite(pyro1, LOW);
    Serial.println("pyro1 fire");
    trylaunch = false;
  }
  
  // put your main code here, to run repeatedly:
}

