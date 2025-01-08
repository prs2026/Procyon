#include <Arduino.h>
#include <macros.h>
#include <radio.h>
#include <lcd.h>


RADIO radio;
LCDDISPLAY lcddisplay;

telepacket currentstate;

uint32_t serialtime;
bool ready = false;

uint32_t dataage = 0;
uint32_t packettime;

void setup() {
  delay(3000);
  Serial.begin();
  Serial.print("init");
  pinMode(LEDINDICATION,OUTPUT);
  digitalWrite(LEDINDICATION,HIGH);
  delay(500);
  digitalWrite(LEDINDICATION,LOW);
  
  radio.init();
  Serial.println("radio good");
  lcddisplay.init();
  ready = true;
}

void setup1(){
  while (ready == false)
  {
    delay(100);
  }
}

void loop() {

  if (millis() - serialtime > 100)
  {
    serialtime = millis();
    Serial.printf(">Data age: %f\n",float((millis()-packettime))/1000);
  }
  
  delay(5000);
  // Lora.beginPacket();
  // int j = 0;

  //   Lora.write('f');
    
    
  //   if (!Lora.endPacket())
  //   {
  //       Serial.println("packet send fail");
  //   }
  
  //Serial.println("loop");
  
}

void loop1(){
  Lora.request();
  Lora.wait();
  if (Lora.available())
  {
    currentstate = radio.recivepacket();
    Serial.println("newpacket");
    Serial.printf(">State: %d\n",currentstate.r.state);
    Serial.printf(">Altitude: %f\n",float(currentstate.r.altitude)/10);
    Serial.printf(">Verticalvel: %f\n",float(currentstate.r.verticalvel)/100);
    packettime = millis();
  }
}