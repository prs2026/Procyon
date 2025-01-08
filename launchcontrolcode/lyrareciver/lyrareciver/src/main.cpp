#include <Arduino.h>
#include <macros.h>
#include <radio.h>

RADIO radio;
telepacket currentstate;

uint32_t serialtime;
bool ready = false;

uint32_t dataage = 0;
uint32_t packettime;

#define BUZZERPIN 16

void setup() {
  delay(3000);
  Serial.begin();
  Serial.print("init");
  pinMode(LEDINDICATION,OUTPUT);
  pinMode(BUZZERPIN,OUTPUT);
  pinMode(P2_EN,OUTPUT);
  pinMode(P2_CONT,INPUT_PULLUP);

  digitalWrite(LEDINDICATION,HIGH);
  delay(500);
  digitalWrite(LEDINDICATION,LOW);
  
  radio.init();
  Serial.println("radio good");
  ready = true;
  Serial.println("/----------------PADSIDE------------/");
}

void loop() {

  Lora.request(200);
  Lora.wait();
  int fire = 0;
  int firecode[] = {'f','i','r','e'};
  while (Lora.available())
  {
    int buf = Lora.read();
    Serial.printf("recieved %c\n",buf);
    if (buf == firecode[fire])
    {
      fire++;
    }
    Serial.printf("firenum = %d\n",fire);
    if (fire == 4)
    {
      Serial.printf("FIRING FIRING FIRING\n");
      digitalWrite(P2_EN,HIGH);
      delay(500);
      digitalWrite(P2_EN,LOW);
      break;
    }
    
    packettime = millis();
  }
  fire = 0;

  int contuinty = digitalRead(P2_CONT);

  if (!contuinty == 1)
  {
    Serial.print("continuity  ");
    tone(BUZZERPIN,4000,300);
  }
  else
  {
    noTone(BUZZERPIN);
  }
  Serial.printf("cont: %d\n",contuinty);
  
}
