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
#define BRKOUT1 10

int previousnum = 0;

void setup() {
  delay(3000);
  Serial.begin();
  Serial.print("init");
  pinMode(LEDINDICATION,OUTPUT);
  pinMode(BUZZERPIN,OUTPUT);
  pinMode(P2_EN,OUTPUT);
  digitalWrite(P2_EN,LOW);
  pinMode(P2_CONT,INPUT_PULLUP);

  digitalWrite(LEDINDICATION,HIGH);
  delay(500);
  digitalWrite(LEDINDICATION,LOW);
  
  radio.init();
  Serial.println("radio good");
  ready = true;
  Serial.println("/----------------PADSIDE------------/");
}

void setup1(){
  while (!ready)
  {
    delay(100);
  }
}

void loop() {
  Serial.print(Lora.status());
  //request data from radio
  Lora.request();
  Serial.print(Lora.status());
  //wait for message
  Lora.wait();
  Serial.print(Lora.status());

  //packet verification check variables
  int fire = 0;
  int firecode[] = {'l','a','u','n','c','h','t','h','a','t','t','h','a','n','g',0};
  //recived a packet? parse it
  while (Lora.available())
  {
    //print how much is left in the buffer (sanity check)
    Serial.printf("avalible left: %d \n",Lora.available());

    // read byte
    int buf = Lora.read();
    // print byte
    Serial.printf("recieved %c\n",buf);

    //if byte matches the expected packet, increment the counter
    if (buf == firecode[fire])
    {
      fire++;
    }

    if (fire == 15 && buf != previousnum)
    {
        fire++;
    }
    

    //print how many correct bytes in a row
    Serial.printf("firenum = %d\n",fire);



    //if the full correct packet has been recived, fire the pyro
    if (fire == 16)
    {
      Serial.printf("FIRING FIRING FIRING\n");
      tone(BUZZERPIN,8000,500);
      digitalWrite(P2_EN,HIGH);
      delay(500);
      digitalWrite(P2_EN,LOW);
      //reset the packet counter
      fire = 0;
      //clear rest of buffer
      while (Lora.available())
      {
        Lora.read();
      }
    }

  }


  
}

void loop1(){
  //   int contuinty = digitalRead(P2_CONT);

  // if (!contuinty == 1)
  // {
  //   Serial.print("continuity  ");
  //   tone(BUZZERPIN,4000,300);
  // }
  // else
  // {
  //   noTone(BUZZERPIN);
  // }
  // Serial.printf("cont: %d\n",contuinty);
  // delay(1000);
}