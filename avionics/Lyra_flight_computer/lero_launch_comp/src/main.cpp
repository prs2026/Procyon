#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <lyralib.h>

#define MOSI PB5
#define MISO PB4
#define SCK PB3


const int radiocsn = PB7;
const int radioce = PB6;

RF24 radio(radioce,radiocsn);

const uint8_t startingchecksum1 = 0xAB;
const uint8_t startingchecksum2 = 0xCD;
const uint8_t endingchecksum1 = 0x12;
const uint8_t endingchecksum2 = 0x34;

const int pyro1 = PB10;

byte radioaddress[][7] = {"flight","ground"};

uint8_t initmessage[] = {0xAB,0xCD,0xEF}; // "flight radio check"
uint8_t initreply[] = {0x12,0x34,0x56}; // "loud and clear"

int payloadnum = 0;

void initradio() {
  if (radio.begin())
    {
      Serial.println("Radio init succsess");
    }
    else
    {
      Serial.println("Radio init failure");;
      return;
    }

    radio.openReadingPipe(1,radioaddress[1]);
    radio.openWritingPipe(radioaddress[0]);
    Serial.println("radio is green.");
    radio.startListening();
}


void setup() {
  Serial.begin(115200);
  Serial.println("\n\n\nrestart");
  // put your setup code here, to run once:
  pinMode(pyro1, OUTPUT);
  pinMode(PC13,OUTPUT);
  digitalWrite(pyro1, LOW);
  SPI.setMISO(MISO);
  SPI.setMOSI(MOSI);
  SPI.setSCLK(SCK);
  SPI.begin();
  
  Serial.println(SPI.transfer(0x16),HEX);
  Serial.println(SPI.transfer(0x32),HEX);
  
  
  initradio();

  digitalWrite(PC13,HIGH);
  delay(1000);
  digitalWrite(PC13,LOW);
  Serial.println("out of init");

}



void loop() {
  if (radio.available())
    {
      Serial.print("Received: ");
      int buf[32];
      radio.read(buf,32);
      for (int i = 0; i < 32; i++)
      {
        Serial.print(buf[i],HEX);
        Serial.print(" ");
      }
      Serial.println("eom");
    }
  
  // put your main code here, to run repeatedly:
}

