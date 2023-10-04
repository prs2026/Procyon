#include <Arduino.h>
#include <macros.h>
#include <RF24.h>

RF24 radio(CE,CSN);

void setup() {


  Serial.begin(115200);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  uint32_t serialstarttime = millis();
  while (!Serial && millis() - serialstarttime < 5000);
  delay(100);
  Serial.println("\n\nSerial init");
  SPI1.setRX(MISO);
  SPI1.setTX(MOSI);
  SPI1.setSCK(SCK);
  SPI1.begin();

  int error = 0; 

  while (!error)
  {
    error = radio.begin(&SPI1);
    if (error == 1)
    {
      break;
    }
    Serial.println("Radio init failed");
    delay(500);
    
  }

  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1,address[0]);

  radio.startListening();
  
  
  Serial.println("Radio init success");

}

void loop() {
    if (radio.available())
    {
      Serial.print("Received: ");
      int buf[32];
      radio.read(buf,32);
      for (int i = 0; i < 32; i++)
      {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
    }
    

}
