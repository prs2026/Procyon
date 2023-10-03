#include <Arduino.h>
#include <macros.h>
#include <RF24.h>

RF24 radio(CE,CSN);

void setup() {


  Serial.begin(115200);
  uint32_t serialstarttime = millis();
  while (!Serial && millis() - serialstarttime < 5000);
  delay(100);
  Serial.println("\n\nSerial init");
  SPI1.setRX(MISO);
  SPI1.setTX(MOSI);
  SPI1.setSCK(SCK);

  int error = 0; 

  while (!error)
  {
    error = radio.begin(&SPI1);
    if (error == 1)
    {
      Serial.println("Radio init success");
      break;
    }
    Serial.println("Radio init failed");
    delay(500);
    
  }

  radio.openReadingPipe(1,address[1]);
  radio.openWritingPipe(address[0]);
  
  Serial.println("Radio init success");

}

void loop() {
    if (radio.available())
    {
      int buf[32];
      radio.read(buf,32);
      Serial.print("Received: ");
      for (int i = 0; i < 32; i++)
      {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
    }
    

}
