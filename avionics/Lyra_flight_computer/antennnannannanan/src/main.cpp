#include <Arduino.h>
#include <macros.h>
#include <RF24.h>

RF24 radio(CE,CSN);

void recivepacket(){
  uint8_t buf[32];
  radio.read(buf,32);

  Serial.println("recived packet: ");
  for (int i = 0; i < 32; i++)
  {
    Serial.print(buf[i],HEX);
    Serial.print(" ");
  }
  Serial.println("eom");


  if (buf[0] == 0xAB && buf[1] == 0xCD)
  {
    Serial.println("recived init packet");

    uint8_t exppayload[32] = {0xCD};
    radio.stopListening();
    radio.write(exppayload,32);
    radio.startListening();
  }
  return;
}

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
    if (error)
    {
      break;
    }
    Serial.println("Radio init failed");
    delay(500);
    
  }

  radio.setRetries(10,30);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX,true);
  radio.openWritingPipe(radioaddress[0]);
  radio.openReadingPipe(1,radioaddress[1]);

  radio.startListening();
  
  
  Serial.println("Radio init success");

}

void loop() {
    if (radio.available())
    {
      recivepacket();
    }
    

}
