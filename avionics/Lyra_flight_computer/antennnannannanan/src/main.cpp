#include <Arduino.h>
#include <macros.h>
#include <RF24.h>

RF24 radio(CE,CSN);

uint32_t dataage;
uint32_t uptime;
uint32_t previousdatapacketmillis;
uint32_t prevserialmillis;
uint32_t launchattempmillis;
uint32_t launchstartmillis;
uint32_t abortattemptmillis;

telepacket currentpacket;

bool trylaunch = false;

telepacket recivepacket(){
  uint8_t buf[32] = {};

  radio.read(buf,32);

  //Serial.print("recived packet: ");
  // for (int i = 0; i < sizeof(buf); i++)
  // {
  //   Serial.print(buf[i],HEX);
  //   Serial.print(" ");
  // }
  //Serial.println("eom");

  telepacket temppacket;
  
  if (buf[0] == 0xAB && buf[1] == 0xCD)
  {
    Serial.println("recived init packet");

    uint8_t exppayload[32] = {0xCD};
    radio.stopListening();
    radio.write(exppayload,32);
    radio.startListening();
    return temppacket;
  }


  if (buf[0] == 0x12)
  {

    Serial.println("recived data packet");
    int j = 0;
    for (int i = 0; i < 32; i++)
    {
      temppacket.data[j] = buf[j];
      j++;
    }
    
  }
  
  return temppacket;
}

void senddatatoserial(telepacket data){
  Serial.printf("AB,%f,%f,%f"//accel
  ",%f,%f,%f" // gyro
  ",%f,%f" // alt, vvel
  ",%f,%f,%f" // orientation
  ",%d,%d,%d,%d,%d,%d,CD \n", // uptime, errorflagmp, errorflagnav, dataage, selfuptimem,state
  float(data.r.accel.x)/100,float(data.r.accel.y)/100,float(data.r.accel.z)/100,
  float(data.r.gyro.x)/100,float(data.r.gyro.y)/100,float(data.r.gyro.z)/100,
  float(data.r.altitude)/10,float(data.r.verticalvel)/100,
  float(data.r.orientationeuler.x)/100,float(data.r.orientationeuler.y)/100,float(data.r.orientationeuler.z)/100,
  data.r.uptime,data.r.errorflagmp,data.r.errorflagnav,millis()-previousdatapacketmillis,millis(),data.r.state);

}

void parsecommand(char command){
  switch (command)
  {
  case 'l':
    Serial.println("trying to launch");
    trylaunch = true;
    launchstartmillis = millis();
    break;

  case 'a':{
    Serial.println("aborting launch");
    trylaunch = false;
    break;
  }
  case 'm':{
    Serial.println("moving data to sd");
    char movechar = 'm';
    radio.stopListening();
    radio.write(&movechar,1);
    radio.startListening();
    break;
  }
  case 'o':{
    Serial.println("getting new pad offsets");
    char padchar = 'o';
    radio.stopListening();
    radio.write(&padchar,1);
    radio.startListening();
    break;
  }
  default:
    break;
  }


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


  currentpacket.r.state = 0;

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

  //radio.setRetries(10,30);
  //radio.setDataRate(RF24_250KBPS);
  //radio.setPALevel(RF24_PA_MAX,true);

  radio.openWritingPipe(radioaddress[0]);
  radio.openReadingPipe(1,radioaddress[1]);

  radio.startListening();
  
  
  Serial.println("Radio init success");

}

void loop() {
    if (radio.available())
    {
      currentpacket = recivepacket();
      previousdatapacketmillis = millis();
      senddatatoserial(currentpacket);
    }

    if (millis() - prevserialmillis > 100)
    {
      senddatatoserial(currentpacket);
      prevserialmillis = millis();
    }

    if (Serial.available())
    {
      char com = Serial.read();
      Serial.printf("echo: %c \n",com);
      parsecommand(com);
    }
    
    if (millis() - launchstartmillis < 5000 && trylaunch && currentpacket.r.state < 1 && millis()- launchattempmillis > 300)
    {
      char launchchar = 'l';
      Serial.println("sending launch");
      radio.stopListening();
      radio.write(&launchchar,1);
      radio.startListening();
      launchattempmillis = millis();
    }

    if (!trylaunch && currentpacket.r.state > 0 && currentpacket.r.state < 10 && millis()- abortattemptmillis > 300)
    {
      char abortchar = 'a';
      Serial.printf("aborting launch, state: %d \n",currentpacket.r.state );
      radio.stopListening();
      radio.write(&abortchar,1);
      radio.startListening();
      abortattemptmillis = millis();
    }
    
    
    
  
}
