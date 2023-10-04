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

bool trylaunch = false;

byte radioaddress[][7] = {"flight","ground"};

uint8_t initmessage[] = {0xAB,0xCD,0xEF}; // "flight radio check"
uint8_t initreply[] = {0x12,0x34,0x56}; // "loud and clear"

uint32_t prevcommcheckmillis;

accelgyrobmp recenttelemetry;
int16_t lyrastate,foxerrorflag,foxuptime;
uint32_t prevtelemetrymillis,prevserialmillis,launchattempmillis,missiontime;

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

    radio.openReadingPipe(1,radioaddress[0]);
    radio.openWritingPipe(radioaddress[1]);
    Serial.println("radio is green.");
    radio.startListening();
}

accelgyrobmp recievetelemetry() {
    uint8_t reciveddatabuf[32];
    radio.read(reciveddatabuf,32);
    /*
    Serial.println("recieved: ");
    for (int i = 0; i < 32; i++)
    {
      Serial.print(reciveddatabuf[i],HEX);
      Serial.print(" ");
    }
    */
    if (reciveddatabuf[0] == initmessage[0] && reciveddatabuf[1] == initmessage[1] && reciveddatabuf[2] == initmessage[2])
    {
      Serial.println("\ninit message recieved");
      radio.stopListening();
      radio.write(&initreply,3);
      radio.startListening();
    }
    
    /*
    data needed: total 31
    altitude 2
    vertical velocity 2
    accel xyz 6
    gyroxyz 6
    pitch roll yaw 6
    uptime 2
    apoogee 2
    state 1
    errorflag 2
    ending and start checksum 2
    
    */
   
  if (reciveddatabuf[0] == startingchecksum1 && reciveddatabuf[sizeof(reciveddatabuf)-1] == endingchecksum1)
  {
    accelgyrobmp outputdata;
    //Serial.println("\nrecieved telemetry packet");
    uint8_t index = 1;
    uint8_t dataaddress[] = {0,1,2,3,4,5,6,7,8,11,13,14};
    for (int i = 0; i < 12; i++)
    {
      //Serial.print("used: ");
      uint8_t databuf[2];
      for (int j = 0; j < 2; j++)
      {
        //Serial.print(databuf[j],HEX);
        //Serial.print(" ");
        databuf[j] = reciveddatabuf[index];
        index++;
      }
      //Serial.print(" to make: ");
      outputdata.data[dataaddress[i]] = bytearraytoint16(databuf);
      //Serial.print(outputdata.data[dataaddress[i]]);
      //Serial.print(" at address:");
      //Serial.print(dataaddress[i]);
      //Serial.println(" ");
    }
    uint8_t databuf[2];
      for (int j = 0; j < 2; j++)
      {
        //Serial.print(databuf[j],HEX);
        //Serial.print(" ");
        databuf[j] = reciveddatabuf[index];
        index++;
      }
      //Serial.print(" to make: ");
    foxuptime = bytearraytoint16(databuf); // uptime
      for (int j = 0; j < 2; j++)
      {
        //Serial.print(databuf[j],HEX);
        //Serial.print(" ");
        databuf[j] = reciveddatabuf[index];
        index++;
      }
      //Serial.print(" to make: ");
    foxerrorflag = bytearraytoint16(databuf); // errorflag
      
    lyrastate = int16_t(reciveddatabuf[index]);
    prevtelemetrymillis = millis();
    return outputdata;
  }
  
}

void broadcast(int data) {
    uint8_t commandbuf[32];
    commandbuf[0] = startingchecksum1;
    commandbuf[sizeof(commandbuf)-1] = endingchecksum1;
    commandbuf[1] = (uint8_t)data;
    Serial.print("sendingcommand, packet: ");
    for (int i = 0; i < sizeof(commandbuf)/sizeof(commandbuf[0]); i++)
    {
      Serial.print(commandbuf[i],HEX);
      Serial.print(" ");
    }
    
    radio.stopListening();
    radio.write(commandbuf,sizeof(commandbuf));
    radio.startListening();
}

void senddatatoground(){
   if (millis() - prevtelemetrymillis > 5000)
   {
    lyrastate = 7;
   }
   
    Serial.print(0xCD,HEX); Serial.print(",");
    Serial.print(millis()); Serial.print(",");
    Serial.print(foxuptime); Serial.print(",");
    Serial.print(missiontime); Serial.print(",");
    Serial.print(float(recenttelemetry.readable.altitude)/10); Serial.print(",");
    Serial.print(float(recenttelemetry.readable.verticalvel)/10); Serial.print(",");
    Serial.print(0); Serial.print(",");// find way to add batt state to telemety
    Serial.print(0); Serial.print(",");// same thing, just with pyro state
    Serial.print(lyrastate); Serial.print(",");
    Serial.print(millis() - prevtelemetrymillis); Serial.print(",");
    Serial.println(0xAB,HEX);
    prevserialmillis = millis();
}

void parsecommand(int data){
  switch (data)
  {
  case 108:
    broadcast(108);
    trylaunch = true;
    Serial.println("launcharmed");
    launchattempmillis = millis();
    break;
  
  case 109:
    broadcast(109);
    break;

  case 79:
  Serial.print("reseting fox");
  broadcast(79);
  break;

  case 122:
  Serial.print("zeroimu");
  broadcast(122);
  break;

  case 119:
  Serial.print("beep");
  broadcast(119);
  break;

  default:
    break;
  }
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
  Serial.print("out of init");
  uint8_t testarray[2] = {0x7F,0xEF};
  Serial.println("inttobytearray test: ");
  Serial.print("coverted to: ");
  Serial.print(testarray[0],HEX);
  Serial.print(" ");
  Serial.print(testarray[1],HEX);
  Serial.print("\nreconstructed to:");
  Serial.println(bytearraytoint16(testarray));
}



void loop() {

  if (Serial.available() > 0 )
  {
  byte data = Serial.read();
  Serial.print("Echo: ");
  Serial.println(data);
  parsecommand(data);
  }
  if (trylaunch == true && millis() - launchattempmillis < 5000 && lyrastate >= 1)
  {
    digitalWrite(pyro1, HIGH);
    delay(1000);
    digitalWrite(pyro1, LOW);
    Serial.println("pyro1 fire");
    trylaunch = false;
    missiontime = millis();
  }

  if (trylaunch == true && millis() - launchattempmillis > 5000)
  {
    Serial.println("launch attemp failed");
    trylaunch = false;
    missiontime = 0;
  }
  
  
  if (radio.available())
  {
    recenttelemetry = recievetelemetry();
  }
  
  if (millis()-prevserialmillis > 50)
  {
    senddatatoground();
  }

  
  // put your main code here, to run repeatedly:
}