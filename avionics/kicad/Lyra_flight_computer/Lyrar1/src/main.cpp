#include <Arduino.h>

#include <Wire.h>

#include <BMI088.h>

#include <Adafruit_BMP3XX.h>

#include <lyralibr1.h>

#include <SPIFlash.h>
#include <SD.h>

#include <RF24.h>

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

Adafruit_BMP3XX bmp;

HardwareSerial Seria(P2EN,P1EN);

SPIFlash flash(FLASHCS,0xEF16);

RF24 radio(CSBRKOUT,PC14);

Vector3 acceldata,gyrodata,orientationeuler;

uint8_t state;

uint16_t errorflag;

void configpins(){
  pinMode(BUZZERPIN,OUTPUT);
  pinMode(REDLED,OUTPUT);
  pinMode(GREENLED,OUTPUT);
  pinMode(BLUELED,OUTPUT);
  pinMode(FLASHCS,OUTPUT);
  pinMode(SDCS,OUTPUT);
  pinMode(PB2,OUTPUT);
  digitalWrite(FLASHCS,HIGH);
  digitalWrite(SDCS,HIGH);
}

void scani2c(){
    byte error, address;
    int nDevices;
 
    Seria.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Seria.print("looping at addr: 0x");
    Seria.println(address,HEX);
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Seria.print("I2C device found at address 0x");
      if (address<16)
        Seria.print("0");
      Seria.print(address,HEX);
      Seria.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Seria.print("Unknown error at address 0x");
      if (address<16)
        Seria.print("0");
      Seria.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Seria.println("No I2C devices found\n");
  else
    Seria.println("done\n");
}

int initflash(){
  Seria.println("initing flash");
  digitalWrite(FLASHCS,LOW);
  Seria.println(SPI.transfer(0x9F),HEX);
  Seria.println(SPI.transfer(0x00),HEX);
  Seria.println(SPI.transfer(0x00),HEX);
  digitalWrite(FLASHCS,HIGH);
}

int logdata(){
  uint8_t datasize = sizeof(millis()) + (sizeof(Vector3)*3) + sizeof(errorflag) + sizeof(state);
                  //       uptime         accel,gyro,euler           errorflag,         state
  uint8_t datatolog[datasize];
  uint16_t index = 2;
  datatolog[index] = STARTINGCHECKSUM1;
  datatolog[index] = STARTINGCHECKSUM2;
  uint8_t databuf[4];
  int32tobytearray(millis(),databuf);
  datatolog[index] = databuf[0];
  datatolog[index+1] = databuf[1];
  datatolog[index+2] = databuf[2];
  datatolog[index+3] = databuf[3];
  index += 4;
  


  for (int i = 0; i < datasize; i++)
  {
    Seria.println(datatolog[i]);
  }
  
}

void setup() {
  // put your setup code here, to run once:
  Seria.begin(115200);
  Seria.println("\n\nrestart");
  Wire.setSCL(PB10);
  Wire.setSDA(PB11);
  Wire.setClock(100000);

  SPI.setMISO(MISO);
  SPI.setMOSI(MOSI);
  SPI.setSCLK(SCK);
  SPI.begin();

  initflash();

  Wire.begin();
  Seria.println("scanning i2c");
  scani2c();

  configpins();
  digitalWrite(BLUELED,LOW);
  //setled(BLUE);
  
}

void loop() {
  setled(BLUE);
  delay(1000);
  setled(GREEN);
  delay(1000);
  // put your main code here, to run repeatedly:
}

