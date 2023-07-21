#include <Arduino.h>

#define TRYIMU

#include <Wire.h>

#include <BMI088.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

#include <SPIFlash.h>
#include <SD.h>

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

Adafruit_BMP3XX bmp;


#define REDLED PA3
#define GREENLED PA2
#define BLUELED PA1

#define BUZZERPIN PB0

#define SCANI2C

#define FLASHCURRENTADDRESS 0x0000f

#define FLASHCS PA4
#define SDCS PA0

// definge colors to numbers for ease of use
#define RED 0
#define GREEN 1
#define BLUE 2
#define YELLOW 3
#define PURPLE 4

SPIFlash flash(FLASHCS,0xEF16);

HardwareSerial Seria(PA10,PA9);

Sd2Card card;
SdVolume volume;
SdFile root;

struct intervals
{
  unsigned long serial;
  unsigned long fetchdata;
  unsigned long telemetry;
  unsigned long logdata;
};


intervals statedelays[7] = {
  {300,200,1000,1000}, // pad idle
  {500,50,300,50}, // ready to launch
  {500,50,100,50}, // powered flight
  {500,50,100,50}, // unpowered acenst
  {500,50,100,50}, // ballistic decent
  {500,50,100,50}, // under canopy
  {500,100,200,500} // landed
};

//#define SCANI2C

int errorflag = 1;

int state = 0;

int filenum = 0;

unsigned long serialmillis,fetchdatamillis,telemetrymillis;


void setled(int color){
  switch (color)
  {
  case 0:
    digitalWrite(REDLED,HIGH);
    digitalWrite(GREENLED,LOW);
    digitalWrite(BLUELED,LOW);
    break;
  
  case 1:
    digitalWrite(REDLED,LOW);
    digitalWrite(GREENLED,HIGH);
    digitalWrite(BLUELED,LOW);
    break;
  
  case 2:
    digitalWrite(REDLED,LOW);
    digitalWrite(GREENLED,LOW);
    digitalWrite(BLUELED,HIGH);
    break;
  
  case 3:
    digitalWrite(REDLED,HIGH);
    digitalWrite(GREENLED,HIGH);
    digitalWrite(BLUELED,LOW);
    break;
  
  case 4:
    digitalWrite(REDLED,HIGH);
    digitalWrite(GREENLED,LOW);
    digitalWrite(BLUELED,HIGH);
    break;
  
  default:
    break;
  }
}

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

void initflash(){
  flash.wakeup();
  flash.initialize();
  if (flash.readDeviceId()  == 0xEF40)
  {
    flash.blockErase4K(0);
    flash.writeByte(0x05,0x55);
    if (flash.readByte(0x05) != 0x55)
    {
      Seria.print("Flash init failure, cry\n expected 0x55 got 0x");
      Seria.println(flash.readByte(0x05),HEX);
    }
    else
    {
      Seria.println("Flash init success!!!");
      flash.readByte(0x00000f);
    }
  }
  else
  {
    Seria.print("Flash init failure, cry\nexpected device id 0xEF40 got 0x");
    Seria.println(flash.readDeviceId(),HEX);

  }
  SPI.begin();
}

void testflash(){
  Seria.println("testing flash");
  digitalWrite(FLASHCS,LOW);
  SPI.transfer(0x9F);
  byte recived = SPI.transfer(0x00);
  byte manufactured = SPI.transfer(0x00);
  byte id = SPI.transfer(0x00);
  digitalWrite(FLASHCS,HIGH);
  Seria.print("recived=");
  Seria.print(recived,HEX);
  Seria.print(" ");
  Seria.print(manufactured,HEX); 
  Seria.print(" ");
  Seria.println(id,HEX);

}

void initimu(){
  int status;
  
  status = gyro.begin(); // init gyro
  if (status < 0)// report error codes, gryo error is always -2
  {
    Seria.print("Gyro init failure, code:");
    Seria.println(status);
    // reset i2c bus because it gets jammed otherwise
    Wire.end();
    Wire.begin();
    errorflag = errorflag * 7;
  }
  else
  {
    Seria.println("Gyro init success");
  }
  
  status = accel.begin(); // init accel
  if (status < 0) // report error codes, accel error is always -5
  {
    Seria.print("Accelerometer init failure, code:");
    Seria.println(status);
    Wire.end();
    Wire.begin();
    errorflag = errorflag * 5;
  }
  else
  {
    Seria.println("Accelerometer init success");
  }
}

void initsd(){
  if (!card.init(SPI_HALF_SPEED,SDCS))
  {
    Seria.println("Card init failure");
  }
  else
  {
    Seria.println("Card present");
    Seria.println("card type: ");
    Seria.println(card.type());
    if (!SD.begin(SDCS))
    {
      Seria.println("SD init failure");
    }
    
  }
  
  SPI.begin();
  
}

void logdata(){

}

void setup() {
  //serial moniter init
  Seria.begin(9600);
  Seria.println("restart");
  //setting pinmodes
  configpins();

  //setting led defualt color
  setled(RED);

  //BMI088 initilization code
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  initimu();
  Wire.beginTransmission(0x76);
  int status = Wire.endTransmission();
  Seria.print("bmp390 chek: ");
  Seria.println(status);

  //SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  
  initflash();
  

  initsd();
  
  File logfile = SD.open("log.csv",FILE_WRITE);
  if (logfile)
  {
    accel.readSensor();
    logfile.print("itgo: ");
    logfile.println(accel.getAccelX_mss());
    Seria.print("log file created");
    logfile.close();
  }
  else
  {
    Seria.println("log file creation failed");
  }
  logfile.close();
  

  testflash();

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Seria.println("Could not find a valid BMP3 sensor, check wiring!");
    errorflag = errorflag * 3;
  }


  // buzz to indicate out of initilization
  tone(BUZZERPIN,700,100);
  delay(100);
  tone(BUZZERPIN,700,100);
  Seria.println("init");

  // indicating an error
  if (errorflag==1)
  {
    setled(GREEN);
  }
  else{
    setled(PURPLE);
    tone(BUZZERPIN,1000,100);
    delay(200);
    tone(BUZZERPIN,1000,100);
  }

  #ifdef SCANI2C
  byte error, address;
  int nDevices;
 
  Seria.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
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
  
  
  #endif   
  


  // printing the combined error flag, which can be used to identify the error by prime factoring
  Seria.print("errorflag=");
  Seria.println(errorflag);
  tone(BUZZERPIN,1500,100);
  
  while (!Seria)
  {
    delay(100);
  }
  
 
}

void loop() {

  
  if (millis() - fetchdatamillis > statedelays[state].serial)
  {
    accel.readSensor();
    

    gyro.readSensor();
  }
  
  

  if (millis() - serialmillis > statedelays[state].serial)
  {
    Seria.print("accels: ");
    Seria.print(accel.getAccelX_mss());
    Seria.print(", ");

    Seria.print(accel.getAccelY_mss());
    Seria.print(", ");
    Seria.print(accel.getAccelZ_mss());

    Seria.print(" gyro: ");

    Seria.print(gyro.getGyroX_rads());
    Seria.print(", ");
    Seria.print(gyro.getGyroY_rads());
    Seria.print(", ");
    Seria.println(gyro.getGyroZ_rads());
  }

  
  //tone(BUZZERPIN,1000,100);


}
