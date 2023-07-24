#include <Arduino.h>

#define TRYIMU

#include <Wire.h>

#include <BMI088.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

#include <lyralib.h>

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

#define FLASHSTARTADDRESS 0x10FFF

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

accelgyrobmp currentsensordata;

accelgyrobmp offsets = {
  -1988,	991,	404,	22,	-55,	71
};

intervals statedelays[7] = {
  {300,50,1000,1000}, // pad idle
  {500,20,300,50}, // ready to launch
  {500,20,100,50}, // powered flight
  {500,20,100,50}, // unpowered acenst
  {500,20,100,50}, // ballistic decent
  {500,20,100,50}, // under canopy
  {500,50,200,500} // landed
};


//#define SCANI2C

uint16_t errorflag = 1;

uint32_t state = 0;

int8_t filenum = 0;

u_int64_t serialmillis,fetchdatamillis,telemetrymillis,computeyprmicros,computevvelmicros;

bool sendserial = false;

u_int32_t globaladdress;

const float localpressure = 1011.51;

const uint8_t checksum = 0xAB;
const uint8_t checksum2 = 0xCD;


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
  if (!flash.readDeviceId()  == 0xEF40)
  {
    Seria.print("Flash init failure, cry\nexpected device id 0xEF40 got 0x");
    Seria.println(flash.readDeviceId(),HEX);
    return;
  }
  /*
  flash.blockErase4K(0x00);
  flash.writeByte(0x05,0x55);
  if (flash.readByte(0x05) != 0x55)
  {
    Seria.print("Flash init failure, cry\n expected 0x55 got 0x");
    Seria.println(flash.readByte(0x05),HEX);
    return;
  }
  */
  Seria.println("Flash init success!!!");
  globaladdress = FLASHSTARTADDRESS;
  while (flash.readByte(globaladdress) != 0xFF)
  {
    /*
    Seria.print("address: 0x");
    Seria.print(address,HEX);
    Seria.print(" data: 0x");
    Seria.println(flash.readByte(address),HEX);
    */
    globaladdress++;
  }
  Seria.print("Current Flash address: 0x");
  Seria.println(globaladdress,HEX);
  Seria.print("data at address: 0x");
  Seria.println(flash.readByte(globaladdress),HEX);

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
  if (status < 0)// report error codes
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
  if (status < 0) // report error codes
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
    accel.setRange(accel.RANGE_12G);
  }
}

void initbmp(){
  Wire.beginTransmission(0x76);
  int status = Wire.endTransmission();
  Seria.print("bmp390 presence, 0 is good: ");
  Seria.println(status);
  if (!bmp.begin_I2C(0x76)) {   // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Seria.println("Could not find a valid BMP3 sensor, check wiring!");
    errorflag = errorflag * 3;
  }
  else
  {
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    Seria.println("BMP390 INIT SUCSESS!!!!");
  }
}

void initsd(){
  if (!card.init(SPI_HALF_SPEED,SDCS))
  {
    Seria.println("Card init failure");
  }
  else
  {
    Seria.print("Card present, ");
    Seria.print("card type: ");
    Seria.println(card.type());
    if (!SD.begin(SDCS))
    {
      Seria.println("SD init failure");
    }
    
  }
  
  SPI.begin();
  
}

void readdata(uint64_t currentaddress,uint32_t *readdata){
  uint32_t datasize = sizeof(accelgyrobmp)+sizeof(uint32_t)+sizeof(checksum);
  uint8_t databuf[datasize];
  uint16_t index = 0;
  Seria.println("sensordata: ");
  for (int i = 0; i < sizeof(currentsensordata)/sizeof(currentsensordata.data[0]); i++)
  {
    databuf[i] = flash.readByte(currentaddress);
    currentaddress++;
    if (i%4 == 0)
    {
      uint8_t buffer[4];
      buffer[0] = databuf[i-3];
      buffer[1] = databuf[i-2];
      buffer[2] = databuf[i-1];
      buffer[3] = databuf[i];
      readdata[index] = bytearraytoint(buffer);
      Seria.println(readdata[index]);
    }
    
  }
  
  
}

void logdata(accelgyrobmp datatolog, uint8_t statetolog){
    int datasize = sizeof(datatolog)+sizeof(statetolog)+sizeof(checksum)*2+sizeof(uint32_t);
    Seria.print("datasize: ");
    Seria.println(datasize);
    uint8_t databuf[datasize];
    uint16_t index = 0;
    uint64_t localaddress = globaladdress;
    for (int i = 0; i < 13; i++)
    {
      uint8_t buffer[4];
      
      inttobytearray(datatolog.data[i],buffer);
      for (int j = 0; j < 4; j++)
      {
        databuf[i+j] = buffer[j];
        Seria.print(databuf[i+j],HEX);
        index++;
      }
      Seria.println("");
    } 
    databuf[13] = statetolog;
    databuf[14] = checksum;
    databuf[15] = checksum2;
    for (int i = 0; i < datasize; i++)
    {
      Seria.println(databuf[i],HEX);
      flash.writeByte(localaddress,databuf[i]);
      localaddress++;
    }
    Seria.println("read: ");
    uint32_t readbuffer[20];
    readdata(globaladdress,readbuffer);
    for (int i = 0; i < sizeof(readbuffer)/sizeof(readbuffer[0]); i++)
    {
      Seria.println(readbuffer[i]);
      if (readbuffer[i] != datatolog.data[i])
      {
        errorflag = errorflag * 3;
        return;
      }
      
    }
    
}

void dumpdatatoserial(bool raw){
  uint32_t dumpaddress = FLASHSTARTADDRESS;
  while (flash.readByte(dumpaddress) != 0xFF)
  {
    switch (raw)
    {
    case true:
      Seria.print("");
      Seria.print(dumpaddress,HEX);
      Seria.print(" , ");
      Seria.println(flash.readByte(dumpaddress),HEX);
      dumpaddress++;
      break;

    case false:
      uint32_t reedbuffer[20];
      readdata(dumpaddress,reedbuffer);
      for (int i = 0; i < sizeof(reedbuffer)/sizeof(reedbuffer[0]); i++)
      {
        Seria.println(reedbuffer[i]);
      }
      dumpaddress += sizeof(accelgyrobmp)+sizeof(uint32_t)+sizeof(checksum);
    break;
    
    default:
      break;
    }

    
  }
  
  
}

accelgyrobmp fetchdata(accelgyrobmp inputdata){
  accelgyrobmp tempdata;
  int oversampling = 5;
  for (int i = 0; i < oversampling-1; i++)
  {
    accel.readSensor();
    gyro.readSensor();
    tempdata.readable.accel_x += (accel.getAccelZ_mss()*10000)-offsets.readable.accel_x;
    tempdata.readable.accel_y += (accel.getAccelY_mss()*10000)-offsets.readable.accel_y;
    tempdata.readable.accel_z += (accel.getAccelX_mss()*10000)-offsets.readable.accel_z;

    tempdata.readable.gyro_x += ((gyro.getGyroZ_rads()*10000)*(180/PI))-offsets.readable.gyro_x;
    tempdata.readable.gyro_y += ((gyro.getGyroY_rads()*10000)*(180/PI))-offsets.readable.gyro_y;
    tempdata.readable.gyro_z += ((gyro.getGyroX_rads()*10000)*(180/PI))-offsets.readable.gyro_z;
    delayMicroseconds(100);
  }
  tempdata.readable.accel_x = tempdata.readable.accel_x / oversampling;
  tempdata.readable.accel_y = tempdata.readable.accel_y / oversampling;
  tempdata.readable.accel_z = tempdata.readable.accel_z / oversampling;

  tempdata.readable.gyro_x = tempdata.readable.gyro_x / oversampling;
  tempdata.readable.gyro_y = tempdata.readable.gyro_y / oversampling;
  tempdata.readable.gyro_z = tempdata.readable.gyro_z / oversampling;

  tempdata.readable.imutemp = accel.getTemperature_C()*1000;
  float timestep = (micros() - computeyprmicros);
  tempdata.readable.yaw = inputdata.readable.yaw + ((tempdata.readable.gyro_z * timestep)/1000000);
  tempdata.readable.pitch = inputdata.readable.pitch + ((tempdata.readable.gyro_y * timestep)/1000000);
  tempdata.readable.roll = inputdata.readable.roll + ((tempdata.readable.gyro_x * timestep)/1000000);

  computeyprmicros = micros();

  tempdata.readable.pressure = bmp.readPressure()*10000;
  tempdata.readable.altitude = bmp.readAltitude(localpressure)*10000;
  tempdata.readable.bmptemp = bmp.readTemperature();
  return tempdata;
}

void generateimuoffsets(int cycles){
  accelgyrobmp tempdata;
  accelgyrobmp tempoffsets;
  accel.setRange(accel.RANGE_3G);

  for (int i = 0; i < cycles-1; i++)
  {
    accel.readSensor();
    gyro.readSensor();
    Seria.print(accel.getAccelZ_mss()*10000);
    Seria.print(" / ");
    Seria.print(accel.getAccelY_mss()*10000);
    Seria.print(" / ");
    Seria.print(accel.getAccelX_mss()*10000);
    Seria.print(" / ");
    Seria.print(gyro.getGyroZ_rads()*10000);
    Seria.print(" / ");
    Seria.print(gyro.getGyroY_rads()*10000);
    Seria.print(" / ");
    Seria.print(gyro.getGyroX_rads()*10000);
    Seria.println("");
    delayMicroseconds(100);
  }
  

  accel.setRange(accel.RANGE_12G);
}

void sendtoserialdata(accelgyrobmp inputdata){
  Seria.print("accels: ");
    Seria.print(float(inputdata.readable.accel_x)/10000,3);
    Seria.print(", ");
    Seria.print(float(inputdata.readable.accel_y)/10000,3);
    Seria.print(", ");
    Seria.print(float(inputdata.readable.accel_z)/10000,3);

    Seria.print(" gyro: ");

    Seria.print(float(inputdata.readable.gyro_x)/10000,3);
    Seria.print(", ");
    Seria.print(float(inputdata.readable.gyro_y)/10000,3);
    Seria.print(", ");
    Seria.print(float(inputdata.readable.gyro_z)/10000,3);

    Seria.print(" yaw,pitch,roll: ");
    Seria.print(float(inputdata.readable.yaw)/10000,3);
    Seria.print(", ");
    Seria.print(float(inputdata.readable.pitch)/10000,3);
    Seria.print(", ");
    Seria.print(float(inputdata.readable.roll)/10000,3);

    Seria.print(" alt: ");
    Seria.println(inputdata.readable.altitude);
}

void parsecommand(uint8_t command){
  switch (command)
  {
  case 115:
    sendserial = !sendserial;
    break;

  case 105:
  logdata(currentsensordata,state);
    break;

  case 103:
    dumpdatatoserial(false);
  break;

  case 100:
    dumpdatatoserial(true);
  break;
  
  default:
    break;
  }
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
  initbmp();


  //SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  testflash();
  initflash();
  
  initsd();


  // buzz to indicate out of initilization
  
  /*
  #ifdef SCANI2C
  scani2c(Wire,Seria);
  #endif   
  */


  // printing the combined error flag, which can be used to identify the error by prime factoring
  Seria.print("errorflag=");
  Seria.println(errorflag);
  tone(BUZZERPIN,1500,100);
  
  //generateimuoffsets(200);


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
    setled(YELLOW);
    tone(BUZZERPIN,1000,100);
    delay(200);
    tone(BUZZERPIN,1000,100);
  }
    computeyprmicros = micros();

}

void loop() {

  
  if (millis() - fetchdatamillis > statedelays[state].serial)
  {
    currentsensordata = fetchdata(currentsensordata);
  }

  if (millis() - serialmillis > statedelays[state].serial && sendserial == true)
  {
    sendtoserialdata(currentsensordata);
  }

  if (Seria.available() > 0)
  {
    int incomingbyte = Seria.read();
    Seria.print("echo: ");
    Seria.println(incomingbyte);
    parsecommand(incomingbyte);
  }
  

}
