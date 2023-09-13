#include <Arduino.h>

#define TRYIMU

#include <Wire.h>

#include <BMI088.h>

#include <Adafruit_BMP3XX.h>

#include <lyralib.h>

#include <SPIFlash.h>
#include <SD.h>

#include <RF24.h>



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
#define RADIO_CE PC15

#define PYRO1 PB11
#define PYRO2 PB10

// definge colors to numbers for ease of use
#define RED 0
#define GREEN 1
#define BLUE 2
#define YELLOW 3
#define PURPLE 4

SPIFlash flash(FLASHCS,0xEF16);

HardwareSerial Seria(PA10,PA9);

RF24 radio(RADIO_CE,PC14);

Sd2Card card;

accelgyrobmp currentsensordata;
int32_t currentabsaccel;

accelgyrobmp offsets = {
  -5141,	1662,	793,	-57,	-3319,	-18
};
intervals statedelays[7] = {
  {300,50,1000,5000,10000}, // pad idle
  {500,20,300,50,1000}, // ready to launch
  {500,20,100,50,500}, // powered flight
  {500,20,100,50,500}, // unpowered acenst
  {500,20,100,50,500}, // ballistic decent
  {500,20,100,50,500}, // under canopy
  {500,50,200,500,2000} // landed
};


//#define SCANI2C

uint32_t errorflag = 1; 
//flash 19
//gyro 7
//accel 5
//bmp 3
//sd 11
//radio 13 and 17
//starting checksum fail 23
//ending checksum fail 29
//cant open sd file 31


uint32_t state = 0;

int8_t filenum = 0;

u_int64_t serialmillis,fetchdatamillis,telemetrymillis,computeyprmicros,computevvelmicros,logdatamillis,beeeeepmillis,statechangemillis,pyro1firemillis;

int32_t prevbaroalt;

int32_t prevverticalvels[10];
uint8_t prevverticalvelsindex = 0;

bool sendserial = false;

u_int32_t globaladdress;

const float localpressure = 1015.92;

const uint8_t startingchecksum1 = 0xAB;
const uint8_t startingchecksum2 = 0xCD;
const uint8_t endingchecksum1 = 0x12;
const uint8_t endingchecksum2 = 0x34;

bool readableserial = true;

uint8_t statechangetryies;

uint8_t pyro1fire;

byte radioaddress[][7] = {"flight","ground"};

void configpins(){
  pinMode(BUZZERPIN,OUTPUT);
  pinMode(REDLED,OUTPUT);
  pinMode(GREENLED,OUTPUT);
  pinMode(BLUELED,OUTPUT);
  pinMode(FLASHCS,OUTPUT);
  pinMode(SDCS,OUTPUT);
  pinMode(PB2,OUTPUT);
  pinMode(PYRO1,OUTPUT);
  digitalWrite(FLASHCS,HIGH);
  digitalWrite(SDCS,HIGH);
}


void findglobaladdress(){
  globaladdress = FLASHSTARTADDRESS;
  flash.writeByte(globaladdress,0x11);
  while (flash.readByte(globaladdress) != 0xFF || flash.readByte(globaladdress+1) != 0xFF || flash.readByte(globaladdress+3) != 0xFF || flash.readByte(globaladdress+4) != 0xFF)
  {
    globaladdress += 10000;
  }
  while (flash.readByte(globaladdress) == 0xFF)
  {
    globaladdress -= 200;
  }
  while (flash.readByte(globaladdress) != 0xFF || flash.readByte(globaladdress+1) != 0xFF || flash.readByte(globaladdress+3) != 0xFF || flash.readByte(globaladdress+4) != 0xFF)
  {
    globaladdress++;
  }
  
  
  Seria.print("Current Flash address: 0x");
  Seria.println(globaladdress,HEX);
  Seria.print("data at address: 0x");
  Seria.println(flash.readByte(globaladdress),HEX);
}


void initflash(){
  flash.wakeup();
  flash.initialize();
  if (!flash.readDeviceId()  == 0xEF40)
  {
    Seria.print("Flash init failure, cry\nexpected device id 0xEF40 got 0x");
    Seria.println(flash.readDeviceId(),HEX);
    errorflag = errorflag * 19;
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
  findglobaladdress();
  

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
    return;
  }
  else
  {
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    Seria.println("BMP390 INIT SUCSESS!!!!");
  }
  

}

void zerobmp(int calibration){
  int64_t currentalt = 0;
  //Seria.println(currentalt);
  for (int i = 0; i < calibration; i++)
  {
    //Seria.print(currentalt);
    //Seria.print(" ");
    currentalt += bmp.readAltitude(localpressure)*10000;
    delay(5);
  }
  offsets.readable.altitude = (int32_t)(currentalt / (calibration));
  Seria.print("BMP390 calibrated, gl = ");
  Seria.println(offsets.readable.altitude);
  //Seria.println(currentalt);
  currentalt = 0;
  for (int i = 0; i < calibration; i++)
  {
    //Seria.print(currentalt);
    //Seria.print(" ");
    currentalt += (bmp.readAltitude(localpressure)*10000)-offsets.readable.altitude;
    delay(5);
  }
  offsets.readable.altitude += (int32_t)(currentalt / (calibration));
  Seria.print("BMP390 calibrated, gl = ");
  Seria.println(offsets.readable.altitude);
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
      errorflag = errorflag * 11;
    }
    
  }
  
  SPI.begin();
  
}


void initradio(){
    if (radio.begin())
    {
      Seria.println("Radio init succsess");
    }
    else
    {
      Seria.println("Radio init failure");
      errorflag = errorflag * 13;
      return;
    }

    radio.openReadingPipe(1,radioaddress[1]);
    radio.openWritingPipe(radioaddress[0]);

    uint8_t teststr[] = {0xAB,0xCD,0xEF}; // "flight radio check"
    uint8_t strtomatch[] = {0x12,0x34,0x56}; // "loud and clear"

    radio.write(&teststr,sizeof(teststr));
    radio.startListening();
    uint32_t timeoutstart = millis();
    while (!radio.available()){
      if (millis() - timeoutstart > 1000){
        Seria.println("radio commcheck timeout");
        errorflag = errorflag*17;
        return;
      }
      
    }
    uint8_t buf[32];
    radio.read(buf,sizeof(buf));
    for (int i = 0; i < sizeof(strtomatch); i++)
    {
      if (buf[i] != strtomatch[i])
      {
        Seria.println("radio comm check mismatch");
        Seria.print("expected: ");
        for (int j = 0; j < sizeof(strtomatch); j++){
          Seria.print(strtomatch[j],HEX);
        }
        Seria.print(" got: ");
        for (int j = 0; j < sizeof(buf); j++){
          Seria.print(buf[j],HEX);
        }

        errorflag = errorflag*11;
        return;
      }
      
    }
    Seria.println("radio is green.");
}


void sendtelemetry(accelgyrobmp datatolog, uint8_t statetolog){
  uint8_t datasize8bit = sizeof(datatolog)+sizeof(statetolog) +sizeof(millis());
  uint8_t telemetrydata[datasize8bit];
  telemetrydata[0] = startingchecksum1;
  uint8_t index = 1;
  int16_t datatotransmit[15];
  uint8_t dataaddress[] = {0,1,2,3,4,5,6,7,8,11,13,14};
  uint8_t buf[2];
  for (int i = 0; i < 12; i++)
  {
    uint8_t convertbuf[2];
    int16tobytearray((int16_t)(datatolog.data[dataaddress[i]]/1000),convertbuf);
    telemetrydata[index] = convertbuf[0];
    telemetrydata[index+1] = convertbuf[1];
    index += 2;
  }
  
  int16tobytearray(millis(),buf);
  telemetrydata[12] = buf[0];
  telemetrydata[13] = buf[1];
  index += 2;
  int16tobytearray(errorflag,buf);
  telemetrydata[index] = buf[0];
  telemetrydata[index+1] = buf[1];
  index += 2;
  telemetrydata[index] = statetolog;
  telemetrydata[31] = endingchecksum1;
  /*
  Seria.print("sent packet: ");
  for (int i = 0; i < sizeof(telemetrydata); i++)
  {
    Seria.print(telemetrydata[i],HEX);
    Seria.print(" ");
  }
  
  Seria.println("")
  */
  radio.stopListening();
  radio.write(telemetrydata,datasize8bit);
  radio.startListening();
}


uint8_t readdata(uint64_t currentaddress,int32_t *readdata){
  uint32_t datasize = sizeof(accelgyrobmp)+sizeof(uint32_t)+sizeof(uint8_t);
  uint8_t databuf[datasize];
  uint64_t localaddress = currentaddress;
  uint8_t index = 0;
  if (flash.readByte(localaddress) != startingchecksum1 || flash.readByte(localaddress+1) != startingchecksum2 ){
    Seria.print("starting checksum failure, expected: 0x");
    Seria.print(startingchecksum1,HEX);
    Seria.print(startingchecksum2,HEX);
    Seria.print(", got: 0x");
    Seria.print(flash.readByte(localaddress),HEX);
    Seria.println(flash.readByte(localaddress+1),HEX);
    Seria.print("at address: ");
    Seria.print(localaddress,HEX);
    errorflag = errorflag * 23;
    return 1;
  }
  localaddress += 2;
  //Seria.println("readdata from: ");
  for (int i = 0; i < 15; i++){
    uint8_t buffer[4];
    for (int j = 0; j < 4; j++){
      buffer[j] = flash.readByte(localaddress);
      /*
      Seria.print(localaddress,HEX);
      Seria.print(", ");
      Seria.print(buffer[j],HEX);
      Seria.print(", ");
      */
      localaddress++;
    }

    readdata[i] = bytearraytoint32(buffer);
    if (readdata[i] <= -2147400000)
    {
      readdata[i] = 0;
    }
    
    //Seria.print(" reconstructed to: ");
    //Seria.println(readdata[i]);
  }
  readdata[15] = flash.readByte(localaddress); //get state
  localaddress++;
  uint8_t errorflagbuf[4];
  for (int i = 0; i < 4; i++)
  {
    errorflagbuf[i] = flash.readByte(localaddress);
    localaddress++;
  }
  readdata[16] = bytearraytoint32(errorflagbuf); // errorflag
  //Seria.print("state: ");
  //Seria.println(readdata[13]);
  uint8_t buffer[4];
  for (int i = 0; i < 4; i++)
  {
    buffer[i] = flash.readByte(localaddress);
    
    //Seria.print(localaddress,HEX);
    //Seria.print(", ");
    //Seria.print(buffer[i],HEX);
    //Seria.print(", ");
    
    localaddress++;
    
    
  }
  readdata[17] = bytearraytoint32(buffer);// uptime
  //Seria.print(" reconstructed to: ");
 //Seria.println(readdata[14]);
  if (flash.readByte(localaddress) != endingchecksum1 || flash.readByte(localaddress+1) != endingchecksum2 ){
  
    Seria.print("ending checksum failure, expected: 0x");
    Seria.print(endingchecksum1,HEX);
    Seria.print(endingchecksum2,HEX);
    Seria.print(", got: 0x");
    Seria.print(flash.readByte(localaddress),HEX);
    Seria.print(flash.readByte(localaddress+1),HEX);
    Seria.print("at address: ");
    Seria.print(localaddress,HEX);
    errorflag = errorflag * 29;
    return 2;
  }
  return 0;
}

void logdata(accelgyrobmp datatolog, uint8_t statetolog, uint32_t errorflagtolog ){
    uint16_t datasize8bit = sizeof(accelgyrobmp)+sizeof(uint32_t)+sizeof(statetolog); // plus errorflag but its not in loop so not added
    //Seria.print("datasize: ");
    //Seria.println(datasize8bit);
    uint8_t databuf[datasize8bit];
    uint8_t index = 0;
    // assign values of datatolog to 4 byte buffer, then add to databuf
    for (int i = 0; i < 15; i++) //change 15 if more values are added
    {
      uint8_t buffer[4];
      int32tobytearray(datatolog.data[i],buffer);
      /*
      Seria.print("expected deci: ");
      Seria.print(datatolog.data[i]);
      Seria.print(" hex: ");
      Seria.print(datatolog.data[i],HEX);
      Seria.print(" coverted : ");
      
      for (int k = 0; k < 4; k++)
      {
        Seria.print(buffer[k],HEX);
      }
      Seria.print(" assigned: ");
      */
      for (int j = 0; j < 4; j++)
      {
        databuf[index] = buffer[j];
        //Seria.print(databuf[index],HEX);
        index++;
      }
      //Seria.print(" index: ");
      //Seria.println(index);
    }
    databuf[index] = statetolog;
    index++;
    
    uint32_t uptime = millis();
    uint8_t uptimebuffer[4];
    int32tobytearray(uptime,uptimebuffer);
    for (int i = 0; i < 4; i++)
    {
      databuf[index] = uptimebuffer[i];
      index++;
    }
    uint64_t localaddress = globaladdress;
    //Seria.print("wrote to address: ");
    //Seria.println(localaddress,HEX);
    flash.writeByte(localaddress,startingchecksum1);
    localaddress++;
    flash.writeByte(localaddress,startingchecksum2);
    localaddress++;
    for (int i = 0; i < datasize8bit; i++)
    {
      //Seria.print("wrote 0x");
      //Seria.print(databuf[i],HEX);
      //Seria.print(" to address: ");
      //Seria.println(localaddress,HEX);
      flash.writeByte(localaddress,databuf[i]);
      localaddress++;
    }
    uint8_t errorflagbuffer[4];
    int32tobytearray(errorflagtolog,errorflagbuffer);
    for (int i = 0; i < 4; i++){
      flash.writeByte(localaddress,errorflagbuffer[i]);
      localaddress++;
    }
    flash.writeByte(localaddress,endingchecksum1);
    localaddress++;
    flash.writeByte(localaddress,endingchecksum2);
    localaddress++;
    globaladdress = localaddress;
}
/*
void dumpdatatoserial(bool raw){
  int32_t dumpaddress = FLASHSTARTADDRESS;
  while (dumpaddress < globaladdress)
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
      int32_t reedbuffer[20];
      readdata(dumpaddress,reedbuffer);
      for (int i = 0; i < sizeof(reedbuffer)/sizeof(reedbuffer[0]); i++)
      {
        Seria.println(reedbuffer[i]);
      }
      dumpaddress += sizeof(accelgyrobmp)+sizeof(uint32_t)+sizeof(startingchecksum1);
    break;
    
    default:
      break;
    }

    
  }
  
  
}
*/
void clearflash(){
  flash.chipErase();
  while (flash.busy())
  {
    delay(10);
  }
  globaladdress == FLASHSTARTADDRESS;
  findglobaladdress();
  Seria.print("flash cleared");
}


accelgyrobmp fetchdata(accelgyrobmp inputdata){
  accelgyrobmp tempdata;
  int oversampling = 10;
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
    for (int i = 0; i < 6; i++)
    {
      if (tempdata.data[i] > 100000000 || tempdata.data[i] < -100000000)
      {
        tempdata.data[i] -= tempdata.data[i];
        tempdata.data[i] += inputdata.data[i];
      }
    }
    delayMicroseconds(100);
  }
  tempdata.readable.accel_x = tempdata.readable.accel_x / oversampling;
  tempdata.readable.accel_y = tempdata.readable.accel_y / oversampling;
  tempdata.readable.accel_z = tempdata.readable.accel_z / oversampling;

  tempdata.readable.gyro_x = tempdata.readable.gyro_x / oversampling;
  tempdata.readable.gyro_y = tempdata.readable.gyro_y / oversampling;
  tempdata.readable.gyro_z = tempdata.readable.gyro_z / oversampling;

  currentabsaccel = sqrt(pow(tempdata.readable.accel_x,2) + pow(tempdata.readable.accel_y,2) + pow(tempdata.readable.accel_z,2));

  tempdata.readable.altitude = (bmp.readAltitude(localpressure)*10000)-offsets.readable.altitude;

  tempdata.readable.altitude > tempdata.readable.appogee ? tempdata.readable.appogee = tempdata.readable.altitude : tempdata.readable.appogee = inputdata.readable.appogee;
    
  tempdata.readable.imutemp = accel.getTemperature_C()*10000;
  float timestep = (micros() - computeyprmicros);

  tempdata.readable.roll = inputdata.readable.roll + ((tempdata.readable.gyro_z * timestep)/1000000);
  tempdata.readable.pitch = inputdata.readable.pitch + ((tempdata.readable.gyro_y * timestep)/1000000);
  tempdata.readable.yaw = inputdata.readable.yaw + ((tempdata.readable.gyro_x * timestep)/1000000);
  tempdata.readable.verticalvel = ((tempdata.readable.altitude - prevbaroalt)/timestep)*1000000;

  prevverticalvels[prevverticalvelsindex] = tempdata.readable.verticalvel;
  prevverticalvelsindex++;
  prevverticalvelsindex > 5 ? prevverticalvelsindex = 0 : prevverticalvelsindex;
  int64_t verticalvel = 0;
  for (int i = 0; i < 10; i++)
  {
    verticalvel += prevverticalvels[i];
  }

  verticalvel = verticalvel/10;
  tempdata.readable.verticalvel = verticalvel;

  computeyprmicros = micros();

  tempdata.readable.pressure = bmp.readPressure()*10000;

  tempdata.readable.bmptemp = bmp.readTemperature()*10000;
  prevbaroalt = tempdata.readable.altitude;
  return tempdata;
}

void generateimuoffsets(int cycles){
  accelgyrobmp tempdata;
  accelgyrobmp tempoffsets;
  //accel.setRange(accel.RANGE_3G);

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
  

  //accel.setRange(accel.RANGE_12G);
}


void sendtoserialdata(accelgyrobmp inputdata,bool readable){
  if (readable)
  {
    Seria.print(">accel: ");
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
    Seria.print(float(inputdata.readable.altitude)/10000,3);

    Seria.print(" vertical velocity: ");
    Seria.print(float(inputdata.readable.verticalvel),3);

    Seria.print("state:");
    Seria.println(state);
  }
  else{
    Seria.print(">accelx: ");
    Seria.println(float(inputdata.readable.accel_x)/10000,4);
    Seria.print(">accely: ");
    Seria.println(float(inputdata.readable.accel_y)/10000,4);
    Seria.print(">accelz: ");
    Seria.println(float(inputdata.readable.accel_z)/10000,4);
    Seria.print(">absaccel: ");
    Seria.println(float(currentabsaccel)/10000,4);
    Seria.print(">gyrox: ");
    Seria.println(float(inputdata.readable.gyro_x)/10000,4);
    Seria.print(">gyroy: ");
    Seria.println(float(inputdata.readable.gyro_y)/10000,4);
    Seria.print(">gyroz: ");
    Seria.println(float(inputdata.readable.gyro_z)/10000,4);
    Seria.print(">yaw: ");
    Seria.println(float(inputdata.readable.yaw)/10000,4);
    Seria.print(">pitch: ");
    Seria.println(float(inputdata.readable.pitch)/10000,4);
    Seria.print(">roll: ");
    Seria.println(float(inputdata.readable.roll)/10000,4);
    Seria.print(">imutemp: ");
    Seria.println(float(inputdata.readable.imutemp)/10000,4);
    Seria.print(">pressure: ");
    Seria.println(float(inputdata.readable.pressure)/10000,4);
    Seria.print(">altitude: ");
    Seria.println(float(inputdata.readable.altitude)/10000,4);
    Seria.print(">apoogee: ");
    Seria.println(float(inputdata.readable.appogee)/10000,4);
    Seria.print(">bmptemp: ");
    Seria.println(float(inputdata.readable.bmptemp)/10000,4);
    Seria.print(">state: ");
    Seria.println(state);
    Seria.print(">uptime: ");
    Seria.println(millis());
    Seria.print(">verticalvel: ");
    Seria.println(float(inputdata.readable.verticalvel)/10000,4);
    Seria.print(">statechangetries: ");
    Seria.println(statechangetryies);
  }
  
    
}


void movedatatosd(uint64_t startingaddress,bool erase){
    char filename[5] = "1";
    char fileextensino[] = ".csv";
    char filenamefull[30] = "logfile";
    char filenametouse[30] = "logfile1.csv";
    strcat(filenamefull,filename);
    strcat(filenamefull,fileextensino);
    int filenum = 2;
    while (SD.exists(filenamefull))
    { 
      char filename[5];
      char filenamefull[20] = "logfile";
      itoa(filenum,filename,10);
      strcat(filenamefull,filename);
      strcat(filenamefull,fileextensino);
      filenum++;
      filenum == 10 ? filenum = 11 : filenum;
      Seria.println(filenamefull);
      if (SD.exists(filenamefull) == false)
      {
        strcpy(filenametouse,filenamefull);
        break;
        
      }
      
    }
    Seria.println(filenametouse);
    
    uint64_t localaddress = startingaddress;
    File logfile = SD.open(filenametouse,FILE_WRITE);
    if (logfile){

      logfile.println("rkf,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,yaw,pitch,roll,imutemp,pressure,altitude,bmptemp,verticalvelocity,apoogee,state,uptime(milliseconds),errorflag");
      //Seria.println("rkf,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,yaw,pitch,roll,imutemp,pressure,altitude,bmptemp,verticalvelocity,appogee,state,uptime(milliseconds),errorflag");
      while (localaddress < globaladdress)
      {
        int32_t readdatabuf[18];
        while (readdata(localaddress,readdatabuf) != 0)
        {
          localaddress++;
        }
        (localaddress,readdatabuf);
        char stringtoprint[200] = "0";
        for (size_t i = 0; i < 15; i++)
        {
          char numberbuffer[11];
          dtostrf(float(readdatabuf[i])/10000,10,4,numberbuffer);
          strcat(stringtoprint,",");
          strcat(stringtoprint,numberbuffer);
          //Seria.println(numberbuffer);
          
        }
        char numberbuffer[10] = "";
        strcat(stringtoprint,",");
        itoa(readdatabuf[15],numberbuffer,10); // get state 
        strcat(stringtoprint,numberbuffer);
        strcat(stringtoprint,",");
        itoa(readdatabuf[16],numberbuffer,10); // get errorflag 
        strcat(stringtoprint,numberbuffer);
        strcat(stringtoprint,",");
        itoa(readdatabuf[17],numberbuffer,10); // get uptime 
        strcat(stringtoprint,numberbuffer);
        //Seria.println(stringtoprint);
        logfile.println(stringtoprint);
        localaddress += 73;
      }
      
    }
    else
    {
      Seria.println("Error opening file");
      errorflag * 31;
    }
  Seria.println("done");
  logfile.close();
}


void parsecommand(uint8_t command){
  switch (command)
  {
  case 115:
    sendserial = !sendserial;
    break;

  case 114:
  readableserial = !readableserial;
  break;

  case 104:
    generateimuoffsets(1000);
  break;

  case 99:
    if (Seria.read() == 108 && Seria.read() == 114)
    {
      Seria.println("eraseing flash");
      clearflash();
    }
  break;

  case 98:
  zerobmp(50);
  Seria.print("zeroing baro");
  break;

  case 122:
  currentsensordata.readable.pitch = 0;
  currentsensordata.readable.yaw = 0;
  currentsensordata.readable.roll = 0;
  Seria.print("zero ypr");
  break;

  //case 119:
  //Seria.print("beeeep");
  //tone(BUZZERPIN,4000,100);
  //break;

  case 116:
  movedatatosd(FLASHSTARTADDRESS,false);
  break;
  
  case 105:
  logdata(currentsensordata,state,errorflag);
    break;

  //case 103:
  //  dumpdatatoserial(false);
  //break;

  case 100:
    //dumpdatatoserial(true);
  break;

  case 97:
  Seria.print("current address: 0x");
  Seria.println(globaladdress,HEX);
  break;

  case 108:
    state = 1;
    Seria.println("ready to launch");
  break;

  case 79:
  break;

  case 112:
  if (Seria.read() == 49)
  {
    Seria.print("firepyro1");
    digitalWrite(PYRO1,HIGH);
    pyro1fire = 1;
    pyro1firemillis = millis();
  }
  break;


  default:
    break;
  }
}

void detectstatechange(int8_t currentstate){
  switch (state)
  {
  case 1: // detect liftoff
    currentabsaccel > 110000 ? statechangetryies++ : statechangetryies = 0;
    statechangetryies > 3 ? state = 2, statechangetryies = 0 : state;
    break;

  case 2: //detect burnout
    currentabsaccel < 60000 ? statechangetryies++ : statechangetryies = 0;
    statechangetryies > 5 ? state = 3, statechangetryies = 0 : state;
  break;

  case 3: // detect appogee
    currentsensordata.readable.altitude < currentsensordata.readable.appogee - currentsensordata.readable.appogee * 0.1 ? statechangetryies++ : statechangetryies = 0;
    statechangetryies > 5 ? state = 4,statechangetryies = 0 : state;
    pyro1fire = 1;
    pyro1firemillis = millis();
    digitalWrite(PYRO1,HIGH);
  break;

  case 4: // detect parachute deploy
    currentabsaccel > 70000 ? statechangetryies ++ : statechangetryies = 0;
    statechangetryies > 5 ? state = 5, statechangetryies = 0 : state;
  break;

  case 5: //detect landing
    currentsensordata.readable.verticalvel < 2000 ? statechangetryies++ : statechangetryies = 0;
    statechangetryies > 10 ? state = 6, statechangetryies = 0 : state;
  break;
  
  default:
    break;
  }
}

void recivecommand(){
  uint8_t commandbuf[32];
  radio.read(commandbuf,32);
  if (commandbuf[0] == startingchecksum1 && commandbuf[sizeof(commandbuf)-1] == endingchecksum1)
  {
    Seria.println("recieved command");
    for (int i = 0; i < sizeof(commandbuf)/sizeof(commandbuf[0]); i++)
    {
      Seria.print(commandbuf[i],HEX);
      Seria.print(" ");
    }
    parsecommand(commandbuf[1]);
  }
  
} 


void setup() {
  //serial moniter init
  Seria.begin(115200);
  Seria.println("\n\nrestart");
  //setting pinmodes
  configpins();
  pinMode(PB2,INPUT_PULLDOWN);

  //setting led defualt color
  setled(RED);

  //BMI088 initilization code
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  initimu();
  initbmp();
  zerobmp(20);


  //SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  testflash();
  initflash();
  
  initsd();

  initradio();
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
    setled(RED);
    tone(BUZZERPIN,1000,100);
    delay(200);
    tone(BUZZERPIN,1000,100);
  }

  currentsensordata = fetchdata(currentsensordata);
  sendtelemetry(currentsensordata,state);

    computeyprmicros = micros();

}

void loop() {

  
  if (millis() - fetchdatamillis > statedelays[state].serial)
  {
    currentsensordata = fetchdata(currentsensordata);
  }

  if (millis() - serialmillis > statedelays[state].serial && sendserial == true)
  {
    sendtoserialdata(currentsensordata,readableserial);
  }

  if (Seria.available() > 0)
  {
    int incomingbyte = Seria.read();
    Seria.print("echo: ");
    Seria.println(incomingbyte);
    parsecommand(incomingbyte);
  }
  
  if (digitalRead(PB2) == HIGH)
  {
    tone(BUZZERPIN,1000,100);
  }

  if (millis() - logdatamillis > statedelays[state].logdata)
  {
    logdata(currentsensordata,state,errorflag);
    logdatamillis = millis();
  }
  
  if (millis() - telemetrymillis > statedelays[state].telemetry)
  {
    sendtelemetry(currentsensordata,state);
    telemetrymillis = millis();
  }
  
  if (millis() - statechangemillis > 10)
  {
    detectstatechange(state);
  }
  

  if (radio.available())
  {
    recivecommand();
  }

  if (millis() - beeeeepmillis > statedelays[state].beep)
  {
    tone(BUZZERPIN,2000,100);
    beeeeepmillis = millis();
  }

  if (errorflag==1)
  {
    setled(GREEN);
  }
  else{
    setled(RED);
  }

  if (millis() - pyro1firemillis > 4000 && pyro1fire == 1)
  {
    pyro1fire = 0;
    digitalWrite(PYRO1,LOW);
  }
  
  
  
}