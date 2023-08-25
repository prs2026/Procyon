#ifndef lyralibfile
#define lyralibfile

#include <Arduino.h>

#include <Wire.h>

#define REDLED PA3
#define GREENLED PA2
#define BLUELED PA1

struct intervals
{
  unsigned long serial;
  unsigned long fetchdata;
  unsigned long telemetry;
  unsigned long logdata;
  unsigned long beep;
};

union accelgyrobmp
{
  struct {
  int32_t accel_x; //0
  int32_t accel_y; //1
  int32_t accel_z; //2
  int32_t gyro_x; //3
  int32_t gyro_y; //4
  int32_t gyro_z; //5
  int32_t yaw; //6
  int32_t pitch; //7
  int32_t roll; //8
  int32_t imutemp; // 9
  int32_t pressure; // 10
  int32_t altitude; //11
  int32_t bmptemp; // 12
  int32_t verticalvel; //13
  int32_t appogee; //14
  } readable;
  int32_t data[15];
};


void int32tobytearray(int32_t value,uint8_t *buf){
  uint32_t adjvalue = value + 2147483647;
  buf[3] = adjvalue;
  buf[2] = adjvalue >> 8;
  buf[1] = adjvalue >> 16;
  buf[0] = adjvalue >> 24;
}

int32_t bytearraytoint32(uint8_t *array){
  int32_t value;
  value = ((int32_t)array[0] << 24) | ((int32_t)array[1] << 16) | ((int32_t)array[2] << 8) | (int32_t)array[3];
  value = value - 2147483647;
  
  return value;
}

void int16tobytearray(int16_t value,uint8_t *buf){
  uint16_t adjvalue = value + 32767;
  buf[1] = adjvalue;
  buf[0] = adjvalue >> 8;
}

int16_t bytearraytoint16(uint8_t *array){
  int16_t value;
  value = ((int16_t)array[0] << 8) | (int16_t)array[1];
  value = value - 32767;
  return value;
}

void scani2c(TwoWire Wire,HardwareSerial Seria){
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
}


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


#endif