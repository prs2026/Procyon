#ifndef lyralibfile
#define lyralibfile

#include <Arduino.h>

#include <Wire.h>

#define REDLED PB8
#define GREENLED PB7
#define BLUELED PB9

#define BUZZERPIN PA0

#define SERVO1 PC6
#define SERVO2 PC7
#define SERVO3 PC8
#define SERVO4 PC9

#define P1EN PC10
#define P2EN PC11


#define SCL PB10
#define SDA PB11

#define SCK PB3
#define MISO PB4
#define MOSI PB5

#define CSBRKOUT PA14
#define FLASHCS PA15
#define SDCS PC12

#define TX PA2
#define RX PA3


#define ACCELINT1 PB13
#define ACCELINT2 PB12

#define GYROINT3 PB1
#define GYROINT4 PB2

#define BATTSENSEPIN PC0
#define PYROSENSEPIN PC1

// define colors to numbers for ease of use
#define RED 0
#define GREEN 1
#define BLUE 2
#define YELLOW 3
#define PURPLE 4

#define FLASHSTARTADDRESS 0x10FFF


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
  int32_t pressure; // 9
  int32_t altitude; //10
  int32_t verticalvel; //11
  int32_t appogee; //12
  uint64_t uptime; //13
  uint32_t errorflag; //14
  uint16_t bmptemp; // 15
  uint16_t imutemp; // 16
  uint8_t state; // 17
  } readable;
  uint8_t data[sizeof(readable)]; // = 13 *4  + 3*2 + 1 *1 = 59
                                 //    int32   int16  int8
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
    digitalWrite(GREENLED,LOW);
    digitalWrite(BLUELED,LOW);
    break;
  
  case 2:
    digitalWrite(REDLED,HIGH );
    digitalWrite(GREENLED,HIGH );
    digitalWrite(BLUELED,LOW);
    break;
  
  case 3:
    digitalWrite(REDLED,HIGH);
    digitalWrite(GREENLED,LOW);
    digitalWrite(BLUELED,HIGH );
    break;
  
  case 4:
    digitalWrite(REDLED,LOW);
    digitalWrite(GREENLED,HIGH );
    digitalWrite(BLUELED,LOW);
    break;
  
  default:
    break;
  }
}

#endif