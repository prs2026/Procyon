#if !defined(MACROS)
#define MACROS
// core libs
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <hardware/sync.h>
#include <hardware/flash.h>
#include <ArduinoEigenDense.h>

// sensor libs
#include <BMI088.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_LIS3MDL.h>

#include "SPI.h"
#include <Wire.h>

#include <string.h>

// log buf lib
#include <CircularBuffer.hpp>
#include <SX126x.h>

#include <Kalman.h>
#include <BasicLinearAlgebra.h>

#include <MicroNMEA.h>
#include <teseo_liv3f_class.h>

using namespace BLA;

// define eigen things to use
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;

// led pins
#define LEDPIN 17

// pin for buzzer
#define BUZZERPIN 16


// pyro enable pins
#define P1_EN 19
#define P2_EN 21
#define P3_EN 23
#define P4_EN 25

// pyro cont pins
#define P1_CONT 18
#define P2_CONT 20
#define P3_CONT 22
#define P4_CONT 24
// battery sense pin
#define BATT_SENSE 26
// i2c0 lines
#define SDA 12
#define SCL 13

//spi lines
#define MOSI 3
#define MISO 4
#define SPISCK 6

#define RXEN 0
#define TXEN 1
#define SXRST 2
#define SXCS 5
#define BUSY 7

#define GPSRST  11

#define BRKOUT1 10 //      SPI1 SCK  UART1 CTS  I2C1 SDA
#define BRKOUT2 27 // ADC1 SPI1 TX   UART1 RTS  I2C1 SCL
#define BRKOUT3 28 // ADC2 SPI1 RX   UART0 TX   I2C0 SDA
#define BRKOUT4 29 // ADC3 SPI1 CSN  UART0 RX   I2C0 SCL

// i2c0 is being used

// size of the buf to log to before launch is detected
#define LOGBUFSIZE (20*6)

// define the altitude to fire the main charge
#define MAINALT 400


// flash macros
#define FLASH_FILESYSTEM_SIZE 13631488 // 13MB
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - (FLASH_FILESYSTEM_SIZE-FLASH_SECTOR_SIZE))

//define basic data structs
struct Vector3float
{
    float x;
    float y;
    float z;
};

struct Vector3int{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Quatstruct{
    float w;
    float x;
    float y;
    float z;
};


// define structs to hold all data from sensors
struct IMUdata{
    Vector3float accel;
    Vector3float gyro;
    float absaccel;
    float temp;
};

struct ADXLdata{
    Vector3float accel;
    float absaccel;
};

struct BAROdata{
    float pressure;
    float altitude;
    float temp;
    float verticalvel;
    float maxrecordedalt;
    float altitudeagl;
    float padalt;
};

struct MAGdata{
    Vector3float gauss;
    Vector3float utesla;
};

struct GPSdata
{
    int sats;
};


//struct to hold various position related things so as to not bog down the main nav state struct
struct position{
    float vertaccel;
    float alt;
    float vvel;
    float maxalt;
};


// union to hold all data for the nav core that needs to be shared
union navpacket
{
    struct
    {
        uint16_t errorflag;
        uint32_t uptime;
        IMUdata imudata;
        BAROdata barodata;
        ADXLdata adxldata;
        MAGdata magdata;
        Vector3float accelworld;
        Vector3float orientationeuler;
        Quatstruct orientationquat;
        Quatstruct orientationquatadj;
        position filtered;
        Vector3float covariences;
        GPSdata gpsdata;
    } r;
};

// union for all data relating to the MP core that needs to be shared
union mpstate{
    struct{
        uint16_t errorflag;
        uint32_t uptime;
        uint32_t MET;
        uint8_t state;
        uint8_t pyrosfired;
        uint8_t pyroscont;
        uint8_t pyrostate;
        uint16_t status;
        int32_t missiontime;
        float batterystate;
    } r;
    uint8_t data8[sizeof(r)/sizeof(uint8_t)];
};

// define the struct to use to log to flash, makes adding checksums easier.
union logpacket{
    struct{
        uint8_t checksum1;
        mpstate MPstate;
        navpacket navsysstate;
        uint8_t checksum2;
    } r;
    uint8_t data[sizeof(r)/sizeof(uint8_t)];
};

// compact struct that holds the data to send to the ground station
union telepacket{
    struct 
    {
        uint8_t checksum;
        Vector3int orientationeuler;
        Vector3int accel;
        Vector3int gyro;
        int16_t altitude;
        int16_t verticalvel;
        uint32_t uptime;
        uint8_t errorflagmp;
        uint8_t errorflagnav;
        uint8_t state;
        uint8_t status;
        uint8_t checksum2;
    } r;
    uint8_t data[sizeof(r)];

};




#endif // MACROS