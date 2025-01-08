#if !defined(MACROS)
#define MACROS
// core libs
#include <Arduino.h>

// sensor libs

#include <SPI.h>
#include <Wire.h>

#include <SX126x.h>
#include <Adafruit_ST7789.h>

#define BUTTON1 5
#define BUTTON2 3
#define BUTTON3 4
#define BUTTON4 6
#define BUTTON5 2

#define BCK 7
#define SCK 8
#define DIN 9
#define LRCK 10

#define LCDRST 11

#define SDA 12
#define SCL 13

#define LCDCS 14
#define LCDDC 15

#define UART0TX 16
#define UART0RX 17

#define LEDINDICATION 18

#define SXRST 2
#define BUSY 7
#define SXCS 5

#define SCLK 6
#define MOSI 3
#define MISO 4

#define LCDWR 25
#define SDCS 26

#define TXEN 1
#define RXEN 0

#define BATTSENSE 29

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

// struct to hold the variences, needed for KF 
struct variences{
    float alt;
    float vvel;
    float vaccel;
    float orientation;
};

//struct to hold various position related things so as to not bog down the main nav state struct
struct position{
    Vector3float accel;
    float alt;
    float vvel;
    float maxalt;
};


// union to hold all data for the nav core that needs to be shared
union navpacket
{
    struct
    {
        int32_t errorflag;
        uint32_t uptime;
        IMUdata imudata;
        BAROdata barodata;
        ADXLdata adxldata;
        Vector3float accelworld;
        Vector3float orientationeuler;
        Quatstruct orientationquat;
        Quatstruct orientationquatadj;
        variences uncertainty;
        position filtered;
        
    } r;
};

// union for all data relating to the MP core that needs to be shared
union mpstate{
    struct{
        int32_t errorflag;
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