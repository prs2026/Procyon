#if !defined(MACROS)
#define MACROS
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <ArduinoEigenDense.h>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;

#define LEDRED 11
#define LEDGREEN 10
#define LEDBLUE 9

#define BUZZERPIN 5

#define P1_EN 6
#define P2_EN 7

#define SERVO1 12
#define SERVO2 13
#define SERVO3 14
#define SERVO4 15

#define P1_CONT 28
#define P2_CONT 27

#define BATT_SENSE 29


#define SDA 22
#define SCL 23

#define MAG_DRDY 8
#define MAG_INT 24
#define ACCEL_INT1 18
#define ACCEL_INT2 19
#define GYRO_INT3 20
#define GYRO_INT4 21
#define BARO_INT 25


#define SPI0_MISO 0
#define SPI0_SCLK 2
#define SPI0_MOSI 3

#define CS_SD 1
#define BRK_CS 4

#define UART0_TX 16
#define UART0_RX 17

#define OFF 0
#define RED 1
#define GREEN 2
#define BLUE 3

#define FLASH_TARGET_OFFSET (256*1024)


#define HANDSHAKETIMEOUT 500;

struct Vector3float
{
    float x;
    float y;
    float z;
};

struct Quatstruct{
    float w;
    float x;
    float y;
    float z;
};

struct IMUdata{
    Vector3float accel;
    Vector3float gyro;
    float temp;
};

struct BAROdata{
    float pressure;
    float altitude;
    float temp;
    float verticalvel;
    float maxrecordedalt;
};

struct MAGdata{
    Vector3float gauss;
    Vector3float utesla;
};

union navpacket
{
    struct
    {
        uint32_t errorflag;
        uint32_t uptime;
        uint32_t state;
        IMUdata imudata;
        BAROdata barodata;
        MAGdata magdata;
        Vector3float pos;
        Vector3float orientationeuler;
        Quatstruct orientationquat;
        Vector3float vel;
        
    } r;
    uint32_t data [sizeof(r)];

};

union mpstate{
    struct{
        uint32_t errorflag;
        uint32_t uptime;
        uint32_t state;
        navpacket navsysstate;
    } r;
    uint32_t data[sizeof(r)];
};



#endif // MACROS