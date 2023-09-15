#if !defined(MACROS)
#define MACROS
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "Lyrav2sensors.h"

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


#define HANDSHAKETIMEOUT 500;

struct Vector3int32
{
    int32_t x;
    int32_t y;
    int32_t z;
};

struct IMUdata{
    Vector3int32 accel;
    Vector3int32 gyro;
    int32_t temp;
};

struct BAROdata{
    uint32_t pressure;
    uint32_t altitude;
    int32_t temp;
};

struct MAGdata{
    Vector3int32 gauss;
    Vector3int32 utesla;
    int32_t heading;
    int32_t tilt;
};



#endif // MACROS