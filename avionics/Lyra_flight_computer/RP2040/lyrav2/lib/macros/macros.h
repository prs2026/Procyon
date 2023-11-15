#if !defined(MACROS)
#define MACROS
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <ArduinoEigenDense.h>
#include <RF24.h>

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

#define ALTVAR 0.5 
#define VVELVAR 0.5

#define ALTNOISE 0.1
#define VVELNOISE 0.2




byte radioaddress[][7] = {"flight","ground"};


RF24 radio(26,BRK_CS);

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
    float altitudeagl;
};

struct MAGdata{
    Vector3float gauss;
    Vector3float utesla;
};


struct variences{
    float alt;
    float vvel;
};


union navpacket
{
    struct
    {
        int32_t errorflag;
        uint32_t uptime;
        uint32_t state;
        IMUdata imudata;
        BAROdata barodata;
        MAGdata magdata;
        Vector3float accelworld;
        float filteredalt;
        float filteredvvel;
        //Vector3float pos;
        Vector3float orientationeuler;
        Quatstruct orientationquat;
        Quatstruct orientationquatadj;
        variences confidence;
        //Vector3float vel;
        
    } r;
    uint32_t data[sizeof(r)/sizeof(uint32_t)];

};

union mpstate{
    struct{
        uint8_t checksum1;
        int32_t errorflag;
        uint32_t uptime;
        uint32_t state;
        navpacket navsysstate;
        uint8_t checksum2;
    } r;
    uint32_t data[sizeof(r)/sizeof(uint32_t)];
    uint8_t data8[sizeof(r)/sizeof(uint8_t)];
};

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
        uint8_t checksum2;
    } r;
    uint8_t data[sizeof(r)];

};

telepacket statetopacket(mpstate state){
    telepacket packet;
    packet.r.checksum = 0x12;
    packet.r.checksum2 = 0x34;
    packet.r.accel.x = int16_t(state.r.navsysstate.r.imudata.accel.x*100);
    packet.r.accel.y = int16_t(state.r.navsysstate.r.imudata.accel.y*100);
    packet.r.accel.z = int16_t(state.r.navsysstate.r.imudata.accel.z*100);

    packet.r.gyro.x = int16_t(state.r.navsysstate.r.imudata.gyro.x*100);
    packet.r.gyro.y = int16_t(state.r.navsysstate.r.imudata.gyro.y*100);
    packet.r.gyro.z = int16_t(state.r.navsysstate.r.imudata.gyro.z*100);

    packet.r.orientationeuler.x = int16_t(state.r.navsysstate.r.orientationeuler.x*100);
    packet.r.orientationeuler.y = int16_t(state.r.navsysstate.r.orientationeuler.y*100);
    packet.r.orientationeuler.z = int16_t(state.r.navsysstate.r.orientationeuler.z*100);

    packet.r.uptime = state.r.uptime;
    packet.r.errorflagmp = state.r.errorflag;
    packet.r.errorflagnav = state.r.navsysstate.r.errorflag;
    packet.r.state = state.r.state;

    packet.r.altitude = int16_t(state.r.navsysstate.r.barodata.altitudeagl*10);
    packet.r.verticalvel = int16_t(state.r.navsysstate.r.barodata.verticalvel*100);
    return packet;
}



#endif // MACROS