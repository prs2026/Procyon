#if !defined(NAVCOREHEADER)
#define NAVCOREHEADER

#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "Lyrav2sensors.h"
#include "macros.h"
#include "SPI.h"
#include "SD.h"
#include <string.h>
#include "LittleFS.h"
//#include <ArduinoEigenDense.h>
#include <generallib.h>

IMU imu;
BARO baro;
MAG mag;

using Eigen::MatrixXd;
using Eigen::Vector3d;


class NAVCORE{
    

    public:
        NAVCORE(){};
        uint32_t errorflag = 1; 
        navpacket _sysstate;
        /*
        1 = no errors
        3 = failed handshake
        5 = i2c devices fail
        7 = accel init fail
        11 = gyro init fail
        13 = baro init fail
        17 = mag init fail
        19 = packet send fail
        */
        struct timings{
            uint32_t sendpacket;
            uint32_t intergrateorientation;
        };
        timings intervals[7] = {
            {50}, // ground idle
            {50}, // launch detect
            {50}, // powered ascent
            {50}, // unpowered ascent
            {50}, // ballistic descent
            {50}, //ready to land
            {50} // landed
        }; 
        timings prevtime;


        int sendpacket(navpacket datatosend){
            if (rp2040.fifo.available() > 250)
            {
                return 1;
            }
            
            rp2040.fifo.push(0xAB);
            for (int i = 0; i < sizeof(datatosend.data)/sizeof(datatosend.data[0]); i++)
            {
                bool error = 1;
                int j = 0;
                rp2040.fifo.push(datatosend.data[i]);
                
            }
            rp2040.fifo.push(0xCD);
            return 0;
        }

        int handshake(){
            uint32_t data;
            rp2040.fifo.push(0xAA);
            data = rp2040.fifo.pop();
            if (data != 0xAB)
            {
                rp2040.fifo.push(data);
                errorflag*3;
                return 1;
            }
            rp2040.fifo.push(0xCD);
            rp2040.fifo.pop();
            navpacket handshakepacket;
            handshakepacket.r.errorflag = 1;

            return 0;
        }

        int initi2c(){
            Wire1.setSCL(SCL);
            Wire1.setSDA(SDA);
            Wire1.setClock(10000);
            Wire1.begin();
            scani2c() ? errorflag*= 5 : errorflag *= 1;
            return 0;
        }

        uint32_t sensorinit(){
            int imustatus;
            imustatus = imu.init();
            imustatus == 1 ? errorflag *= 7 : errorflag *= 1;
            imustatus == 2 ? errorflag *= 11 : errorflag *= 1;
            baro.init() ? errorflag *= 13 : errorflag *= 1;
            mag.init() ? errorflag *= 17 : errorflag *= 1;
            return 0;
        }

        void getsensordata(){
            //imu.read();
            baro.readsensor();
            mag.read();


            _sysstate.r.magdata = mag.data;
            _sysstate.r.imudata = imu.data;
            _sysstate.r.barodata = baro.data;
            return;
        }



};

#endif // NAVCOREHEADER
