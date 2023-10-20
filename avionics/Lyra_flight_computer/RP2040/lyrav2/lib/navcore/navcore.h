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

//using Eigen::MatrixXd;
//using Eigen::Vector3d;


class NAVCORE{
    

    public:
    
        navpacket _sysstate;

        float alpha = 0.98;

        NAVCORE(){
            _sysstate.r.orientationquat = {1,0,0,0};
        };
        /*
        1 = no errors
        3 = failed handshake
        5 = i2c devices fail
        7 = accel init fail
        11 = gyro init fail
        13 = baro init fail
        17 = mag init fail
        19 = packet send fail
        negative = fatal error
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
                _sysstate.r.errorflag*3;
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
            scani2c(false) ? _sysstate.r.errorflag*= 5 : _sysstate.r.errorflag *= 1;
            return 0;
        }

        uint32_t sensorinit(){
            int imustatus;
            int barostatus;
            int magstatus;
            imustatus = imu.init();;
            imustatus == 1 ? _sysstate.r.errorflag *= 7 : _sysstate.r.errorflag *= 1;
            imustatus == 2 ? _sysstate.r.errorflag *= 11 : _sysstate.r.errorflag *= 1;
            barostatus = baro.init();
            barostatus ? _sysstate.r.errorflag *= 13 : _sysstate.r.errorflag *= 1;
            magstatus = mag.init();
            magstatus ? _sysstate.r.errorflag *= 17 : _sysstate.r.errorflag *= 1;

            if (magstatus != 0 || imustatus != 0 || barostatus != 0)
            {
                _sysstate.r.errorflag *= -1;
            }
            
            
            return 0;
        }

        void getsensordata(){
            imu.read();
            baro.readsensor();
            mag.read();


            _sysstate.r.magdata = mag.data;
            _sysstate.r.imudata = imu.data;
            _sysstate.r.barodata = baro.data;
            return;
        }

        void computeorientation(){
            double timestep = (micros() - prevtime.intergrateorientation)/1e6;
            Quaterniond orientationquat = quatstructtoeigen(_sysstate.r.orientationquat);
            Vector3d gyro = vectorfloatto3(_sysstate.r.imudata.gyro);
            Vector3d accel = vectorfloatto3(_sysstate.r.imudata.accel);
            Vector3d mag = vectorfloatto3(_sysstate.r.magdata.utesla);
            Vector3d orientationeuler = vectorfloatto3(_sysstate.r.orientationeuler);

            AngleAxisd aa(timestep*gyro.norm(), gyro.normalized());
            Quaterniond qdelta(aa);

            orientationquat = orientationquat * qdelta;


            prevtime.intergrateorientation = micros();

            Quaterniond accelquat;

            accelquat.x() = accel.x();
            accelquat.y() = accel.y();
            accelquat.z() = accel.z();

            accelquat = orientationquat * accelquat * orientationquat.inverse();

            accel.x() = accelquat.x();
            accel.y() = accelquat.y();
            accel.z() = accelquat.z();
            

            Vector3d accelnorm(accel.normalized());

            float phi = acos(accelnorm.y());

            Vector3d naxis(accelnorm.cross(Vector3d(0,1,0)));

            naxis = naxis.normalized();


            Quaterniond accelrotquat(AngleAxisd((1-alpha)*phi,naxis));

            orientationquat = accelrotquat * orientationquat;


            orientationeuler = orientationquat.toRotationMatrix().eulerAngles(0,1,2);
            
            _sysstate.r.orientationquat = eigentoquatstruct(orientationquat);
            _sysstate.r.orientationeuler = vector3tofloat(orientationeuler);

        };



};

#endif // NAVCOREHEADER