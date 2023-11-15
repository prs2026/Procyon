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
    
    navpacket prevsysstate;
    uint32_t kfupdatetime;
    uint32_t kfpredicttime;

    public:
    
        navpacket _sysstate;
        

        float alpha = 0.05;

        void KFinit(){
            _sysstate.r.filteredalt = _sysstate.r.barodata.altitudeagl;
            _sysstate.r.filteredvvel = _sysstate.r.barodata.verticalvel;
           
            _sysstate.r.confidence.alt = 100;
            _sysstate.r.confidence.vvel = 100;
            prevsysstate = _sysstate;
            prevtime.kfpredict = micros();
            prevtime.kfupdate = micros();
        }

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
            uint32_t kfpredict;
            uint32_t kfupdate;
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
            scani2c(true) ? _sysstate.r.errorflag*= 5 : _sysstate.r.errorflag *= 1;
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

        Quatstruct quatfromaccel(Vector3float accelfloat, Vector3float magfloat){ 
            Vector3d accel = vectorfloatto3(accelfloat).normalized();
            Vector3d mag = vectorfloatto3(magfloat).normalized();
            Quaterniond rotquat;
            Matrix3d rotmatrix;
            Vector3d rot[3]; // [0] = down, [1] = east [2] = north
            rot[0] = -accel;
            rot[0] = rot[0].normalized();

            rot[1] = (rot[0].cross(mag));
            rot[1] = rot[1].normalized();

            rot[2] = rot[0].cross(rot[1]);
            rot[2] = rot[2].normalized();
            
            rotmatrix << rot[0].transpose() , rot[1].transpose(), rot[2].transpose();
            rotquat = rotmatrix;
            Quatstruct result = eigentoquatstruct(rotquat);
            return result;
        }

        void KFpredict(){
            navpacket extrapolatedsysstate = _sysstate;

            double timestep = (micros() - kfpredicttime)/1e6;

            extrapolatedsysstate.r.filteredalt = _sysstate.r.filteredalt + (timestep*_sysstate.r.filteredvvel); // extrapolate with velocity dynamics
            extrapolatedsysstate.r.filteredvvel = _sysstate.r.filteredvvel;// + (timestep*_sysstate.r.accelworld.z); 

            extrapolatedsysstate.r.confidence.alt = _sysstate.r.confidence.alt + (pow(timestep,2)*_sysstate.r.confidence.vvel) + ALTVAR; // extrapolate variences with velocity dynamics
            extrapolatedsysstate.r.confidence.vvel = _sysstate.r.confidence.vvel + VVELVAR;// +pow(timestep,2)*0.5 + 0.05;

            extrapolatedsysstate = computeorientation(extrapolatedsysstate);
            extrapolatedsysstate.r.accelworld = getworldaccel(extrapolatedsysstate);
            //Serial.printf(">extrap var: %f\n",extrapolatedsysstate.r.confidence.alt);

            kfpredicttime = micros();
            _sysstate = extrapolatedsysstate;
        }                                                         



        void KFupdate(){
            double timestep = (micros() - kfupdatetime)/1e6;

            variences kgain; // calc new kalman gain
            kgain.alt = /*0.2;*/  _sysstate.r.confidence.alt/(_sysstate.r.confidence.alt+ALTNOISE);
            kgain.vvel = /*0.5;*/ _sysstate.r.confidence.vvel/(_sysstate.r.confidence.vvel+VVELNOISE);
            //Serial.printf(">kalman gain: %f\n",kgain.alt);
            
            _sysstate.r.filteredalt = prevsysstate.r.filteredalt + kgain.alt*(_sysstate.r.barodata.altitudeagl - prevsysstate.r.filteredalt); // state update
            _sysstate.r.filteredvvel = prevsysstate.r.filteredvvel + kgain.vvel*(_sysstate.r.barodata.verticalvel - prevsysstate.r.filteredvvel);

            // _sysstate.r.orientationquat = quatfromaccel(_sysstate.r.imudata.accel,_sysstate.r.magdata.utesla);
            // Quaterniond rotquat = quatstructtoeigen(_sysstate.r.orientationquat);
            // _sysstate.r.orientationeuler = vector3tofloat(rotquat.toRotationMatrix().eulerAngles(0,1,2));

            _sysstate.r.confidence.alt = (1-kgain.alt)*prevsysstate.r.confidence.alt; // variences update
            _sysstate.r.confidence.vvel = (1-kgain.vvel)*prevsysstate.r.confidence.vvel;
            
            
            prevsysstate = _sysstate;
            prevsysstate.r.confidence = _sysstate.r.confidence;
            kfupdatetime = micros();
        }

        navpacket computeorientation(navpacket currentpacket){
            double timestep = (micros() - prevtime.intergrateorientation)/1e6;
            Quaterniond orientationquat = quatstructtoeigen(currentpacket.r.orientationquat);
            Vector3d gyro = vectorfloatto3(currentpacket.r.imudata.gyro);
            Vector3d accel = vectorfloatto3(currentpacket.r.imudata.accel);
            Vector3d mag = vectorfloatto3(currentpacket.r.magdata.utesla);
            Vector3d orientationeuler = vectorfloatto3(currentpacket.r.orientationeuler);

            AngleAxisd aa(timestep*gyro.norm(), gyro/gyro.norm());
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


            Quaterniond quatadj(0.707,0.707,0,0);

            Quaterniond orientationquatadj = quatadj*orientationquat;//*quatadj.conjugate();

            Matrix3d R = orientationquatadj.toRotationMatrix();

            orientationeuler = R.eulerAngles(0,1,2);

            

            currentpacket.r.orientationquatadj = eigentoquatstruct(orientationquatadj);
            currentpacket.r.orientationquat = eigentoquatstruct(orientationquat);
            currentpacket.r.orientationeuler = vector3tofloat(orientationeuler);
            
            return currentpacket;
        }

        Vector3float getworldaccel(navpacket _state){
            Vector3d accelvec = vectorfloatto3(_state.r.imudata.accel);
            Quaterniond orientationquat = quatstructtoeigen(_state.r.orientationquatadj);//.inverse();
            Quaterniond accelquat2;
            accelquat2.w() = 0;

            // Matrix3d R = orientationquat.toRotationMatrix();
            // Matrix3d Rtrans = R.inverse();

            accelquat2.x() = accelvec.x();
            accelquat2.y() = accelvec.y();
            accelquat2.z() = accelvec.z();

            accelquat2 = orientationquat.inverse() * accelquat2 * orientationquat;

            accelvec.x() = accelquat2.x();
            accelvec.y() = accelquat2.y();
            accelvec.z() = accelquat2.z();

            Vector3d grav(0,0,9.801);
            Vector3d _accelworld = accelvec-grav;

            return vector3tofloat(_accelworld);

        };

        

};

#endif // NAVCOREHEADER