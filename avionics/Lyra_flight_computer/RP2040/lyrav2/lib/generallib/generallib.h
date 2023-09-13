#if !defined(GENERALLIB)
#define GENERALLIB
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "Lyrav2sensors.h"
#include "macros.h"
#include "SPI.h"
#include "SD.h"
#include <string.h>

File logfile;

IMU imu;
BARO baro;
MAG mag;

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
        Vector3int32 pos;
        Vector3int32 orientation;
        Vector3int32 vel;
        Vector3int32 acceleration;
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




int waitfornextfifo(int timeoutmillis){
    uint32_t pushmillis = millis();
    //int currentavalible = rp2040.fifo.available();
    while ((millis() - pushmillis) < 1 && rp2040.fifo.available() <= rp2040.fifo.available());
    if (rp2040.fifo.available() >= 1)
    {
        return 0;
    }
    return 1;
}

int waitfornextfifo(){
    while (rp2040.fifo.available() < 1);
    return 0;
}

uint32_t fifopop(){
    uint32_t data;
    rp2040.fifo.pop_nb(&data);
    return data;
}



class MPCORE{
    
    

    public:
        MPCORE(){};

        mpstate _sysstate;

        uint32_t errorflag = 1;
        /*
            1 = no error
            3 = handshake fail
            5 = serial init failure
            7 = sd init fail
            11 = flash init fail
            13 = no packet recived
            17 = bad telemetry packet
            19 = telemetry send fail
            23 = bad telemetry packet
        */
        bool sendserialon = false;


        struct timings{
            uint32_t logdata;
            uint32_t led;
            uint32_t serial;
        };
        timings intervals[7] = {
            {2000,1000,100}, // ground idle
            {100,200,100}, // launch detect
            {50,500,100}, // powered ascent
            {50,500,100}, // unpowered ascent
            {50,500,100}, // ballistic descent
            {50,800,100}, //ready to land
            {1000,1500,100} // landed
        };
        timings prevtime;
        bool ledstate = false;



        void setuppins(){
            pinMode(LEDRED,OUTPUT);
            pinMode(LEDGREEN,OUTPUT);
            pinMode(LEDBLUE,OUTPUT);
            pinMode(BUZZERPIN,OUTPUT);

            digitalWrite(LEDRED, LOW);
            digitalWrite(LEDGREEN, HIGH);
            digitalWrite(LEDBLUE, HIGH);
            return;
        }


        void beep(){
            tone(BUZZERPIN,2000,200);
            return;
        }

        void beep(int freq){
            tone(BUZZERPIN,freq,200);
            return;
        }

        void beep(int freq, unsigned int duration){
            tone(BUZZERPIN,freq,duration);
        return;
        }


        void setled(int color){
            switch (color)
            {
            case OFF:
                digitalWrite(LEDRED, HIGH);
                digitalWrite(LEDGREEN, HIGH);
                digitalWrite(LEDBLUE, HIGH);
                break;

            case RED:
                digitalWrite(LEDRED, LOW);
                digitalWrite(LEDGREEN, HIGH);
                digitalWrite(LEDBLUE, HIGH);
                break;
            
            case GREEN:
                digitalWrite(LEDRED, HIGH);
                digitalWrite(LEDGREEN, LOW);
                digitalWrite(LEDBLUE, HIGH);
                break;

            case BLUE:
                digitalWrite(LEDRED, HIGH);
                digitalWrite(LEDGREEN, HIGH);
                digitalWrite(LEDBLUE, LOW);
                break;
            
            default:
                break;
            }
        }



        int fetchnavdata(){
            navpacket recivedpacket;
            if (rp2040.fifo.available() <= 0)
            {
                return 1;
            }
            if (rp2040.fifo.pop() != 0xAB)
            {
                return 1;
            }
            
            for (int i = 0; i < sizeof(recivedpacket.data)/sizeof(recivedpacket.data[0]); i++)
            {
                recivedpacket.data[i] = rp2040.fifo.pop();
            }
            

            _sysstate.r.navsysstate = recivedpacket;

            while (rp2040.fifo.available() > 0)
            {
                uint32_t buf;
                rp2040.fifo.pop_nb(&buf);
            }
            
            
            return 0;
        }

        int handshake(){
            uint32_t data;
            int connectiontries = 0;
            while (connectiontries <= 3)
            {
            
                if (waitfornextfifo(1000) == 1)
                {
                    Serial.println("NAV core timeout");
                    rp2040.restartCore1();
                    connectiontries++;
                    break;
                }

                rp2040.fifo.pop_nb(&data);
                if (!rp2040.fifo.push_nb(0xAB))
                {
                    Serial.println("unable to push to fifo");
                    break;
                }
                
                waitfornextfifo(500);
                rp2040.fifo.pop_nb(&data);
                if(data != 0xCD)
                {
                    Serial.print("NAV Handshake Failed: expected 0xCD, got 0x");
                    Serial.println(data,HEX);
                    rp2040.restartCore1();
                    connectiontries++;
                    break;
                }
                rp2040.fifo.push_nb(0xEF);
                Serial.println("NAV Handshake complete");
                return 0;
            }
            Serial.println("NAV Handshake failed");
            errorflag*= 3;
            return 1;
        }

        void serialinit(){
            Serial.begin(115200);
            //Serial1.begin(115200);
            uint32_t serialstarttime = millis();
            while (!Serial && millis() - serialstarttime < 5000);
            delay(100);
            

            Serial.println("\n\nMP Serial init");
            //Serial1.println("\n\nMP Serial init");
            
            return;
        }

        int initsd(){
            if (!SD.begin(CS_SD))
            {
                Serial.println("SD init failure, card not present or not working");
                errorflag*=7;
                return 1;
            }
            Serial.println("SD card init succeess");
            return 0;
        }

        int senddatatoserial(){
            Serial.printf(">MP uptime %d \n",_sysstate.r.uptime);
            Serial.printf(">NAV uptime %d \n",_sysstate.r.navsysstate.r.uptime);

            Serial.printf(">MP errorflag %d \n", _sysstate.r.errorflag);
            Serial.printf(">NAV errorflag %d \n", _sysstate.r.navsysstate.r.errorflag);

            Serial.printf(">accel x: %f \n", float(_sysstate.r.navsysstate.r.imudata.accel.x)/10000);
            Serial.printf(">accel y: %f \n", float(_sysstate.r.navsysstate.r.imudata.accel.y)/10000);
            Serial.printf(">accel z: %f \n", float(_sysstate.r.navsysstate.r.imudata.accel.z)/10000);

            Serial.printf(">gyro x: %f \n", float(_sysstate.r.navsysstate.r.imudata.gyro.x)/10000);
            Serial.printf(">gyro y: %f \n", float(_sysstate.r.navsysstate.r.imudata.gyro.y)/10000);
            Serial.printf(">gyro z: %f \n", float(_sysstate.r.navsysstate.r.imudata.gyro.z)/10000);

            Serial.printf(">altitude: %f \n", float(_sysstate.r.navsysstate.r.barodata.altitude)/10000);

            Serial.printf(">mag x: %f \n",float(_sysstate.r.navsysstate.r.magdata.utesla.x)/10000);
            Serial.printf(">mag y: %f \n",float(_sysstate.r.navsysstate.r.magdata.utesla.y)/10000);
            Serial.printf(">mag z: %f \n",float(_sysstate.r.navsysstate.r.magdata.utesla.z)/10000);

            Serial.printf(">magraw x: %f \n",float(_sysstate.r.navsysstate.r.magdata.gauss.x)/10000);
            Serial.printf(">magraw y: %f \n",float(_sysstate.r.navsysstate.r.magdata.gauss.y)/10000);
            Serial.printf(">magraw z: %f \n",float(_sysstate.r.navsysstate.r.magdata.gauss.z)/10000);

            return 0;
        }

        int parsecommand(char input){
            Serial.println(input);
            if (int(input) == 115)
            {
                Serial.println("printing data to teleplot");
                sendserialon = !sendserialon;
            }
            return 0;
        }

        
};


//**************************************************************************************\\





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
            imu.readIMU();
            baro.readsensor();
            mag.read();


            _sysstate.r.magdata = mag.data;
            _sysstate.r.imudata = imu.data;
            _sysstate.r.barodata = baro.data;
        }

        void computeorientation(){
            
        }


};

#endif // GENERALLIB