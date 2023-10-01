#if !defined(MPCOREHEADER)
#define MPCOREHEADER

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


fs::File logtofile;


Sd2Card card;


class MPCORE{
    

    public:
        MPCORE(){

        };

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
        bool sendtoteleplot = true;


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
            pinMode(CS_SD,OUTPUT);

            digitalWrite(CS_SD,HIGH);
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
            SPI.setRX(SPI0_MISO);
            SPI.setTX(SPI0_MOSI);
            SPI.setSCK(SPI0_SCLK);
            SPI.begin();
            
            int loopbackbyte = SPI.transfer(0xEF);
            if (loopbackbyte != 0xEF)
            {
                Serial.printf("\nloopback failed, expected 239 got: %d \n",loopbackbyte);
            }
            Serial.printf("loopback sucessed, expected 239 got %d \n",loopbackbyte);
            
            SPI.end();




            if (!card.init(SPI_HALF_SPEED,CS_SD))
            {
                Serial.printf("SD card not present or not working, code: %d \n",card.errorCode());
            }

            if (!SD.begin(CS_SD))
            {
                Serial.println("SD init failure, card not present or not working");
                errorflag*=7;
                return 1;
            }
            /*
            logfile = SD.open("test.txt",FILE_WRITE);

            if (!logfile)
            {
                Serial.println("SD init fail, cant open file to log to");
                return 1;
            }
            logfile.println("lyrav2 be workin");
            logfile.close();
            
            */
            Serial.println("SD card init succeess");
            return 0;
        }

        int flashinit(){
                Serial.println("flash init start");

                rp2040.fifo.idleOtherCore();
                delay(200);

                int error = LittleFS.begin();

                if (error != 0)
                {
                    Serial.printf("filesystem mount fail %d\n",error);
                    errorflag *= 11;
                    rp2040.resumeOtherCore();
                    return 1;
                }

                error = LittleFS.format();

                if (error != 0)
                {
                    Serial.printf("filesystem format fail %d\n", error);
                    errorflag *= 11;
                    rp2040.resumeOtherCore();
                    return 1;
                }

                fs::File testfile = LittleFS.open("/init.txt","w+");

                if (!testfile)
                {
                    Serial.println("file open failed");
                    errorflag *= 11;
                    rp2040.resumeOtherCore();
                    return 2;
                }
                char teststr[] = "test";

                testfile.print(teststr);

                char readstr[6];

                testfile.readBytes(readstr,sizeof(teststr));

                if (!strcmp(teststr,readstr))
                {
                    Serial.print("r/w mismatch, expected:");
                    Serial.print(teststr);
                    Serial.print(" got :");
                    Serial.println(readstr);
                }
                
                Serial.print("r/w complete, expected:");
                Serial.print(teststr);
                Serial.print(" got :");
                Serial.println(readstr);

                
                testfile.close();
                Serial.println("flash init complete");
                rp2040.resumeOtherCore();
                return 0;
        }

        int senddatatoserial(){
            if (sendtoteleplot)
            {
                Serial.printf(
                ">MP uptime: %d \n" 
                ">NAV uptime: %d \n" 
                ">MP errorflag %d \n" 
                ">NAV errorflag %d \n" 
                ">accel x: %f \n" 
                ">accel y: %f \n"
                ">accel z: %f \n"  
                ">gyro x: %f \n" 
                ">gyro y: %f \n"
                ">gyro z: %f \n"
                ">altitude: %f \n" 
                ">verticalvel: %f \n"
                ">mag x: %f \n" 
                ">mag y: %f \n" 
                ">mag z: %f \n"
                ">magraw x: %f \n"
                ">magraw y: %f \n"
                ">magraw z: %f \n"
                ">orientationeuler x: %f \n"
                ">orientationeuler y: %f \n"
                ">orientationeuler z: %f \n",
                _sysstate.r.uptime
                ,_sysstate.r.navsysstate.r.uptime

                , _sysstate.r.errorflag
                , _sysstate.r.navsysstate.r.errorflag

                ,_sysstate.r.navsysstate.r.imudata.accel.x
                ,_sysstate.r.navsysstate.r.imudata.accel.y
                ,_sysstate.r.navsysstate.r.imudata.accel.z

                ,_sysstate.r.navsysstate.r.imudata.gyro.x*(180/M_PI)
                ,_sysstate.r.navsysstate.r.imudata.gyro.y*(180/M_PI)
                ,_sysstate.r.navsysstate.r.imudata.gyro.z*(180/M_PI)

                , _sysstate.r.navsysstate.r.barodata.altitude
                , _sysstate.r.navsysstate.r.barodata.verticalvel

                ,_sysstate.r.navsysstate.r.magdata.utesla.x
                ,_sysstate.r.navsysstate.r.magdata.utesla.y
                ,_sysstate.r.navsysstate.r.magdata.utesla.z

                ,_sysstate.r.navsysstate.r.magdata.gauss.x
                ,_sysstate.r.navsysstate.r.magdata.gauss.y
                ,_sysstate.r.navsysstate.r.magdata.gauss.z

                ,_sysstate.r.navsysstate.r.orientationeuler.x*(180/M_PI)
                ,_sysstate.r.navsysstate.r.orientationeuler.y*(180/M_PI)
                ,_sysstate.r.navsysstate.r.orientationeuler.z*(180/M_PI)
                 );
                 // this is ugly, but better than a million seperate prints
                return 0;
            }
            else
            {
                //Serial.printf("%f,%f,%f \n",_sysstate.r.navsysstate.r.imudata.accel.x,_sysstate.r.navsysstate.r.imudata.accel.y,_sysstate.r.navsysstate.r.imudata.accel.z);
            }
            
            

            return 0;
        }

        int parsecommand(char input){
            Serial.println(input);
            if (int(input) == 115)
            {
                Serial.println("printing data to teleplot");
                sendserialon = !sendserialon;
                sendtoteleplot = true;
            }
            else if (int(input) == 113)
            {
                Serial.println("sending accel data to magneto");
                sendserialon = !sendserialon;
                sendtoteleplot = false;

            }
            
            return 0;
        }

        
};

#endif // MPCOREHEADER
