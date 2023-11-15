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



//fs::File logtofile;


Sd2Card card;


class MPCORE{
    

    public:
        

        mpstate _sysstate;
        int detectiontime = 0;
        uint32_t landedtime = 0;
        bool datamoved = false;
        uint32_t landingdetectiontime = 0;



        MPCORE(){
            _sysstate.r.state = 0;
            _sysstate.r.checksum1 = 0xAB;
            _sysstate.r.checksum2 = 0xCD;
        };

        int32_t errorflag = 1;
        /*
            1 = no error
            3 = handshake fail
            5 = serial init failure
            7 = sd init fail
            11 = flash init fail
            13 = no packet recived
            17 = bad telemetry packet
            19 = radio init fail
            23 = bad telemetry packet
        */
        bool sendserialon = false;
        bool sendtoteleplot = true;

        int ledcolor;


        struct timings{
            uint32_t logdata;
            uint32_t led;
            uint32_t serial;
            uint32_t sendtelemetry;
            uint32_t beep;
            uint32_t detectstatechange;
            uint32_t loop;
        };
        timings intervals[7] = {
            {2000,1000,50,200,30000,10}, // ground idle
            {10,200,100, 200, 500,10}, // launch detect
            {10,500,100, 200, 1000,10}, // powered ascent
            {10,500,100,200, 1000,10}, // unpowered ascent
            {10,500,100,200, 1000,10}, // ballistic descent
            {10,800,100,200, 1000,10}, //ready to land
            {1000,1500,100,200, 1000,10} // landed
        };
        timings prevtime;
        bool ledstate = false;



        void setuppins(){
            pinMode(LEDRED,OUTPUT);
            pinMode(LEDGREEN,OUTPUT);
            pinMode(LEDBLUE,OUTPUT);
            pinMode(BUZZERPIN,OUTPUT);
            pinMode(CS_SD,OUTPUT);
            pinMode(BRK_CS,OUTPUT);

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

        int initsd(){
            SPI.setRX(SPI0_MISO);
            SPI.setTX(SPI0_MOSI);
            SPI.setSCK(SPI0_SCLK);
            SPI.begin();
            
            // int loopbackbyte = SPI.transfer(0xEF);
            // if (loopbackbyte != 0xEF)
            // {
            //     Serial.printf("\nloopback failed, expected 239 got: %d \n",loopbackbyte);
            // }
            // Serial.printf("loopback sucessed, expected 239 got %d \n",loopbackbyte);
            
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
            
            SDLib::File logfile = SD.open("test.txt",FILE_WRITE);

            if (!logfile)
            {
                Serial.println("SD init fail, cant open file to log to");
                return 1;
            }
            logfile.println("lyrav2 be workin");
            logfile.close();
            
            
            Serial.println("SD card init succeess");
            return 0;
        }



        int logdata(){
            uint32_t openingtime = micros();
            fs::File logfile = LittleFS.open("/log.csv", "a+");
            //Serial.printf("opening file took %d \n",micros()-openingtime);
            openingtime = micros();
            if (!logfile){
                return 1;
                errorflag *= 11;
            };
            //Serial.printf("checking file took %d \n",micros()-openingtime);
            openingtime = micros();
            int j = 0;
            for (int i = 0; i < sizeof(mpstate); i++)
            {
                logfile.write(_sysstate.data8[j]);
                j++;
            }
        
            
            openingtime = micros();
            logfile.close();
            //Serial.printf("closing file took %d \n\n",micros()-openingtime);
            return 0;
        }

        int erasedata(){
           int error = LittleFS.remove("/log.csv");
           if (error != 1)
           {
                Serial.println("file erase fail");
                return 1;
           }
           Serial.println("file erase success");
           return 0;
           
        }

        int movedata(){
            Serial.println("moving data to sd");
            

            int fileunique = 1;
            char fileuniquestr[3];
            char fileend[] = ".csv";

            int error = 1;

            char newfilename[25] = "/log";



            for (int i = 0; i < 400; i++)
            {  
                strcpy(newfilename, "/log");
                itoa(fileunique, fileuniquestr, 10);
                strcat(newfilename, fileuniquestr);
                strcat(newfilename, fileend);
                //Serial.print("checking if file exists ");
                //Serial.println(newfilename);
                int exists = SD.exists(newfilename);
                if (exists == 0)
                {
                    Serial.print("making new file with name ");
                    Serial.println(newfilename);
                    error = 0;
                    break;
                }
                fileunique++;
            }

            SDLib::File sdfile = SD.open(newfilename, FILE_WRITE);

            error = sdfile;
            
            if (error == 0)
            {
                Serial.print("unable to make file");
                Serial.println(newfilename);
                return 1;
            }
            
            fs::File readfile = LittleFS.open("/log.csv", "r");
            sdfile.println("checksum,uptime mp,uptime nav, errorflag mp, errorflag nav,accel x, accel y, accel z, accelworld x,accelworld y,accelworld z, gyro x, gyro y, gyro z, mag x, mag y, mag z, magraw x, magraw y, magraw z, euler x, euler y, euler z, quat w, quat x, quat y, quat z, altitude, pressure, verticalvel,filteredvvel,maxrecorded alt,altitudeagl,filteredalt,imutemp, barotemp, state,checksum2");
            
            Serial.printf("flash amount used: %d\n",readfile.size());

            while (readfile.available() > 0)
            {
                mpstate readentry;
                uint8_t buf[sizeof(mpstate)];
                readfile.read(buf,sizeof(mpstate));
                int j = 0;
                for (int i = 0; i < sizeof(mpstate); i++)
                {
                    readentry.data8[j] = buf[j];
                    j++;
                }
                if (readentry.r.checksum1 != 0xAB || readentry.r.checksum2 != 0xCD)
                {
                    uint32_t starttime = millis();
                    while (millis() - starttime < 1000)
                    {   
                        int thisbyte = readfile.read();
                        if (thisbyte == 0xAB)
                        {
                            Serial.println("found start of next entry");
                            readfile.seek(readfile.position() - 1);
                            break;
                        }
                        Serial.printf("waiting for start of next entry, exp 0xAB got %x \n", thisbyte);
                        
                    }
                    
                }
                
                sdfile.printf(
                    "101,"//checksum // checksum,uptime mp,uptime nav, errorflag mp, accel x, accel y, accel z, gyro x, gyro y, gyro z, mag x, mag y, mag z, magraw x, magraw y, magraw z, euler x, euler y, euler z, quat w, quat x, quat y, quat z, altitude, pressure, verticalvel, imutemp, barotemp, state,checksum
                    "%d,%d,"//uptimes
                    "%d,%d,"//errorflag
                    "%f,%f,%f," // accel
                    "%f,%f,%f," // gyro
                    "%f,%f,%f," // mag
                    "%f,%f,%f," // magraw
                    "%f,%f,%f," // orientation euler"
                    "%f,%f,%f,%f," // orientation quat"
                    "%f,%f,%f,%f,%f,%f,%f," //altitude, presusre, verticalvel,filtered vvel, altitudeagl, filtered alt
                    "%f,%f," // temps, imu baro mag
                    "%d,202\n", //state
                    readentry.r.uptime,
                    readentry.r.navsysstate.r.uptime,
                    readentry.r.errorflag,
                    readentry.r.navsysstate.r.errorflag,
                    readentry.r.navsysstate.r.imudata.accel.x,
                    readentry.r.navsysstate.r.imudata.accel.y,
                    readentry.r.navsysstate.r.imudata.accel.z,
                    readentry.r.navsysstate.r.accelworld.x,
                    readentry.r.navsysstate.r.accelworld.y,
                    readentry.r.navsysstate.r.accelworld.z,
                    readentry.r.navsysstate.r.imudata.gyro.x*(180/M_PI),
                    readentry.r.navsysstate.r.imudata.gyro.y*(180/M_PI),
                    readentry.r.navsysstate.r.imudata.gyro.z*(180/M_PI),
                    readentry.r.navsysstate.r.magdata.utesla.x,
                    readentry.r.navsysstate.r.magdata.utesla.y,
                    readentry.r.navsysstate.r.magdata.utesla.z,
                    readentry.r.navsysstate.r.magdata.gauss.x,
                    readentry.r.navsysstate.r.magdata.gauss.y,
                    readentry.r.navsysstate.r.magdata.gauss.z,
                    readentry.r.navsysstate.r.orientationeuler.x*(180/M_PI),
                    readentry.r.navsysstate.r.orientationeuler.y*(180/M_PI),
                    readentry.r.navsysstate.r.orientationeuler.z*(180/M_PI),
                    readentry.r.navsysstate.r.orientationquat.w,
                    readentry.r.navsysstate.r.orientationquat.x,
                    readentry.r.navsysstate.r.orientationquat.y,
                    readentry.r.navsysstate.r.orientationquat.z,
                    readentry.r.navsysstate.r.barodata.altitude,
                    readentry.r.navsysstate.r.barodata.pressure,
                    readentry.r.navsysstate.r.barodata.verticalvel,
                    readentry.r.navsysstate.r.filteredvvel,
                    readentry.r.navsysstate.r.barodata.maxrecordedalt,
                    readentry.r.navsysstate.r.barodata.altitudeagl,
                    readentry.r.navsysstate.r.filteredalt,
                    readentry.r.navsysstate.r.imudata.temp,
                    readentry.r.navsysstate.r.barodata.temp,
                    readentry.r.state
                    );

            }
            
            readfile.close();
            sdfile.close();
            erasedata();
            Serial.println("done moving data");
            return 0;
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
            errorflag *= -1;
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


        int flashinit(){
                Serial.println("flash init start");
                // LittleFSConfig cfg;
                // cfg.setAutoFormat(false);
                // LittleFS.setConfig(cfg);

                //rp2040.fifo.idleOtherCore();
                //delay(200);
                Serial.println("other core idle, trying to begin littlefs");
                int error = LittleFS.begin();

                if (error = 0)
                {
                    Serial.printf("filesystem mount fail %d\n",error);
                    errorflag *= 11;
                    rp2040.resumeOtherCore();
                    return 1;
                }
                //Serial.println("littlefs started!");

                // error = LittleFS.format();

                // if (error != 0)
                // {
                //     Serial.printf("filesystem format fail %d\n", error);
                //     errorflag *= 11;
                //     rp2040.resumeOtherCore();
                //     return 1;
                // }

                FSInfo64 *info;
                error = LittleFS.info64(*info);

                if (error != 1)
                {
                    Serial.printf("filesystem info fail %d\n", error);
                    errorflag *= 11;
                    rp2040.resumeOtherCore();
                    return 1;
                }
                

                uint32_t total = info->totalBytes;
                uint32_t used = info->usedBytes;
                uint32_t avail = total - used;

                Serial.printf("FS info: total %d, used %d, avail %d\n",total,used,avail);

                LittleFS.remove("/f.txt");

                fs::File testfile = LittleFS.open("/f.txt","w+");

                if (!testfile)
                {
                    Serial.println("file open failed");
                    errorflag *= 11;
                    rp2040.resumeOtherCore();
                    return 2;
                }

                //Serial.println("file opened");
                int testnum = 1;

                testfile.print(testnum);
                //Serial.print("file written");
                testfile.close();
                testfile = LittleFS.open("/f.txt","r");

                int readnum = testfile.read() - 48;

                //Serial.println("file read");

                if (readnum != testnum)
                {
                    Serial.printf("read fail, expected %d, got %d\n",testnum,readnum);
                    errorflag *= 11;
                    rp2040.resumeOtherCore();
                    return 3;
                }
                
                //Serial.printf("read success, expected %d, got %d\n",testnum,readnum);

                
                testfile.close();
                Serial.println("flash init complete");
                rp2040.resumeOtherCore();
                return 0;
        }


        int radioinit(){
            SPI.end();
            SPI.setRX(SPI0_MISO);
            SPI.setTX(SPI0_MOSI);
            SPI.setSCK(SPI0_SCLK);
            SPI.begin();
            Serial.println("radio init start");

            int error = radio.begin(&SPI);
            if (!error)
            {
                Serial.println("radio init fail");
                error = radio.isChipConnected();

                if (!error)
                {
                    Serial.println("radio not connected");
                    errorflag *= 19;
                    return 1;
                }

            }
            Serial.println("radio init success");

    

            //radio.setPALevel(RF24_PA_MAX);
            //radio.setAutoAck(true);
            //radio.setRetries(10,15);
            //radio.setDataRate(RF24_250KBPS);

            radio.openWritingPipe(radioaddress[1]);
            radio.openReadingPipe(1,radioaddress[0]);

            uint32_t radiostarttime = millis();

            bool sucess = false;

            while ((millis() - radiostarttime) < 2000)
            {
                if (radiocommcheck() == 0)
                {
                    sucess == true;
                    break;
                }
                //Serial.println("radio handshake fail");
            }
            if (sucess)
            {
               
                Serial.println("radio handshake timeout");
                errorflag *= 19;
                return 1;
            }
            Serial.println("radio handshake complete");
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
                ">accelworld x: %f \n" 
                ">accelworld y: %f \n"
                ">accelworld z: %f \n"  
                ">gyro x: %f \n" 
                ">gyro y: %f \n"
                ">gyro z: %f \n"
                ">altitude: %f \n" 
                ">verticalvel: %f \n"
                ">filtered vvel: %f \n"
                ">mag x: %f \n" 
                ">mag y: %f \n" 
                ">mag z: %f \n"
                // ">magraw x: %f \n"
                // ">magraw y: %f \n"
                // ">magraw z: %f \n"
                ">orientation pitch: %f \n"
                ">orientation yaw: %f \n"
                ">orientation roll: %f \n"

                ">orientation w: %f \n"
                ">orientation x: %f \n"
                ">orientation y: %f \n"
                ">orientation z:s %f \n"

                ">maxrecorded alt: %f \n"
                ">filtered alt: %f \n"
                ">state : %d \n"
                ">altitudeagl : %f \n"
                ">varience alt : %f \n"
                ">varience vvel : %f \n"
                ">baro temp : %f \n",
                _sysstate.r.uptime
                ,_sysstate.r.navsysstate.r.uptime

                , _sysstate.r.errorflag
                , _sysstate.r.navsysstate.r.errorflag

                ,_sysstate.r.navsysstate.r.imudata.accel.x
                ,_sysstate.r.navsysstate.r.imudata.accel.y
                ,_sysstate.r.navsysstate.r.imudata.accel.z

                ,_sysstate.r.navsysstate.r.accelworld.x
                ,_sysstate.r.navsysstate.r.accelworld.y
                ,_sysstate.r.navsysstate.r.accelworld.z

                ,_sysstate.r.navsysstate.r.imudata.gyro.x*(180/M_PI)
                ,_sysstate.r.navsysstate.r.imudata.gyro.y*(180/M_PI)
                ,_sysstate.r.navsysstate.r.imudata.gyro.z*(180/M_PI)

                , _sysstate.r.navsysstate.r.barodata.altitude
                , _sysstate.r.navsysstate.r.barodata.verticalvel
                , _sysstate.r.navsysstate.r.filteredvvel

                ,_sysstate.r.navsysstate.r.magdata.utesla.x
                ,_sysstate.r.navsysstate.r.magdata.utesla.y
                ,_sysstate.r.navsysstate.r.magdata.utesla.z

                // ,_sysstate.r.navsysstate.r.magdata.gauss.x
                // ,_sysstate.r.navsysstate.r.magdata.gauss.y
                // ,_sysstate.r.navsysstate.r.magdata.gauss.z

                ,_sysstate.r.navsysstate.r.orientationeuler.x*(180/M_PI)
                ,_sysstate.r.navsysstate.r.orientationeuler.y*(180/M_PI)
                ,_sysstate.r.navsysstate.r.orientationeuler.z*(180/M_PI)

                ,_sysstate.r.navsysstate.r.orientationquat.w
                ,_sysstate.r.navsysstate.r.orientationquat.x
                ,_sysstate.r.navsysstate.r.orientationquat.y
                ,_sysstate.r.navsysstate.r.orientationquat.z

                , _sysstate.r.navsysstate.r.barodata.maxrecordedalt
                , _sysstate.r.navsysstate.r.filteredalt
                , _sysstate.r.state
                , _sysstate.r.navsysstate.r.barodata.altitudeagl
                , _sysstate.r.navsysstate.r.confidence.alt
                , _sysstate.r.navsysstate.r.confidence.vvel
                , _sysstate.r.navsysstate.r.barodata.temp
                 );
                 // this is ugly, but better than a million seperate prints
                return 0;
            }
            else
            {
                Serial.printf("%f,%f,%f \n",_sysstate.r.navsysstate.r.orientationeuler.x,_sysstate.r.navsysstate.r.orientationeuler.y,_sysstate.r.navsysstate.r.orientationeuler.z);
            }
            
            

            return 0;
        }

        int changestate(){

            Vector3d accelvec = vectorfloatto3(_sysstate.r.navsysstate.r.imudata.accel);
            Vector3d gyrovec  = vectorfloatto3(_sysstate.r.navsysstate.r.imudata.gyro);
            if (_sysstate.r.state == 1) // detect liftoff
            {
                
                float accelmag = accelvec.norm();
                accelmag > 20 ? detectiontime = detectiontime : detectiontime = millis();
                if (millis() - detectiontime >= 50)
                {
                    _sysstate.r.state = 2;
                    detectiontime = millis();
                }
                
            }
            else if (_sysstate.r.state == 2) // detect burnout
            {
                
                float accelmag = accelvec.norm();
                accelvec.z() < 2 ? detectiontime = detectiontime : detectiontime = millis();
                if (millis() - detectiontime >= 200)
                {
                    _sysstate.r.state = 3;
                    detectiontime = millis();
                }
            }

            else if (_sysstate.r.state == 3) // detect appogee
            {
                _sysstate.r.navsysstate.r.barodata.altitudeagl < _sysstate.r.navsysstate.r.barodata.maxrecordedalt*0.95 ?  detectiontime = detectiontime : detectiontime = millis();

                if (millis() - detectiontime >= 100)
                {
                    _sysstate.r.state = 4;
                    detectiontime = millis();
                }
            }

            else if (_sysstate.r.state == 4) // detect chute opening
            {
                
                float accelmag = accelvec.norm();
                accelmag > 7 ? detectiontime = detectiontime : detectiontime = millis();
                if (millis() - detectiontime >= 300)
                {
                    _sysstate.r.state = 5;
                    detectiontime = millis();
                }
            }

            else if (_sysstate.r.state == 5) // detect landing
            {   

                

                if (abs(_sysstate.r.navsysstate.r.barodata.verticalvel) < 0.3 && accelvec.norm() < 20 &&  accelvec.norm() > 5  && gyrovec.norm() < 0.5)
                {
                     detectiontime = detectiontime;
                }
                else{
                    detectiontime = millis();
                }

                if (millis() - detectiontime >= 500)
                {
                    _sysstate.r.state = 6;
                    detectiontime = millis();
                    landedtime = millis();
                }
            }
            

            if (_sysstate.r.state > 1 && abs(_sysstate.r.navsysstate.r.barodata.verticalvel) < 0.3 && accelvec.norm() < 20 &&  accelvec.norm() > 5  && gyrovec.norm() < 0.5)
            {
                landingdetectiontime = landingdetectiontime;
            }
            else{
                landingdetectiontime = millis();
            }
            if (millis() - landingdetectiontime >= 1000)
                {
                    _sysstate.r.state = 6;
                    detectiontime = millis();
                    landedtime = millis();
            }
            

            

            

            return 0;
        }

        int parsecommand(char input){
            if (int(input) == 0)
            {
                return 1;
            }
            Serial.println(int(input));
            
            

            if (input == 'l' && _sysstate.r.state < 2){
                _sysstate.r.state = 1;
                return 0;
            }

            else if (input == 'a' && (_sysstate.r.state < 3 || _sysstate.r.state >= 6 )){
                _sysstate.r.state = 0;
                return 0;
            }

            switch (input)
            {
            case 's':
                Serial.println("printing data to teleplot");
                sendserialon = !sendserialon;
                sendtoteleplot = true;
                break;

            case 'w':
                Serial.println("printing data to processing");
                sendserialon = !sendserialon;
                sendtoteleplot = false;
                break;


            case 'e':
                erasedata();
                break;

            case 'D':
                logdata();
                break;
            
            case 'm':
                movedata();
                break;
            
            case 'o':
                baro.getpadoffset();
                break;
            
            default:
                break;
            }
            
            
            return 0;
        }

        int sendtelemetry(){
            telepacket packettosend;
            uint8_t databufs[32];
            

            packettosend = statetopacket(_sysstate);
            int j = 0;
            for (int i = 0; i < sizeof(databufs); i++)
            {
                databufs[j] = packettosend.data[j];
                j++;
            }
            radio.stopListening();
            bool error = radio.write(&databufs,sizeof(databufs),true);
            //radio.txStandBy(100);
            
            radio.startListening();
            // if (!error)
            // {
            //     //Serial.printf("telemetry send fail \n");
            //     return 1;
            // }

            // for (int i = 0; i < sizeof(databufs); i++)
            // {
            //   Serial.print(databufs[i],HEX);
            //   Serial.print(" ");
            // }
            // Serial.println("eom");
            
            // Serial.printf("%f,%f,%f"//accel
            // ",%f,%f,%f" // gyro
            // ",%f,%f" // alt, vvel
            // ",%f,%f,%f" // orientation
            // ",%d,%d,%d \n", // uptime, errorflagmp, errorflagnav, dataage, selfuptime
            // float(packettosend.r.accel.x)/100,float(packettosend.r.accel.y)/100,float(packettosend.r.accel.z)/100,
            // float(packettosend.r.gyro.x)/100,float(packettosend.r.gyro.y)/100,float(packettosend.r.gyro.z)/100,
            // float(packettosend.r.altitude)/100,float(packettosend.r.verticalvel)/100,
            // float(packettosend.r.orientationeuler.x)/100,float(packettosend.r.orientationeuler.y)/100,float(packettosend.r.orientationeuler.z)/100,
            // packettosend.r.uptime,packettosend.r.errorflagmp,packettosend.r.errorflagnav);

            
            return 0;
            
        }

};

#endif // MPCOREHEADER