#if !defined(GENERALLIB)
#define GENERALLIB
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

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


#define I2C1_SDA 22
#define I2C1_SCL 23

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



class MPCORE{
    
    

    public:
        MPCORE(){};

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

        uint32_t handshake(){
            rp2040.fifo.push_nb();
            return;
        }

        void serialinit(){
            Serial.begin(115200);
            Serial1.begin(115200);
            while (!Serial && !Serial1);

            

            Serial.println("\n\nMP Serial init");
            Serial1.println("\n\nMP Serial init");
            
            return;
        }


};

class NAVCORE{
    
    

    public:
        NAVCORE(){};

        

        uint32_t handshake(){
            
            return;
        }



};

#endif // GENERALLIB