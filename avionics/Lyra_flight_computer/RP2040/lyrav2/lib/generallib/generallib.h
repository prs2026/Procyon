#if !defined(GENERALLIB)
#define GENERALLIB
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "macros.h"
#include "SPI.h"
#include "SD.h"
#include <string.h>
#include "LittleFS.h"


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

Vector3float vector3tofloat(Eigen::Vector3d v){
    Vector3float result;
    result.x = v.x();
    result.y = v.y();
    result.z = v.z();
    return result;
}

Eigen::Vector3d vectorfloatto3(Vector3float v){
    Eigen::Vector3d result;
    result.x() = v.x;
    result.y() = v.y;
    result.z() = v.z;
    return result;
}

Quaterniond quatstructtoeigen(Quatstruct q){
    Quaterniond result;
    result.w() = q.w;
    result.x() = q.x;
    result.y() = q.y;
    result.z() = q.z;
    return result;
}

Quatstruct eigentoquatstruct(Quaterniond q){
    Quatstruct result;
    result.w = q.w();
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();
    return result;
}

int radiocommcheck(){
    uint8_t initpayload[32] = {0xAB,0xCD};
    uint8_t exppayload = 0xCD;
    bool error;
    radio.stopListening();
    error = radio.write(&initpayload,sizeof(initpayload));   
    radio.startListening(); 
    
    if (!error)
    {
        //Serial.println("other radio didnt ack/couldnt send");
        return 2;
    }
    

    uint32_t timeoutstart = millis();
    while (!radio.available()){
        if (millis() - timeoutstart > 200){
            //Serial.println("radio commcheck timeout");
            return 1;
        }
    }
    
    uint8_t buf[32];

    radio.read(&buf,sizeof(buf));

    if (buf[0] != exppayload)
    {
        Serial.printf("radio commcheck fail, expected %x, got %x\n",exppayload,buf);
        return 1;
    }
    Serial.println("radio commcheck success");
    return 0;
    }





#endif // GENERALLIB