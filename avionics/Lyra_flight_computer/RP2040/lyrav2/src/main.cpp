
#include <Arduino.h>
#include <quats.h>
#include "generallib.h"

MPCORE MP;
NAVCORE NAV;



void setup() { // main core setup
    MP.setuppins();
    MP.beep();
    MP.setled(BLUE);
    MP.serialinit();

    MP.handshake();

    MP.initsd();

    MP.errorflag == 1 ? MP.setled(GREEN) : MP.setled(BLUE);
    
    Serial.print("MP boot complete error code: ");
    Serial.println(MP.errorflag);
    waitfornextfifo();
    navpacket initpacket = MP.fetchnavdata();
    
    Serial.print("NAV boot complete, error code :");
    Serial.println(initpacket.r.errorflag);

    MP.beep();
}

void setup1() { // nav core setup
    NAV.handshake();
    NAV.initi2c();
    NAV.sensorinit();
    navpacket initpacket;
    initpacket.r.errorflag = NAV.errorflag;
    NAV.sendpacket(initpacket);

}

void loop() { // main core loop
    MP.setled(OFF);
    delay(1000);
    MP.setled(GREEN);
    delay(1000);
}


void loop1() { // nav core loop

}

