
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
    MP.fetchnavdata();
    
    Serial.print("NAV boot complete, error code :");
    Serial.println(MP._sysstate.r.navsysstate.r.errorflag);

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

    if (!MP.fetchnavdata())
    {
        //Serial.print("recived packet at timestamp :");
        //Serial.println(millis());
    }
    
    if (millis()- MP.prevtime.led >= MP.intervals[MP._sysstate.r.state].led)
    {
        MP.ledstate ? MP.setled(GREEN) : MP.setled(OFF);
        MP.ledstate =! MP.ledstate;
        MP.prevtime.led = millis();
    }
    MP._sysstate.r.uptime = millis();
    
    if (Serial.available())
    {
        char buf[10]; 
        int i;
        Serial.readBytes(buf,10);
        Serial.print("echo: ");
        Serial.println(buf);
        MP.parsecommand(buf);
    }

    if (MP.sendserialon & millis() - MP.prevtime.serial >= MP.intervals[MP._sysstate.r.state].serial)
    {
        MP.senddatatoserial();
        MP.prevtime.serial = millis();
    }
    
    
}


void loop1() { // nav core loop
    NAV.getsensordata();
    if ((millis() - NAV.prevtime.sendpacket) >= NAV.intervals[NAV._sysstate.r.state].sendpacket)
    {
        int error = NAV.sendpacket(NAV._sysstate);
        if (error > 0)
        {
            //Serial.print("packet failed on iteration ");
            //Serial.print(error);
        }
        else
        {
            //Serial.print("packet sent on iteration ");
            //Serial.print(error);
        }
        
        NAV.prevtime.sendpacket = millis();
        //Serial.print(" at timestamp: ");
        //Serial.println(millis());
    }
    
    NAV._sysstate.r.uptime = millis();
}

