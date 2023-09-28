
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

    MP.flashinit();
    MP.initsd();

    MP.errorflag == 1 ? MP.setled(GREEN) : MP.setled(BLUE);
    
    Serial.print("MP boot complete error code: ");
    Serial.println(MP.errorflag);
    waitfornextfifo();
    MP.fetchnavdata();
    
    Serial.print("NAV boot complete, error code :");
    Serial.println(MP._sysstate.r.navsysstate.r.errorflag);

    //MP.beep();
}

void setup1() { // nav core setup
    NAV.handshake();
    delay(200);
    NAV.initi2c();
    NAV.sensorinit();
    navpacket initpacket;
    initpacket.r.errorflag = NAV.errorflag;
    NAV.sendpacket(initpacket);

}

void loop() { // main core loop

    if (rp2040.fifo.available())
    {
        int _avalible = rp2040.fifo.available();
        int _error = MP.fetchnavdata();
        //Serial.printf("recived packet at timestamp : %d with error %d and %d bytes in the fifo",MP._sysstate.r.uptime,_error,_avalible);
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
        char buf = Serial.read(); 
        int i;
        Serial.printf("echo: %c dec: %d \n",buf,buf);
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
    NAV.computeorientation();
    if ((millis() - NAV.prevtime.sendpacket) >= NAV.intervals[NAV._sysstate.r.state].sendpacket)
    {
        int inbuf = rp2040.fifo.available();
        int error = NAV.sendpacket(NAV._sysstate);
        if (error > 0)
        {
            //Serial.printf("packet send failure at iteration %d and timestamp %d with %d bytes already in fifo \n"
            //,error,NAV._sysstate.r.uptime,inbuf);
        }
        else
        {
            //Serial.print("packet sent on iteration ");
            //Serial.print(error);
        }
        
        NAV.prevtime.sendpacket = millis();
        
    }
    
    NAV._sysstate.r.uptime = millis();
}

