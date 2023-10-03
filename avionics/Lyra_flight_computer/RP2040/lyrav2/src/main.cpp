#include <Arduino.h>
#include "generallib.h"
#include "navcore.h"
#include "mpcore.h"

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
    MP.radioinit();
    

    MP.errorflag == 1 ? MP.setled(GREEN) : MP.setled(BLUE);
    
    Serial.print("MP boot complete error code: ");
    Serial.println(MP.errorflag);
    waitfornextfifo();
    MP.fetchnavdata();
    
    Serial.print("NAV boot complete, error code :");
    Serial.println(MP._sysstate.r.navsysstate.r.errorflag);
    MP._sysstate.r.uptime = millis();
    MP.logdata();
    //MP.readdata();

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
    MP.changestate();

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

    if (millis()- MP.prevtime.beep >= MP.intervals[MP._sysstate.r.state].beep)
    {
        switch (MP._sysstate.r.state)
        {
        case 0:
            MP.beep(600);
            break;

        case 1: // ready to launch
            MP.beep(4000);
            break;

        case 2: // powered ascent
            MP.beep(6000);
            break;

        case 3: // unpowered descent
            MP.beep(6000);
            break;

        case 4: // under canopy
            MP.beep(6000);
            break;

        case 5: // landed
            MP.beep(4000);
            break;
        
        default:
            break;
        }
        MP.prevtime.beep = millis();
    }
    
    
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

    if (millis() - MP.prevtime.logdata >= MP.intervals[MP._sysstate.r.state].logdata)
    {
        MP.logdata();
        MP.prevtime.logdata = millis();
    }
    
    
    MP._sysstate.r.uptime = millis();
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

