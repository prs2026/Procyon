#include <Arduino.h>
#include <quats.h>
#include <generallib.h>
#include <Lyrav2sensors.h>

MPCORE MP;
NAVCORE NAV;




void setup() { // main core setup
    MP.setuppins();
    MP.beep();
    MP.setled(BLUE);
    MP.serialinit();

    MP.handshake();

    MP.setled(GREEN);

    MP.beep();
}

void setup1() { // nav core setup
    NAV.handshake();
}

void loop() { // main core loop

}


void loop1() { // nav core loop

}

