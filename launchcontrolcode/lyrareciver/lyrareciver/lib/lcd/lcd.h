#if !defined(LCDLIB3)
#define LCDLIB3
// core libs
#include <Arduino.h>
#include <macros.h>

Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, LCDCS, LCDDC, LCDRST);


class LCDDISPLAY
{
private:
    /* data */
public:
    LCDDISPLAY(/* args */);
    bool init();
};

LCDDISPLAY::LCDDISPLAY(/* args */)
{
}

bool LCDDISPLAY::init(){
    Serial.println("init display");
    tft.init(240,320);
    Serial.println("init complete?");

    tft.fillScreen(ST77XX_BLACK);
    return 1;
}


#endif // LCDLIB