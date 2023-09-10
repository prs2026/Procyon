#if !defined(Lyrav2sensors)
#define Lyrav2sensors
#include <Arduino.h>
#include <Wire.h>
#include <BMI088.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LIS3MDL.h>


/* accel object */
Bmi088Accel accel(Wire1,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire1,0x68);

Adafruit_BMP3XX bmp;
Adafruit_LIS3MDL mdl;

int IMUinit(){
    int status;
    status = accel.begin();

    if (status < 0 )
    {
        Serial.print("accel init failure, error code: ");
        Serial.println(status);
        return 1;
    }

    accel.setRange(accel.RANGE_24G);
    

    status = gyro.begin();
    if (status < 0 )
    {
        Serial.print("gyro init failure, error code: ");
        Serial.println(status);
        return 2;
    }
    Serial.println("IMU init success");
    return 0;
}

int baroinit(){
    if (!bmp.begin_I2C(0x76,&Wire1))
    {
        Serial.println("BMP init failure");
        return 1;
    }
    Serial.println("BMP init success");
    return 0;
}

int maginit(){
    if (!mdl.begin_I2C(0x1C,&Wire1))
    {
        Serial.println("MAG init fail");
        return 1;
    }



    Serial.println("MAG init success");
    return 0;
}



uint8_t scani2c(){
    byte error, address;
    int nDevices;
    Serial.print("Scanning..");
    
    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire1.beginTransmission(address);
        error = Wire1.endTransmission();
        if (error == 0)
        {
        Serial.print("I2C device found at address 0x");
        if (address<16)
            Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");
    
        nDevices++;
        }
        else if (error==4)
        {
        Serial.print("Unknown error at address 0x");
        if (address<16)
            Serial.print("0");
        Serial.println(address,HEX);
        }    
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
    
    return 0;
}

#endif // Lyrav2sensors
