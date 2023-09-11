#if !defined(Lyrav2sensors)
#define Lyrav2sensors
#include <Arduino.h>
#include <Wire.h>

#include <macros.h>
#include <quats.h>

#include <BMI088.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LIS3MDL.h>


/* accelunit object */
Bmi088Accel accelunit(Wire1,0x18);
/* gyrounit object */
Bmi088Gyro gyrounit(Wire1,0x68);

Adafruit_BMP3XX bmp;
Adafruit_LIS3MDL mdl;




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
    if (nDevices == 0){
        Serial.println("No I2C devices found\n");
        return 1;
    }
    else
        Serial.println("done\n");
    
    return 0;
}

class IMU{
    


public:
    IMU(){};
    Vector3int32 accel;
    Vector3int32 gyro;
    int32_t temp;

    int init(){
        int status;
        status = accelunit.begin();

        if (status < 0 )
        {
            Serial.print("accelunit init failure, error code: ");
            Serial.println(status);
            return 1;
        }

        accelunit.setRange(accelunit.RANGE_24G);
        
        status = gyrounit.begin();
        if (status < 0 )
        {
            Serial.print("gyrounit init failure, error code: ");
            Serial.println(status);
            return 2;
        }
            Serial.println("IMU init success");
            return 0;
        }

};


class BARO
{

public:
    BARO(){};

    int init(){
        if (!bmp.begin_I2C(0x76,&Wire1))
        {
            Serial.println("BMP init failure");
            return 1;
        }
        Serial.println("BMP init success");
    return 0;
}
};

class MAG{

public:
    MAG(){};

    int init(){
        if (!mdl.begin_I2C(0x1C,&Wire1))
        {
            Serial.println("MAG init fail");
            return 1;
        }

        Serial.println("MAG init success");
    return 0;
}



};


#endif // Lyrav2sensors
