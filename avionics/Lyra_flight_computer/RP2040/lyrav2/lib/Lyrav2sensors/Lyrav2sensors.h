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

const float SEALEVELPRESSURE = 1015.4;


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
    IMUdata data;

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

    void readIMU(int oversampling = 10){
        IMUdata _data;

        for (int i = 0; i < oversampling; i++)
        {
            accelunit.readSensor();
            gyrounit.readSensor();
            _data.accel.x += (int32_t(accelunit.getAccelX_mss()*10000));
            _data.accel.y += (int32_t(accelunit.getAccelY_mss()*10000));
            _data.accel.z += (int32_t(accelunit.getAccelZ_mss()*10000));

            _data.gyro.z += int32_t((gyrounit.getGyroX_rads()/(180/PI))*10000);
            _data.gyro.y += int32_t((gyrounit.getGyroY_rads()/(180/PI))*10000);
            _data.gyro.z += int32_t((gyrounit.getGyroZ_rads()/(180/PI))*10000);
            
            delayMicroseconds(200);
        }
        _data.accel.x /= oversampling;
        _data.accel.y /= oversampling;
        _data.accel.z /= oversampling;

        _data.gyro.z /= oversampling;
        _data.gyro.y /= oversampling;
        _data.gyro.z /= oversampling;

        _data.temp = accelunit.getTemperature_C();

        data = _data;

        return;
        
    }

};


class BARO
{

public:
    BARO(){};
    BAROdata data;

    int init(){
        if (!bmp.begin_I2C(0x76,&Wire1))
        {
            Serial.println("BMP init failure");
            return 1;
        }
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        Serial.println("BMP init success");
    return 0;
    }
    void readsensor(){
        
        data.altitude = bmp.readAltitude(SEALEVELPRESSURE)*10000;

        
    }

};

class MAG{

public:
    MAG(){};
    MAGdata data;
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
