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

float bcali[3] = {-0.007,0.0022,-0.0353};

float acali[3][3] = {
    {1.007907899,0,0},
    {0,1.007805928,0},
    {0,0,1.011191385}};

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

    void read(int oversampling = 50){
        IMUdata _data;

        
        for (int i = 0; i < oversampling; i++)
        {
            accelunit.readSensor();
            gyrounit.readSensor();

            _data.accel.x += accelunit.getAccelZ_mss();
            _data.accel.y += accelunit.getAccelY_mss();
            _data.accel.z += accelunit.getAccelX_mss();

            _data.gyro.x += gyrounit.getGyroZ_rads()*(180/PI);
            _data.gyro.y += gyrounit.getGyroY_rads()*(180/PI);
            _data.gyro.z += gyrounit.getGyroX_rads()*(57.29577941458908); // when the radians to degrees calculation of 180/PI is done at runtime, it breaks but this works so 

            delayMicroseconds(500);

        }
        
        _data.accel.x /= oversampling;
        _data.accel.y /= oversampling;
        _data.accel.z /= oversampling;
        
        _data.gyro.x /= oversampling;
        _data.gyro.y /= oversampling;
        _data.gyro.z /= oversampling;
        
        float currmeas[3] = {_data.accel.x-bcali[0],_data.accel.y-bcali[1],_data.accel.z-bcali[2]};
        //Serial.printf("%f, %f, %f gainadj: %f, %f, %f ",_data.accel.x,_data.accel.y,_data.accel.z,currmeas[0],currmeas[1],currmeas[2]);
        _data.accel.x = acali[0][0]*currmeas[0]+acali[1][0]*currmeas[1]+acali[2][0]*currmeas[2];
        _data.accel.y = acali[0][1]*currmeas[0]+acali[1][1]*currmeas[1]+acali[2][1]*currmeas[2];
        _data.accel.z = acali[0][2]*currmeas[0]+acali[1][2]*currmeas[1]+acali[2][2]*currmeas[2];
        //Serial.printf("multiplied: %f, %f, %f \n",_data.accel.x,_data.accel.y,_data.accel.z);
        
       
        
        _data.temp = accelunit.getTemperature_C();

        data = _data;

        return;
        
    }

};


class BARO
{
float prevalt;
float prevverticalvel[5];
int address = 0;
uint64_t prevtime;

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

        data.altitude = bmp.readAltitude(SEALEVELPRESSURE);
        data.pressure = bmp.readPressure();
        data.temp = bmp.readTemperature();

        float timestep = (micros() - prevtime);
        //Serial.printf(">timestep: %f \n",timestep);
        //prevverticalvel[address] = ((data.altitude - prevalt)/timestep);

        data.verticalvel = ((data.altitude - prevalt));//prevverticalvel[address];
        
        //for (int i = 0; i < 5; i++)
        //{
        //    data.verticalvel += prevverticalvel[i-1];
        //}
        //data.verticalvel /= 6;
        


        address >= 4 ? address = 0 : address++;

        prevalt = data.altitude;
        prevtime = micros();
    }

};

class MAG{

float bcali[3] = {-11.1517,-51.38845,0};

float acali[3][3] = {
    {0.4181084228,0,0},
    {0,0.3914369911,0},
    {0,0,1}};



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

    int read(){
        mdl.read();

        data.gauss.x = mdl.x;
        data.gauss.y = mdl.y;
        data.gauss.z = mdl.z;

        sensors_event_t event;
        
        mdl.getEvent(&event);

        data.utesla.x = event.magnetic.x;
        data.utesla.y = event.magnetic.y;
        data.utesla.z = event.magnetic.z;

        float currmeas[3] = {data.utesla.x-bcali[0],data.utesla.y-bcali[1],data.utesla.z-bcali[2]};
        //Serial.printf("%f, %f, %f gainadj: %f, %f, %f ",_data.accel.x,_data.accel.y,_data.accel.z,currmeas[0],currmeas[1],currmeas[2]);
        data.utesla.x = acali[0][0]*currmeas[0]+acali[1][0]*currmeas[1]+acali[2][0]*currmeas[2];
        data.utesla.y = acali[0][1]*currmeas[0]+acali[1][1]*currmeas[1]+acali[2][1]*currmeas[2];
        data.utesla.z = acali[0][2]*currmeas[0]+acali[1][2]*currmeas[1]+acali[2][2]*currmeas[2];

        data.headingdeg = atan2(data.utesla.y,data.utesla.x)*(180/PI);
        return 0;
    } 


};


#endif // Lyrav2sensors
