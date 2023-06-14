#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>


Adafruit_MPU6050 imu;
Adafruit_BMP280 bmp;
Servo canards[4];

float kp = 0.2;
float ki = 0.01;
float kd = 0.1;

float preverror = 0.1;
float accumulatederror = 0.1;
unsigned long prevmillis;

unsigned long uptimemillis;
// x+ x- y+ y-
int canardpos[4];
int canardpins[4] = {25, 26, 14, 13};
int canardsdefualtpos[4] = {104, 43, 67, 67};

float mpucalibratedgyros[3];
float mpucalibratedaccell[3];

struct datatolog
{
  int state;
  unsigned long uptimemillis;
  unsigned long missiontimemillis;

  float roll;
  float pitch;
  float yaw;

  float roll_rate;
  float pitch_rate;
  float yaw_rate;

  float x_accel;
  float y_accel;
  float z_accel;

  float imu_alt;
  float imu_velocity;

  float vertical_velocity;

  float imu_temp;
  float baro_temp;

  float baro_alt;
  float baro_pressure;

  int pyrostatus1;
  int pyrostatus2;

  int canardstatusx1;
  int canardstatusx2;

  int canardstatusy1;
  int canardstatusy2;



};

struct datatotransmit
{
  int state;
  unsigned long uptimemillis;
  unsigned long missiontimemillis;

  float roll;
  float pitch;
  float yaw;

  float roll_rate;
  float pitch_rate;
  float yaw_rate;

  float x_accel;
  float y_accel;
  float z_accel;

  float imu_alt;
  float imu_velocity;

  float vertical_velocity;

  float baro_alt;
  
  int pyrostatus1;
  int pyrostatus2;

};


void initImu() {
  Serial.println("Initializing IMU...");

  if (!imu.begin())
  {
    Serial.println("IMU initialization failed");
    while (1){
      delay(10);
    }
  }
  Serial.println("IMU initialization succeeded");

  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (imu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (imu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (imu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("good imu init");
  delay(10);
}

void initbaro() {
 Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
}
}

float PID(float setpoint, float input, float kp, float ki, float kd) {
  
  float deltatime = (millis() - prevmillis) / 1000;
  float error = setpoint - input;
  float deltaerror = error - preverror;
  accumulatederror += error * deltatime;
  float p = kp * accumulatederror;
  float i = ki * error;
  float d = kd * deltaerror;
  preverror = error;
  prevmillis = uptimemillis;
  return p + i + d;
  
}

void calibratempu() {

  unsigned long prevtime = millis();
  float valuesgyro[3] = {1,1,1};
  float valuesaccel[3] = {1,1,1};
  int iterations = 0;
  while (iterations <= 20)
  {
    if (millis()-prevtime > 1000)
    {
      sensors_event_t a, g, temp;
      imu.getEvent(&a, &g, &temp);
      valuesgyro[0] += g.gyro.x;
      valuesgyro[1] += g.gyro.y;
      valuesgyro[2] += g.gyro.z;
      valuesaccel[0] += a.acceleration.x-9.81;
      valuesaccel[1] += a.acceleration.y;
      valuesaccel[2] += a.acceleration.z;
      iterations++;
      Serial.print("X: ");
      Serial.print(a.acceleration.x);
      Serial.print(", Y: ");
      Serial.print(a.acceleration.y);
      Serial.print(", Z: ");
      Serial.print(a.acceleration.z);
      Serial.print("X: ");
      Serial.print(valuesaccel[0]);
      Serial.print(", Y: ");
      Serial.print(valuesaccel[1]);
      Serial.print(", Z: ");
      Serial.print(valuesaccel[2]);
      Serial.print(", iterations: ");
      Serial.println(iterations);
      
    }
    
    if (iterations >= 20)
    {
      break;
    }
    
  }
  Serial.print(" X: ");
  Serial.print(valuesgyro[0]);
  Serial.print(", Y: ");
  Serial.print(valuesgyro[1]);
  Serial.print(", Z: ");
  Serial.print(valuesgyro[2]);
  Serial.print(" accel X: ");
  Serial.print(valuesaccel[0]);
  Serial.print(", Y: ");
  Serial.print(valuesaccel[1]);
  Serial.print(", Z: ");
  Serial.print(valuesaccel[2]);
  Serial.println("\t");
  for (int i = 0; i < 2; i++)
  {
    if (valuesgyro[i] <= 1.01 && valuesgyro[i] >= 0.99)
    {
      mpucalibratedgyros[i] = 0;
    }
    else{
      mpucalibratedgyros[i] = (valuesgyro[i]-1)/20;
      Serial.print(valuesgyro[i]-1);
      Serial.print(" ");
    }
    Serial.println(" accels: ");
    if (valuesaccel[i] <= 1.01 && valuesaccel[i] >= 0.99)
    {
      mpucalibratedaccell[i] = 0;
    }
    else{
      mpucalibratedaccell[i] = (valuesaccel[i]-1)/20;
      Serial.println(valuesaccel[i]-1);
    }
  }

}
void setup() {
  uptimemillis = millis();
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("MPU init");

  initImu();
  initbaro();

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  for (int i = 0; i < 3; i++)
  {
    canards[i].setPeriodHertz(50);
    canards[i].attach(canardpins[i], 500, 2400);
    canards[i].write(canardsdefualtpos[i]);
  }
   
  calibratempu();
  // put your setup code here, to run once:
}

void loop() {
  uptimemillis = millis();
  // put your main code here, to run repeatedly:

  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("X: ");
  Serial.print(a.acceleration.x-mpucalibratedaccell[0]);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y-mpucalibratedaccell[1]);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z-mpucalibratedaccell[2]);
  Serial.print("\t");

  Serial.print("X: ");
  Serial.print(g.gyro.x-mpucalibratedgyros[0]);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y-mpucalibratedgyros[1]);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z-mpucalibratedgyros[2]);
  Serial.print("\t");

  Serial.print("temp: ");
  Serial.print(temp.temperature);
  Serial.print("\t");

  Serial.print(F("alt = "));
  Serial.print(bmp.readAltitude(1020.65)); /* Adjusted to local forecast! */
  Serial.println(" m");
  delay(100);
}
