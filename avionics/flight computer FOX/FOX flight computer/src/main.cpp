#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>


Adafruit_MPU6050 imu;
Adafruit_BMP280 bmp;
Servo canards[4];



float localpressure = 1013.25;

float kp = 0.5;
float ki = 0.1;
float kd = 0.1;

float kproll = 1;
float kiroll = 0.1;
float kdroll = 0.1;

float preverror = 0.1;
float accumulatederror = 0.1;
unsigned long uptimemillis;
int rolloffset = 0;
// x+ x- y+ y-
int canardpos[4];
int canardpins[4] = {25, 26, 14, 13};
int canardsdefualtpos[4] = {90,  90, 90, 90};

float drift[3] = {0.55,0.52,0.62};

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

struct sensordata
{
  float accel_x;
  float accel_y;
  float accel_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  float imu_temp;
  float baro_temp;

  float baro_alt;
  float baro_pressure;

};

struct orientation
{
  float angle_x;
  float angle_y;
  float angle_z;
};


struct prevmillllis
{
  unsigned long cycle;
  unsigned long getdata;
  unsigned long serial;
  unsigned long pid;
  unsigned long controlcycle;
};

prevmillllis prevmilliss;
sensordata currentdata;
orientation currentorientation;

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

  imu.setFilterBandwidth(MPU6050_BAND_44_HZ);
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
  
  float deltatime = (millis() - prevmilliss.pid) / 1000;
  float error = setpoint - input;
  float deltaerror = error - preverror;
  accumulatederror += error * deltatime;
  float p = kp * error;
  float i = ki * accumulatederror;
  float d = kd * deltaerror;
  preverror = error;
  prevmilliss.pid = uptimemillis;
  return p;
  
}

void calibratempu() {

  unsigned long prevtime = millis();
  float valuesgyro[3] = {1,1,1};
  float valuesaccel[3] = {1,1,1};
  int iterations = 0;
  int targetiterations = 200;
  while (iterations <= targetiterations)
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
    
    if (iterations >= targetiterations)
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
      mpucalibratedgyros[i] = (valuesgyro[i]-1)/targetiterations;
      Serial.print(valuesgyro[i]-1);
      Serial.print(" ");
    }
    Serial.println(" accels: ");
    if (valuesaccel[i] <= 1.01 && valuesaccel[i] >= 0.99)
    {
      mpucalibratedaccell[i] = 0;
    }
    else{
      mpucalibratedaccell[i] = (valuesaccel[i]-1)/targetiterations;
      Serial.println(valuesaccel[i]-1);
    }
  }

}

sensordata getsensordata(){
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  sensordata data;
  data.accel_x = a.acceleration.x-mpucalibratedaccell[0];
  data.accel_y = a.acceleration.y-mpucalibratedaccell[1];
  data.accel_z = a.acceleration.z-mpucalibratedaccell[2];

  data.gyro_x = g.gyro.x-mpucalibratedgyros[0];
  data.gyro_y = g.gyro.y-mpucalibratedgyros[1];
  if (g.gyro.z-mpucalibratedaccell[2]<0.01 && g.gyro.z-mpucalibratedgyros[2]>-0.01)
  {
    data.gyro_z = 0;
  }
  else{
    data.gyro_z = g.gyro.z-mpucalibratedgyros[2];
  }

  data.imu_temp = temp.temperature;
  data.baro_temp = bmp.readTemperature();

  data.baro_alt = bmp.readAltitude(localpressure);
  data.baro_pressure = bmp.readPressure();
  
  return data;
}

void sendserialdata(sensordata data,orientation orient){
  Serial.print("");
  Serial.print(data.accel_x);
  Serial.print(", ");
  Serial.print(data.accel_y);
  Serial.print(", ");
  Serial.print(data.accel_z);

  Serial.print(",\t");
  Serial.print(data.gyro_x);
  Serial.print(", ");
  Serial.print(data.gyro_y);
  Serial.print(", ");
  Serial.print(data.gyro_z);

  Serial.print(",\t");
  Serial.print(orient.angle_x);
  Serial.print(", ");
  Serial.print(orient.angle_y);
  Serial.print(", ");
  Serial.print(orient.angle_z);

  Serial.print(",\t");
  Serial.print(data.imu_temp);
  Serial.print(", ");
  Serial.print(data.baro_alt);
  Serial.println(",");
}

orientation computeorientation(sensordata data, orientation orient){
  float timestep = (uptimemillis-prevmilliss.getdata)*0.1;
  orient.angle_x = orient.angle_x + (data.gyro_x*drift[0]) * timestep;
  orient.angle_y = orient.angle_y + (data.gyro_y*drift[1]) * timestep;
  orient.angle_z = orient.angle_z + (data.gyro_z*drift[2]) * timestep;
  return orient;
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
  currentdata = getsensordata();
  // put your setup code here, to run once:
}

void loop() {
  uptimemillis = millis();
  if (uptimemillis - prevmilliss.getdata > 10)
    {
      currentdata = getsensordata();
      currentorientation = computeorientation(currentdata,currentorientation);
      prevmilliss.getdata = uptimemillis;
    }
  // put your main code here, to run repeatedly:
  
  /* Print out the values */
  if (uptimemillis - prevmilliss.serial > 100)
  {
    sendserialdata(currentdata,currentorientation);
    prevmilliss.serial = uptimemillis;
  }

  if (uptimemillis - prevmilliss.controlcycle > 20)
  {
    rolloffset = PID(0, currentorientation.angle_z, kp, ki, kd);
    canardpos[0] = (canardsdefualtpos[0] + PID(0, currentorientation.angle_x, kp, ki, kd));
    canardpos[1] = (canardsdefualtpos[1] + (PID(0, currentorientation.angle_x, kp, ki, kd))*-1);
    canardpos[2] = (canardsdefualtpos[2] + PID(0, currentorientation.angle_y, kp, ki, kd));
    canardpos[3] = (canardsdefualtpos[3] + (PID(0, currentorientation.angle_y, kp, ki, kd))*-1);
    
    canardpos[0] += rolloffset;
    canardpos[1] += rolloffset;
    canardpos[2] += rolloffset;
    canardpos[3] += rolloffset;


    canards[0].write(canardpos[0]);
    canards[1].write(canardpos[1]);
    canards[2].write(canardpos[2]);
    canards[3].write(canardpos[3]);
    prevmilliss.controlcycle = uptimemillis;
  }
  
    

  prevmilliss.cycle = uptimemillis;
}
