#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <Wifi.h>
#include <esp_now.h>
#include <Preferences.h>


Adafruit_MPU6050 imu;
Adafruit_BMP280 bmp;
Servo canards[4];
Preferences flash;


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
unsigned long missiontimemillis;
int rolloffset = 0;
// x+ x- y+ y-
int canardpos[4];
int canardpins[4] = {25, 26, 14, 13};
int canardsdefualtpos[4] = {90,  90, 90, 90};

int detectiontries = 1;


float prevbaroalt;

float drift[3] = {0.55,0.52,0.62};

int buzzerpin = 16;
int battpin = 32;

float battvoltage;

float mpucalibratedgyros[3];
float mpucalibratedaccell[3];

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

int state = 0;

struct intervals
{
  int datalogging;
  int sendtelem;
  int getdata;
  int sendserial;
};


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

  float accel_x;
  float accel_y;
  float accel_z;
  float absaccel;

  float velocity;
  float vertical_velocity;

  float imu_temp;
  float baro_temp;

  float baro_alt;
  float baro_pressure;

  int canardstatusx1;
  int canardstatusx2;

  int canardstatusy1;
  int canardstatusy2;

  int command;

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

  float accel_x;
  float accel_y;
  float accel_z;

  float vel_x;
  float vel_y;
  float vel_z;

  float absaccel;
  float absvel;

  float vertical_vel;
  float baro_alt;

  float batteryvolt;
};

struct datanew
{
  int command;
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

  float absaccel;
};

struct orientation
{
  float angle_x;
  float angle_y;
  float angle_z;
  float vel_x;
  float vel_y;
  float vel_z;
  float absvel;
  float verticalvel;
};


struct prevmillllis
{
  unsigned long cycle;
  unsigned long getdata;
  unsigned long serial;
  unsigned long pid;
  unsigned long controlcycle;
  unsigned long telemtransmit;
  unsigned long detectionprevmillis;
};


prevmillllis prevmilliss;
sensordata currentdata;
orientation currentorientation;
datanew grounddata;

intervals stateintervals[7] = {
  {2000,1000,100,200}, //ground idle
  {50,50,20,100}, // ready to launch
  {50,20,20,100}, // powered ascent
  {50,20,20,100}, // unpowered ascent
  {50,20,20,100}, // ballistic decsent
  {50,20,20,100}, // retarded descent
  {1000,100,100,200} // landed
};


datatotransmit transmitdata;

esp_now_peer_info_t peerInfo;

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

void beep(int freq, int dur){
  tone(buzzerpin, freq, dur);
}

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


void calibratempu() {
  currentorientation.angle_x = 0;
  currentorientation.angle_y = 0;
  currentorientation.angle_z = 0;
  currentorientation.absvel = 0;
  currentorientation.vel_x = 0;
  currentorientation.vel_y = 0;
  currentorientation.vel_z = 0;
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
      valuesaccel[0] += a.acceleration.x;
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
  for (int i = 0; i < 3; i++)
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
    if (valuesaccel[i] <= 1.001 && valuesaccel[i] >= 0.999)
    {
      mpucalibratedaccell[i] = 0;
    }
    else{
      mpucalibratedaccell[i] = (valuesaccel[i]-1)/targetiterations;
      Serial.println(valuesaccel[i]-1);
    }
  }
  prevmilliss.getdata = uptimemillis;

  mpucalibratedaccell[1] += 9.81;
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

  data.absaccel = sqrt(pow(data.accel_x,2)+pow(data.accel_y,2)+pow(data.accel_y,2));
  
  return data;
}

orientation computeorientation(sensordata data, orientation orient){
  float timestep = (uptimemillis-prevmilliss.getdata)*0.1;
  orient.angle_x = orient.angle_x + (data.gyro_x*drift[0]) * timestep;
  orient.angle_y = orient.angle_y + (data.gyro_y*drift[1]) * timestep;
  orient.angle_z = orient.angle_z + (data.gyro_z*drift[2]) * timestep;

  orient.vel_x = orient.vel_x + (data.accel_x*timestep);
  orient.vel_y = orient.vel_y + ((data.accel_y+9.81)*timestep);
  orient.vel_z = orient.vel_z + (data.accel_z*timestep);
  orient.absvel = sqrt(pow(orient.vel_x,2)+pow(orient.vel_y,2)+pow(orient.vel_z,2));

  orient.verticalvel = (data.baro_alt-prevbaroalt)*timestep;
  prevbaroalt = data.baro_alt;

  return orient;
}

datatotransmit preptelemetry(sensordata data, orientation orient){
  datatotransmit message;
  message.state = state;
  message.accel_x = data.accel_x;
  message.accel_y = data.accel_y;
  message.accel_z = data.accel_z;

  message.vel_x = orient.vel_x;
  message.vel_y = orient.vel_y;
  message.vel_z = orient.vel_z;

  message.absvel = orient.absvel;

  message.pitch_rate = data.gyro_x;
  message.yaw_rate = data.gyro_y;
  message.roll_rate= data.gyro_z;

  message.pitch = orient.angle_x;
  message.yaw = orient.angle_y;
  message.roll = orient.angle_z;

  message.baro_alt = data.baro_alt;
  message.vertical_vel = orient.verticalvel;

  message.uptimemillis = uptimemillis;

  message.absaccel = data.absaccel;

  message.missiontimemillis = missiontimemillis;

  message.batteryvolt = battvoltage;
  return message;
}

datatolog prepdatalog(sensordata data, orientation orient){
  datatolog message;
  message.uptimemillis = uptimemillis;
  message.missiontimemillis = missiontimemillis;
  message.state = state;
  message.accel_x = data.accel_x;
  message.accel_y = data.accel_y;
  message.accel_z = data.accel_z;

  message.pitch_rate = data.gyro_x;
  message.yaw_rate = data.gyro_y;
  message.roll_rate= data.gyro_z;

  message.pitch = orient.angle_x;
  message.yaw = orient.angle_y;
  message.roll = orient.angle_z;

  message.baro_alt = data.baro_alt;
  message.baro_pressure = data.baro_pressure;

  message.imu_temp = data.imu_temp;
  message.baro_temp = data.baro_temp;

  message.canardstatusx1 = canardpos[0];
  message.canardstatusx2 = canardpos[1];

  message.canardstatusy1 = canardpos[2];
  message.canardstatusy2 = canardpos[3];

  message.command = grounddata.command;

  return message;
}

void onsendtelem(const uint8_t *mac_addr, esp_now_send_status_t status){
  
}

void onrecivetelem(const uint8_t *macAddr, const uint8_t *data, int dataLen){
  datanew* telemetrytemp = (datanew*) data;
  grounddata = *telemetrytemp;
  Serial.print("Received");
  beep(10000,50);
  
}


void broadcast(const datatotransmit &message)
// Emulates a broadcast
{
  // Broadcast a message to every device in range
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peerInfo = {};
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
  if (!esp_now_is_peer_exist(broadcastAddress))
  {
    esp_now_add_peer(&peerInfo);
  }
  // Send message
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
  // Print results to serial monitor
  /*
  if (result == ESP_OK)
  {
    Serial.print("Broadcast message success ");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    Serial.println("ESP-NOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.println("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Unknown error");
  }
  */
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
  Serial.print(", ");
  Serial.print(battvoltage);
  Serial.println(",");
}

void logdata(datatolog data){

}


void setup() {
  beep(2000, 100);
  uptimemillis = millis();
  Serial.begin(115200);
  Serial.println("MPU init");
  initImu();
  initbaro();
  beep(3000, 100);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println("WiFi init ");
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();
  beep(4000, 100);

  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESP-NOW Init Success");
    esp_now_register_recv_cb(onrecivetelem);
    esp_now_register_send_cb(onsendtelem);
  }
  else
  {
    Serial.println("ESP-NOW Init Failed");

  }
  beep(3000, 100);
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
  flash.begin("data",false);
  beep(5000, 100);
  delay(100);
  beep(5000, 100);
  // put your setup code here, to run once:
}

void loop() {
  uptimemillis = millis();
  if (uptimemillis - prevmilliss.getdata > stateintervals[state].getdata)
    {
      currentdata = getsensordata();
      currentorientation = computeorientation(currentdata,currentorientation);
      battvoltage = (float(analogRead(battpin))-780)/180;
      prevmilliss.getdata = uptimemillis;
    }
  // put your main code here, to run repeatedly:
  
  if (uptimemillis - prevmilliss.serial > stateintervals[state].sendserial)
  {
    sendserialdata(currentdata,currentorientation);
    prevmilliss.serial = uptimemillis;
  }
  if (uptimemillis - prevmilliss.controlcycle > 20 && state == 2)
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
  
  if (uptimemillis - prevmilliss.telemtransmit > stateintervals[state].sendtelem)
  {
    datatotransmit telemetry = preptelemetry(currentdata,currentorientation);
    broadcast(telemetry);
    prevmilliss.telemtransmit = uptimemillis;
  }

  if (grounddata.command != 0)
  {
    switch (grounddata.command)
    {
    case 109:
      calibratempu();
      grounddata.command = 0;
      break;
    
    case 108:
      state = 1;
      grounddata.command = 0;
      break;
    
    default:
      break;
    }
  }
  
  if (uptimemillis - prevmilliss.detectionprevmillis < 100)
  {
    switch (state)
    {
    case 1:
      // detecting liftoff
      if (currentdata.absaccel > 15 && detectiontries <= 4)
      {
        detectiontries++;
      }
      else if (detectiontries >= 4){
        state = 2;
        detectiontries = 0;
      }
      else{detectiontries = 0;}
      break;
    case 2:
      // detecting burnout
      if (currentdata.absaccel < 2 && detectiontries <= 4)
      {
        detectiontries++;
      }
      else if (detectiontries >= 4){
        state = 3;
        detectiontries = 0;
      }
      else{detectiontries = 0;}
      break;
    
    case 3:
      // detecting apoogee
      if (currentorientation.verticalvel < 5 && detectiontries <= 4)
      {
        detectiontries++;
      }
      else if (detectiontries >= 4){
        state = 4;
        detectiontries = 0;
      }
      else{detectiontries = 0;}
      break;

    case 4:
      // detecting  parachute openeing
      if (currentdata.absaccel > 6 && detectiontries <= 4)
      {
        detectiontries++;
      }
      else if (detectiontries >= 4){
        state = 5;
        detectiontries = 0;
      }
      else{detectiontries = 0;}
      break;

    case 5:
      // detecting landing
      if (currentorientation.verticalvel > 3 && detectiontries <= 4)
      {
        detectiontries++;
      }
      else if (detectiontries >= 4){
        state = 6;
        detectiontries = 0;
      }
      else{detectiontries = 0;}
      break;
    
    default:
      break;
    }
  }
  
  
    
  prevmilliss.detectionprevmillis = uptimemillis;
  prevmilliss.cycle = uptimemillis;
}
