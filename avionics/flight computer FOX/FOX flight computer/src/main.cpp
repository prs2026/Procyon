#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <Wifi.h>
#include <esp_now.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <procyonlib.h>

#include <SPIFFS.h>

MPU6050 imu;

Adafruit_BMP280 bmp;
Servo canards[4];

File file;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 34
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vecto



/*
float kp = 0.5;
float ki = 0.1;
float kd = 0.1;

float kproll = 1;
float kiroll = 0.1;
float kdroll = 0.1;

float preverror = 0.1;
float accumulatederror = 0.1;



int rolloffset = 0;
// x+ x- y+ y-
int canardpos[4];
int canardpins[4] = {25, 26, 14, 13};
int canardsdefualtpos[4] = {90,  90, 90, 90};
*/
int detectiontries = 1;

unsigned long uptimemillis;
unsigned long missiontimemillis;


float prevbaroalt;

float drift[3] = {0.55,0.52,0.62};

int buzzerpin = 16;
int battpin = 32;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float battvoltage;

float mpucalibratedgyros[3];
float mpucalibratedaccell[3];
float bmpcalibratedaltidue;


float accelbias[3] = {0.6378775,0.0223087,-1.2146027};
float gyrobias[3] = {-0.0389415,0.0166735,0.0245533};

bool sendserial = false;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

int state = 0;

int dataaddress;

int commandbuf;
const int aveamount = 5;
float rawabsaccelreadings[aveamount];
float verticalvelreadings[aveamount];
int indicies = 0;

float averagerawabsaccel,averageverticalver;

struct intervals
{
  int datalogging;
  int sendtelem;
  int getdata;
  int sendserial;
};


struct sensordata
{
  float adjaccel_x;
  float adjaccel_y;
  float adjaccel_z;

  float rawaccel_x;
  float rawaccel_y;
  float rawaccel_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  float imu_temp;
  float baro_temp;

  float baro_alt;
  float baro_pressure;

  float vertical_vel;

  float adjabsaccel;
  float rawabsaccel;

  float pitch;
  float yaw;
  float roll;

  float rawpitch;
  float rawyaw;
  float rawroll;

  float absvel;
  
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
  unsigned long logdata;
};



prevmillllis prevmilliss;
sensordata currentdata;
datanew grounddata;

intervals stateintervals[7] = {
  {5000,1000,100,80}, //ground idle
  {100,50,25,100}, // ready to launch
  {50,20,25,100}, // powered ascent
  {50,20,25,100}, // unpowered ascent
  {50,20,25,100}, // ballistic decsent
  {50,20,25,100}, // retarded descent
  {1000,20,100,200} // landed
};



datatotransmit transmitdata;

esp_now_peer_info_t peerInfo;
/*
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
*/

//mpu interrupt deteteciton routine

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// function definintions
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

void initimu(){
  // initlizing and testing the imu
  Serial.println("IMU init");
  imu.initialize();
  pinMode(INTERRUPT_PIN,INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(imu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = imu.dmpInitialize();
  // gyro offsets
  imu.setXGyroOffset(88);
  imu.setYGyroOffset(17);
  imu.setZGyroOffset(-34);
  imu.setXAccelOffset(-1610);
  imu.setYAccelOffset(-4511);
  imu.setZAccelOffset(3418); // 1688 factory default for my test chip

  //-1610,   -4511,    3418,      88,     -17,     -34
  // accel then gyro
  if (devStatus == 0)
  { 
    // calibrate accels and gyros
    imu.setFullScaleAccelRange(8);
    imu.CalibrateAccel(6);
    imu.CalibrateGyro(6);
    imu.PrintActiveOffsets();
    

    //enable dmp
    imu.setDMPEnabled(true);

    // enable inturrupts
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = imu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = imu.dmpGetFIFOPacketSize();
  }
  
}


void calibrateimu(){
  imu.CalibrateAccel(12);
  imu.CalibrateGyro(12);
  imu.PrintActiveOffsets();
}

void calibratebaro(){
  int targetiterations = 50;
  float alt;
  for (int i = 0; i < targetiterations; i++)
  {
    alt += bmp.readAltitude(1013.25);
    Serial.print(alt);
    Serial.print(" iter ");
    Serial.println(i);
    delay(50);
  }
  alt = alt/targetiterations;
  bmpcalibratedaltidue = alt;
}

sensordata getsensordata(){
  sensordata data;
  float timestep = (uptimemillis - prevmilliss.getdata);
  int scalefactor = 16384;
  if (imu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    imu.dmpGetQuaternion(&q, fifoBuffer);
    imu.dmpGetGravity(&gravity, &q);
    imu.dmpGetAccel(&aa, fifoBuffer);
    imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    imu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    data.pitch = ypr[1] * 180/M_PI;
    data.yaw = ypr[0] * 180/M_PI;
    data.roll = ypr[2] * 180/M_PI;

    data.adjaccel_x = ((float(aaReal.x)/scalefactor)*9.81);
    data.adjaccel_y = ((float(aaReal.y)/scalefactor)*9.81);
    data.adjaccel_z = ((float(aaReal.z)/scalefactor)*9.81);

    data.rawaccel_x = ((float(ax)/scalefactor)*9.81);
    data.rawaccel_y = ((float(ay)/scalefactor)*9.81);
    data.rawaccel_z = ((float(az)/scalefactor)*9.81);

    data.rawabsaccel = sqrt(pow(data.rawaccel_x,2)+pow(data.rawaccel_y,2)+pow(data.rawaccel_z,2));

    data.adjabsaccel = sqrt(pow(data.adjaccel_x,2)+pow(data.adjaccel_y,2)+pow(data.adjaccel_z,2));

    data.gyro_x = gx;
    data.gyro_y = gy;
    data.gyro_z = gz;

    data.rawpitch = data.pitch + (data.gyro_x * timestep);
    data.rawyaw = data.yaw + (data.gyro_y * timestep);
    data.rawroll = data.roll + (data.gyro_z * timestep);

  }

  data.baro_alt = bmp.readAltitude() - bmpcalibratedaltidue;
  data.baro_pressure = bmp.readPressure();
  data.baro_temp =  bmp.readTemperature();

  data.vertical_vel = (data.baro_alt - prevbaroalt) *timestep;
  prevbaroalt = data.baro_alt;

  battvoltage = analogRead(battpin)-780;

  rawabsaccelreadings[indicies] = data.rawabsaccel;
  verticalvelreadings[indicies] = data.vertical_vel;
  indicies ++;
  if (indicies >= aveamount-1)
  {
    indicies = 0;
  }
  
  for (int i = 0; i < aveamount; i++)
  {
    averagerawabsaccel += rawabsaccelreadings[i];
    averageverticalver += rawabsaccelreadings[i];

  }
  averagerawabsaccel = averagerawabsaccel/aveamount;
  averageverticalver = averageverticalver/aveamount;
  
  switch (state)
    {
    case 1:
      // detecting liftoff
      
      if (averagerawabsaccel >= 23)
      {
        state = 2;
      }

      break;
    case 2:
      // detecting burnout
      if (averagerawabsaccel <= 13)
      {
        state = 3;
      }
      break;
    
    case 3:
      // detecting apoogee
      if (averageverticalver <= 12)
      {
        state = 4;
      }
      break;

    case 4:
      // detecting  parachute openeing
      if (averagerawabsaccel >= 7)
      {
        state = 5;
      }
      break;

    case 5:
      // detecting landing
      if (averageverticalver <= 20 && uptimemillis - missiontimemillis > 10000)
      {
        state = 6;
      }
      break;
    
    default:
      break;
    }
  return data;
}

datatotransmit preptelemetry(sensordata data){
  datatotransmit message;
  message.state = state;
  message.accel_x = data.rawaccel_x;
  message.accel_y = data.rawaccel_y;
  message.accel_z = data.rawaccel_z;

  message.absvel = data.absvel;

  message.pitch_rate = data.gyro_x;
  message.yaw_rate = data.gyro_y;
  message.roll_rate= data.gyro_z;

  message.pitch = data.rawpitch;
  message.yaw = data.rawyaw;
  message.roll = data.rawroll;

  message.baro_alt = data.baro_alt;
  message.vertical_vel = data.vertical_vel;

  message.uptimemillis = uptimemillis;

  message.absaccel = data.adjabsaccel;

  message.missiontimemillis = missiontimemillis;

  message.batteryvolt = battvoltage;
  return message;
}


void onsendtelem(const uint8_t *mac_addr, esp_now_send_status_t status){
  
}

void onrecivetelem(const uint8_t *macAddr, const uint8_t *data, int dataLen){
  datanew* telemetrytemp = (datanew*) data;
  grounddata = *telemetrytemp;
  Serial.print("Received");
  tone(buzzerpin,10000,50);
  
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

void sendserialdata(sensordata data){
  int decimals = 4;
  Serial.print(uptimemillis);
  Serial.print(", ");

  Serial.print(state);
  Serial.print(",\t");

  Serial.print(data.adjaccel_x,decimals);
  Serial.print(", ");
  Serial.print(data.adjaccel_y,decimals);
  Serial.print(", ");
  Serial.print(data.adjaccel_z,decimals);
  
  Serial.print(", ");
  Serial.print(data.rawaccel_x,decimals);
  Serial.print(", ");
  Serial.print(data.rawaccel_y,decimals);
  Serial.print(", ");
  Serial.print(data.rawaccel_z,decimals);

  Serial.print(", \t");
  Serial.print(data.rawabsaccel,decimals);
  Serial.print(",");
  Serial.print(data.adjabsaccel,decimals);

  Serial.print(",\t");
  Serial.print(data.gyro_x,decimals);
  Serial.print(", ");
  Serial.print(data.gyro_y,decimals);
  Serial.print(", ");
  Serial.print(data.gyro_z,decimals);

  Serial.print(",\t");
  Serial.print(data.pitch,decimals);
  Serial.print(", ");
  Serial.print(data.yaw,decimals);
  Serial.print(", ");
  Serial.print(data.roll,decimals);

  Serial.print(",\t");
  Serial.print(data.rawpitch,decimals);
  Serial.print(", ");
  Serial.print(data.rawyaw,decimals);
  Serial.print(", ");
  Serial.print(data.rawroll,decimals);

  Serial.print(",\t");
  Serial.print(data.imu_temp);
  Serial.print(", ");
  Serial.print(data.baro_alt);
  Serial.print(", ");
  Serial.print(battvoltage);
  Serial.print(", ");
  Serial.print(averagerawabsaccel);
  Serial.println(",");
}

void logdata(sensordata data){
  int datapoints = 30;
  //long datatobelogged[datapoints];
  int decimals = 2;
  decimals = pow(10,decimals);
  /*
  file.write(10101);
  file.write(uptimemillis);
  file.write(missiontimemillis);
  file.write(state);
  file.write(data.adjaccel_x*decimals);
  file.write(data.adjaccel_y*decimals);
  file.write(data.adjaccel_z*decimals);
  file.write(data.gyro_x*decimals);
  file.write(data.gyro_y*decimals);
  file.write(data.gyro_z*decimals);
  file.write(data.pitch*decimals);
  file.write(data.yaw*decimals);
  file.write(data.roll*decimals);
  file.write(data.baro_alt*decimals);
  file.write(data.baro_pressure*decimals);
  file.write(data.baro_temp*decimals);
  file.write(data.imu_temp*decimals);
  file.write(commandbuf*decimals);
  file.write(data.rawaccel_x*decimals);
  file.write(data.rawaccel_y*decimals);
  file.write(data.rawaccel_z*decimals);
  file.write(data.rawpitch*decimals);
  file.write(data.rawyaw*decimals);
  file.write(data.rawroll*decimals);
  file.write(battvoltage*decimals);
  file.write(data.adjabsaccel*decimals);
  file.write(data.rawabsaccel*decimals);
  file.write(data.vertical_vel*decimals);
  file.write(20202);
  */
    String datatobelogged = "10101," + String(uptimemillis) + "," + String(missiontimemillis) + "," + String(state) + "," + 
    String(data.adjaccel_x*decimals) + "," + String(data.adjaccel_y*decimals) + "," + String(data.adjaccel_z*decimals) + "," + 
    String(data.gyro_x*decimals) + "," + String(data.gyro_y*decimals) + "," + String(data.gyro_z*decimals) + "," + 
    String(data.pitch*decimals) + "," + String(data.yaw*decimals) + "," + String(data.roll*decimals) + "," + 
    String(data.baro_alt*decimals) + "," + String(data.baro_pressure*decimals) + "," + String(data.baro_temp*decimals) + "," + 
    String(data.imu_temp*decimals) + "," + String(commandbuf*decimals) + "," + 
    String(data.rawaccel_x*decimals) + "," + String(data.rawaccel_y*decimals) + "," + String(data.rawaccel_z*decimals) + "," +
    String(data.rawpitch*decimals) + "," + String(data.rawyaw*decimals) + "," + String(data.rawroll*decimals) + "," + 
    String(battvoltage*decimals)+ "," + 
    String(data.adjabsaccel*decimals) + "," + String(data.rawabsaccel*decimals) + "," + String(data.vertical_vel*decimals) + 
    "," + String(averagerawabsaccel*decimals) + "," + String(averageverticalver*decimals) +  ","+"20202"
    ;
  
  
  
  
  
  commandbuf = 0;
  file.println(datatobelogged);
}

void dumpdatatoserial(){
  file.close();
  Serial.println("checksum,foxuptime,missiontime,state,x_accel,y accel, z accel,gyro x, gyro y, gyro z, pitch, yaw ,roll, barometeric alt, baro pressure,baro temp, imu temp, command, raw x accel, raw y accel, raw z accel, raw pitch, raw yaw, raw roll,battery voltage,adjabsaccel,rawabsaccel,vertical velocity, rawabsvel average,ertical velocity average, terminalchecksum");
  char filename[] = "/data.csv";
  file = SPIFFS.open(filename, "r");
  while (file.available())
  {
    Serial.print(file.readString());
  }
  
  sendserial = false;
  file.close();
  file = SPIFFS.open(filename, "a");
}

void clearloggeddata(){
  dumpdatatoserial();
  file.close();
  file = SPIFFS.open("/data.csv","w");
  Serial.print("Cleared flash, current address: ");
  
}


void setup() {
  tone(buzzerpin,2000,100);
  uptimemillis = millis();
  Serial.begin(115200);

  analogSetWidth(10);

  Wire.begin();
  Wire.setClock(400000);
  initimu();

  initbaro();
  tone(buzzerpin,3000, 100);


  WiFi.mode(WIFI_MODE_STA);
  Serial.println("WiFi init ");
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();

  tone(buzzerpin,4000, 100);

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
  
  tone(buzzerpin,3000, 100);
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  
  //calibratempu();
  calibratebaro();
  currentdata = getsensordata();
  

  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS init failure");
  }
  
  file = SPIFFS.open("/data.csv", "a");

  if (!file)
  {
    Serial.println("File open failed");
  }


  

  tone(buzzerpin,5000, 100);
  delay(100);
  tone(buzzerpin,5000, 100);
  // put your setup code here, to run once:
}

void loop() {
  uptimemillis = millis();
  if (uptimemillis - prevmilliss.getdata > stateintervals[state].getdata)
    {
      currentdata = getsensordata();
      
      prevmilliss.getdata = uptimemillis;
    }
  // put your main code here, to run repeatedly:
  
  if (uptimemillis - prevmilliss.logdata > stateintervals[state].datalogging){
    logdata(currentdata);
    prevmilliss.logdata = uptimemillis;
  }

  if (uptimemillis - prevmilliss.serial > stateintervals[state].sendserial && sendserial == true)
  {

    sendserialdata(currentdata);
    prevmilliss.serial = uptimemillis;
  }
  if (Serial.available() > 0)
  {
      int incomingbyte = Serial.read();
      Serial.print("echo: ");
      Serial.println(incomingbyte);
      grounddata.command = incomingbyte;
    }
  
  if (uptimemillis - prevmilliss.telemtransmit > stateintervals[state].sendtelem)
  {
    datatotransmit telemetry = preptelemetry(currentdata);
    broadcast(telemetry);
    prevmilliss.telemtransmit = uptimemillis;
  }

  if (grounddata.command != 0)
  {
    commandbuf == grounddata.command;
    switch (grounddata.command)
    {
    case 109:
      currentdata.rawpitch = currentdata.pitch;
      currentdata.rawyaw = currentdata.yaw;
      currentdata.rawroll = currentdata.roll;
      currentdata.absvel = 0;
      
      calibrateimu();

      grounddata.command = 0;
      break;

    case 112:
      //calibratempunew();
      
      grounddata.command = 0;
      break;
    
    case 108:
      state = 1;
      grounddata.command = 0;
      break;
    
    case 98:
      calibratebaro();
      grounddata.command = 0;
      break;
    
    case 116:
      ESP.restart();
      grounddata.command = 0;
      break;
    
    case 117:
      Serial.println(SPIFFS.totalBytes());
      Serial.println(SPIFFS.usedBytes());
      grounddata.command = 0;
      break;
    
    case 119:
      tone(buzzerpin,5000,200);
      delay(500);
      tone(buzzerpin,5000,200);
      grounddata.command = 0;
      break;

    case 115:
      sendserial = !sendserial;
      Serial.write("toggleserial");
      grounddata.command = 0;
      break;
    
    case 100:
      dumpdatatoserial();
      
      grounddata.command = 0;
      break;
    
    case 99:
      clearloggeddata();
      
      grounddata.command = 0;
      break;

    default:
      break;
    }
  }
  /*
  if (uptimemillis - prevmilliss.detectionprevmillis > 25)
  {
    switch (state)
    {
    case 1:
      // detecting liftoff
      
      if (averagerawabsaccel >= 15)
      {
        state = 2;
      }

      break;
    case 2:
      // detecting burnout
      if (averagerawabsaccel <= 5)
      {
        state = 3;
      }
      break;
    
    case 3:
      // detecting apoogee
      if (averageverticalver <= 5)
      {
        state = 4;
      }
      break;

    case 4:
      // detecting  parachute openeing
      if (averagerawabsaccel >= 7)
      {
        state = 5;
      }
      break;

    case 5:
      // detecting landing
      if (averageverticalver <= 5 && uptimemillis - missiontimemillis > 10000)
      {
        state = 6;
      }
      break;
    
    default:
      break;
    }
  }
  */
  
    
  prevmilliss.detectionprevmillis = uptimemillis;
  prevmilliss.cycle = uptimemillis;


  /*
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
  */

 /*
void calibratempu() {
  currentdata.pitch = 0;
  currentdata.yaw = 0;
  currentdata.roll = 0;
  currentdata.absvel = 0;
  currentdata.vel_x = 0;
  currentdata.vel_y = 0;
  currentdata.vel_z = 0;

  int targetiteratinos = 400;
  float accelvalues[3];
  float gyrovalues[3];

  for (int i = 0; i < targetiteratinos; i++)
  {
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);

    accelvalues[0] += a.acceleration.x;
    accelvalues[1] += a.acceleration.y;
    accelvalues[2] += a.acceleration.z;

    gyrovalues[0] += g.gyro.x;
    gyrovalues[1] += g.gyro.y;
    gyrovalues[2] += g.gyro.z;

    delay(25);

  }
  accelvalues[0] = accelvalues[0]/targetiteratinos;
  accelvalues[1] = accelvalues[1]/targetiteratinos;
  accelvalues[2] = accelvalues[2]/targetiteratinos;

  Serial.print(accelvalues[0],7);
  Serial.print(",");
  Serial.print(accelvalues[1],7);
  Serial.print(",");
  Serial.print(accelvalues[2],7);
  Serial.println("\t");

  gyrovalues[0] = gyrovalues[0]/targetiteratinos;
  gyrovalues[1] = gyrovalues[1]/targetiteratinos;
  gyrovalues[2] = gyrovalues[2]/targetiteratinos;

  Serial.print(gyrovalues[0],7);
  Serial.print(",");
  Serial.print(gyrovalues[1],7);
  Serial.print(",");
  Serial.print(gyrovalues[2],7);
  Serial.println("\t");

}
*/

/*
  for (int i = 0; i < 3; i++)
  {
    canards[i].setPeriodHertz(50);
    canards[i].attach(canardpins[i], 500, 2400);
    canards[i].write(canardsdefualtpos[i]);
  }
  */


 /*
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

  message.pitch = data.pitch;
  message.yaw = data.yaw;
  message.roll = data.roll;

  message.baro_alt = data.baro_alt;
  message.baro_pressure = data.baro_pressure;

  message.imu_temp = data.imu_temp;
  message.baro_temp = data.baro_temp;
  
  message.canardstatusx1 = canardpos[0];
  message.canardstatusx2 = canardpos[1];

  message.canardstatusy1 = canardpos[2];
  message.canardstatusy2 = canardpos[3];
  
  message.command = grounddata.command;
  */
  /*
    String datatobelogged = "101," + String(uptimemillis) + "," + String(missiontimemillis) + "," + String(state) + "," + 
    String(data.adjaccel_x*decimals) + "," + String(data.adjaccel_y*decimals) + "," + String(data.adjaccel_z*decimals) + "," + 
    String(data.gyro_x*decimals) + "," + String(data.gyro_y*decimals) + "," + String(data.gyro_z*decimals) + "," + 
    String(data.pitch*decimals) + "," + String(data.yaw*decimals) + "," + String(data.roll*decimals) + "," + 
    String(data.baro_alt*decimals) + "," + String(data.baro_pressure*decimals) + "," + String(data.baro_temp*decimals) + "," + 
    String(data.imu_temp*decimals) + "," + String(commandbuf*decimals) + "," + 
    String(data.rawaccel_x*decimals) + "," + String(data.rawaccel_y*decimals) + "," + String(data.rawaccel_z*decimals) + "," +
    String(data.rawpitch*decimals) + "," + String(data.rawyaw*decimals) + "," + String(data.rawroll*decimals) + "," + 
    String(battvoltage*decimals)+ "," + 
    String(data.adjabsaccel*decimals) + "," + String(data.rawabsaccel*decimals) + "," + String(data.vertical_vel*decimals)
    ;
    */
   /*
  decimals = pow(10,decimals);
  datatobelogged[0] = 10101;
  datatobelogged[1] = uptimemillis;
  datatobelogged[2] = missiontimemillis;
  datatobelogged[3] = state;
  datatobelogged[4] = data.adjaccel_x*decimals;
  datatobelogged[5] = data.adjaccel_y*decimals;
  datatobelogged[6] = data.adjaccel_z*decimals;
  datatobelogged[7] = data.gyro_x*decimals;
  datatobelogged[8] = data.gyro_y*decimals;
  datatobelogged[9] = data.gyro_z*decimals;
  datatobelogged[10] = data.pitch*decimals;
  datatobelogged[11] = data.yaw*decimals;
  datatobelogged[12] = data.roll*decimals;
  datatobelogged[13] = data.baro_alt*decimals;
  datatobelogged[14] = data.baro_pressure*decimals;
  datatobelogged[15] = data.baro_temp*decimals;
  datatobelogged[16] = data.imu_temp*decimals;
  datatobelogged[17] = commandbuf*decimals;
  datatobelogged[18] = data.rawaccel_x*decimals;
  datatobelogged[19] = data.rawaccel_y*decimals;
  datatobelogged[20] = data.rawaccel_z*decimals;
  datatobelogged[21] = data.rawpitch*decimals;
  datatobelogged[22] = data.rawyaw*decimals;
  datatobelogged[23] = data.rawroll*decimals;
  datatobelogged[24] = battvoltage*decimals;
  datatobelogged[25] = data.adjabsaccel*decimals;
  datatobelogged[26] = data.rawabsaccel*decimals;
  datatobelogged[27] = data.vertical_vel*decimals;
  datatobelogged[28] = 20202;
  


  for (int i = 0; i < datapoints; i++)
  {
    file.print(char(datatobelogged[i]));
  }
  */

 /*
      if (detectiontries >= 4){
        state = 3;
        detectiontries = 0;
      }
      else if (currentdata.rawabsaccel > 15)
      {
        detectiontries++;
      }
      
      else{detectiontries = 0;}
      */
}
