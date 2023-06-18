#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
//#include <BluetoothSerial.h>

//BluetoothSerial SerialBT;

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

struct prevmillis{
  unsigned long uptimemillis;
  unsigned long bluetoothprevmillis;
  unsigned long serialprevmillis;
  unsigned long telemetryprevmillis;
  unsigned long launchprevmillis;
};

prevmillis prevmilliss;
datatotransmit telemetry;
datanew grounddata;
 
int pyropin1 = 25;
int pyropin2 = 26;


void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
   
  datatotransmit* telemetrytemp =(datatotransmit*) data;
  prevmilliss.telemetryprevmillis = prevmilliss.uptimemillis;
  telemetry = *telemetrytemp;
}
 
void broadcast(const datanew &message)
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
}

void launch(){
  grounddata.command = 108;
  broadcast(grounddata);
  Serial.print("launching");
  prevmilliss.launchprevmillis = prevmilliss.uptimemillis;
  digitalWrite(pyropin1, HIGH);
  digitalWrite(pyropin2, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 init");

  pinMode(pyropin1, OUTPUT);
  pinMode(pyropin2, OUTPUT);

  digitalWrite(pyropin1, LOW);
  digitalWrite(pyropin2, LOW);
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  esp_now_register_recv_cb(OnDataRecv);
  WiFi.disconnect();
}


 
void loop() {
  byte incomingbyte;
  prevmilliss.uptimemillis = millis();
  if (Serial.available() > 0)
  {
    incomingbyte = Serial.read();
    Serial.print(" Echo: ");
    Serial.println(incomingbyte);
    switch (incomingbyte)
    {
    case 109:
      grounddata.command = 109;
      broadcast(grounddata);
      Serial.print("calibrating");
      break;
    
    case 108:
      launch();

      break;
    default:
      break;
    }
  }

  if (prevmilliss.uptimemillis - prevmilliss.serialprevmillis > 50)
  {
    int dataage = prevmilliss.uptimemillis-prevmilliss.telemetryprevmillis;
    if (dataage > 7000)
    {
      telemetry.state = -1;
    }
    
    Serial.print("101,");//0
    Serial.print(prevmilliss.uptimemillis);//1
    Serial.print(",");
    Serial.print(telemetry.uptimemillis);//2
    Serial.print(",");
    Serial.print(dataage);//3
    Serial.print(",");
    Serial.print(telemetry.state);//4
    Serial.print(",");
    Serial.print(telemetry.accel_x);//5
    Serial.print(",");
    Serial.print(telemetry.accel_y);//6
    Serial.print(",");
    Serial.print(telemetry.accel_z);//7
    Serial.print(",");
    Serial.print(telemetry.roll_rate);//8
    Serial.print(",");
    Serial.print(telemetry.pitch_rate);//9
    Serial.print(",");
    Serial.print(telemetry.yaw_rate);//10
    Serial.print(",");
    Serial.print(telemetry.pitch);//11
    Serial.print(",");
    Serial.print(telemetry.roll);//12
    Serial.print(",");
    Serial.print(telemetry.yaw);//13
    Serial.print(",");
    Serial.print(telemetry.baro_alt);//14
    Serial.print(",");
    Serial.print(telemetry.absvel);//15
    Serial.print(",");
    Serial.print(telemetry.vel_x);//16
    Serial.print(",");
    Serial.print(telemetry.vel_y);//17
    Serial.print(",");
    Serial.print(telemetry.vel_z);//18
    Serial.print(",");
    Serial.print(telemetry.vertical_vel);//19
    Serial.print(",");
    Serial.print(telemetry.absaccel);//20
    Serial.print(",");
    Serial.print(telemetry.batteryvolt);//21
    Serial.println("");
    prevmilliss.serialprevmillis = prevmilliss.uptimemillis;
  }
  if (prevmilliss.uptimemillis - prevmilliss.launchprevmillis > 200)
  {
    digitalWrite(pyropin1, LOW);
    digitalWrite(pyropin2, LOW);
  }
  
  
  
}