#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
//#include <BluetoothSerial.h>

//BluetoothSerial SerialBT;

struct datatotransmit
{
  int state;
  unsigned long uptimemillis;
  //unsigned long missiontimemillis;

  float roll;
  float pitch;
  float yaw;

  float roll_rate;
  float pitch_rate;
  float yaw_rate;

  float accel_x;
  float accel_y;
  float accel_z;

  //float imu_alt;
  float imu_velocity;

  //float vertical_velocity;

  float baro_alt;

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
};

prevmillis prevmilliss;
datatotransmit telemetry;
datanew grounddata;
 
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

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 init");
  /*
  SerialBT.begin("ESP32");

  Serial.println("Bluetooth init");
  while (!SerialBT.available()) {
  
    Serial.println("Bluetooth not ready");
    delay(500);
  }
  */

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
  /*
  if (prevmilliss.uptimemillis - prevmilliss.bluetoothprevmillis > 200)
  {
    Serial.println("bluetooth ");
    prevmilliss.bluetoothprevmillis = prevmilliss.uptimemillis;
  }
  */
  if (Serial.available() > 0)
  {
    incomingbyte = Serial.read();
    Serial.print(" Echo: ");
    Serial.println(incomingbyte);
    switch (incomingbyte)
    {
    case 109:
      grounddata.command = 107;
      broadcast(grounddata);
      break;
    
    default:
      break;
    }
  }

  if (prevmilliss.uptimemillis - prevmilliss.serialprevmillis > 100)
  {
    Serial.print("101,");
    Serial.print(prevmilliss.uptimemillis);
    Serial.print(",");
    Serial.print(telemetry.uptimemillis);
    Serial.print(",");
    Serial.print(prevmilliss.uptimemillis-prevmilliss.telemetryprevmillis);
    Serial.print(",");
    Serial.print(telemetry.state);
    Serial.print(",");
    Serial.print(telemetry.accel_x);
    Serial.print(",");
    Serial.print(telemetry.accel_y);
    Serial.print(",");
    Serial.print(telemetry.accel_z);
    Serial.print(",");
    Serial.print(telemetry.roll_rate);
    Serial.print(",");
    Serial.print(telemetry.pitch_rate);
    Serial.print(",");
    Serial.print(telemetry.yaw_rate);
    Serial.print(",");
    Serial.print(telemetry.pitch);
    Serial.print(",");
    Serial.print(telemetry.roll);
    Serial.print(",");
    Serial.print(telemetry.yaw);
    Serial.print(",");
    Serial.print(telemetry.baro_alt);
    Serial.print(",");
    Serial.print(telemetry.imu_velocity);
    Serial.println("");
    prevmilliss.serialprevmillis = prevmilliss.uptimemillis;
  }
  
  
}