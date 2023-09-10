#include Wire.h

void requestevent(){
  Wire.write(0xAD);
  
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin(0x10)
  Wire.onRequest(requestevent);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100)
}
