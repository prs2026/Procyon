#include <Arduino.h>

#define TRYIMU

//#define TRYBMP

//#define TRYFLASH

#include <Wire.h>

#if defined(TRYIMU)
  #include <BMI088.h>

  /* accel object */
  Bmi088Accel accel(Wire,0x18);
  /* gyro object */
  Bmi088Gyro gyro(Wire,0x68);

#endif // TRYIMU

#if defined(TRYBMP)
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BMP3XX.h>

  Adafruit_BMP3XX BMP;
#endif // TRYBMP

#if defined(TRYFLASH)
  #include <SPIFlash.h>

  SPIFlash flash(FLASHCS);
#endif // TRYFLASH



#define REDLED PA3
#define GREENLED PA2
#define BLUELED PA1

#define BUZZERPIN PB0

#define FLASHCS PA4

#define RED 0
#define GREEN 1
#define BLUE 2
#define YELLOW 3
#define PURPLE 4

HardwareSerial Seria(PA10,PA9);






#define SCANI2C



int errorflag = 1;








void setled(int color){
  switch (color)
  {
  case 0:
    digitalWrite(REDLED,HIGH);
    digitalWrite(GREENLED,LOW);
    digitalWrite(BLUELED,LOW);
    break;
  
  case 1:
    digitalWrite(REDLED,LOW);
    digitalWrite(GREENLED,HIGH);
    digitalWrite(BLUELED,LOW);
    break;
  
  case 2:
    digitalWrite(REDLED,LOW);
    digitalWrite(GREENLED,LOW);
    digitalWrite(BLUELED,HIGH);
    break;
  
  case 3:
    digitalWrite(REDLED,HIGH);
    digitalWrite(GREENLED,HIGH);
    digitalWrite(BLUELED,LOW);
    break;
  
  case 4:
    digitalWrite(REDLED,HIGH);
    digitalWrite(GREENLED,LOW);
    digitalWrite(BLUELED,HIGH);
    break;
  
  default:
    break;
  }
}

void setup() {
  Seria.begin(9600);
  Seria.println("restart");
  pinMode(BUZZERPIN,OUTPUT);
  pinMode(REDLED,OUTPUT);
  pinMode(GREENLED,OUTPUT);
  pinMode(BLUELED,OUTPUT);
  pinMode(FLASHCS,OUTPUT);
  digitalWrite(REDLED,LOW);
  digitalWrite(GREENLED,LOW);
  digitalWrite(BLUELED,LOW); 
  setled(RED);
  #ifdef TRYIMU
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();
  #endif // TRYIMU || TRYBMP
  
  
  #ifdef TRYFLASH
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    SPI.setSCLK(PA5);
    SPI.begin();

    flash.begin();
    if (flash.getCapacity()>0)
    {
      Seria.println("Flash Init Success");
    }
    else
    {
      Seria.println("Flash init failure, cry");
      errorflag = errorflag * 3;
    }
  #endif // TRYFLASH
  

  


  
  #ifdef TRYIMU
  int status;
  status = accel.begin();
  if (status < 0)
  {
    Seria.print("Accelerometer init failure, code:");
    Seria.println(status);
    Wire.end();
    Wire.begin();
    errorflag = errorflag * 5;
  }
  else
  {
    Seria.println("Accelerometer init success");
  }
  
  
  status = gyro.begin();
  if (status < 0)
  {
    Seria.print("Gyro init failure, code:");
    Seria.println(status);
    Wire.end();
    Wire.begin();
    errorflag = errorflag * 7;
  }
  else
  {
    Seria.println("Gyro init success");
  }
  #endif // TRYIMU

  #ifdef TRYBMP
  if (BMP.begin_I2C())
  {
    Seria.println("BMP init success");
  }
  else{
    Seria.println("BMP init failure,cry harder");
  }
  #endif

  tone(BUZZERPIN,500,100);
  
  
  // put your setup code here, to run once:
  tone(BUZZERPIN,700,100);
  delay(100);
  tone(BUZZERPIN,700,100);
  Seria.println("init");

  #if defined(SCANI2C)
  byte error, address;
  int nDevices;
 
  Seria.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Seria.print("I2C device found at address 0x");
      if (address<16)
        Seria.print("0");
      Seria.print(address,HEX);
      Seria.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Seria.print("Unknown error at address 0x");
      if (address<16)
        Seria.print("0");
      Seria.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Seria.println("No I2C devices found\n");
  else
    Seria.println("done\n");
  #endif // 
  if (errorflag==1)
  {
    setled(GREEN);
  }
  else{
    setled(PURPLE);
    tone(BUZZERPIN,1000,100);
    delay(200);
    tone(BUZZERPIN,1000,100);
  }
  Seria.print("errorflag=");
  Seria.println(errorflag);
  

 
}

void loop() {
  delay(1000);
  //tone(BUZZERPIN,1000,100);
  Seria.print(".");
 
}
