int switchState = 0; // Holds the state of the Switch (HIGH - Pressed, LOW - Floating)
void setup(){   // The void setup function runs only once at the beginning. It is the place to define the OUTPUTs and INPUTs
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT); 
// configures pin 3 (digital) as an OUTPUT
  pinMode(5, OUTPUT);
  pinMode(2, INPUT);
  Serial.begin(9600);
  digitalWrite(3,HIGH);
}
void loop() { // This function will execute endlessly from the first to the last row
  switchState=digitalRead(2);
  Serial.println(switchState);// reads the digital Value of pin 2
  if (switchState == 0) { 
    Serial.print("buttonlow");
// compares if the switchState corresponds to a LOW value. If the statement is true, it executes the orders contained within {}. Otherwise it ignores them. 
    digitalWrite(3, HIGH); // Sets the signal in 3 to HIGH (turn on the LED) and 4 and 5 to LOW (turn off the LEDs connected to 4 and 5)
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
  }
  else { // If the Switch is pressed, the program will ignore the 'if' conditional and enter the 'else'
    digitalWrite(3, LOW);
    digitalWrite(4, LOW); 
    digitalWrite(5, HIGH);
    delay(250); // Delays time for 250ms - a quarter of a second. It freezes the program during this time.
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    delay(250);
  }
}
