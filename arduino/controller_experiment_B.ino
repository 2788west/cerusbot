#include "CytronMotorDriver.h"

// Configure the motor driver
CytronMD motor(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4

// Set defaults
const byte numChars = 5;
char receivedChars[numChars];

boolean newData = false;

// Setup
void setup() {
  Serial.begin(9600);
}


// Loop
void loop() {
  handleSerial();
  showNewData();
  convertData();
  }


//Handle serial communication from PC/Jetson
void handleSerial() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if(rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0';
      ndx = 0;
      newData = true;
    }
  }
}  


//Shows the data (Debugging only)
void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
    }
}


//Converts data to the correct format (Movement, Speed)
void convertData() {
  if (newData == true) {
    char moveCommand = receivedChars[0];
    Serial.print("Movement command: ");
    Serial.println(moveCommand);
    
    char velocityString[] = {receivedChars[1], receivedChars[2], receivedChars[3], '\0'};
    int velocity;
    sscanf(velocityString, "%d", &velocity); 
    Serial.print("Speed: ");
    Serial.println(velocity);
    newData = false;
  }  
}

void runMotors() {
  motor.setSpeed(velocity);
  delay(500); 
}
  



      
    
    
