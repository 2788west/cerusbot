#include <CytronMotorDriver.h>

// Configure the motor driver
CytronMD motor1(PWM_DIR, 5, 6);  // PWM = Pin 5, DIR = Pin 6
CytronMD motor2(PWM_DIR, 7, 8);  // PWM = Pin 7 DIR = Pin 8

// Configure the encoder pins
#define CHA 2
#define CHB 3

// Set defaults
const byte numChars = 5; //command from jetson (5 bytes)
char receivedChars[numChars];
char moveCommand;
boolean newData = false;
volatile int master_count = 0; //universial count
volatile byte INTFLAG1 = 0; //interrupt status flag


// Setup
void setup() {
  Serial.begin(9600);

  pinMode(CHA, INPUT);
  pinMode(CHB, INPUT);

  attachInterrupt(0, flag, RISING);
  //interrupt 0 digital pin 2 positive edge trigger

}


// Loop
void loop() {
  handleSerial();
  runMotors();

  if(INTFLAG1) {
    Serial.println(master_count);
    delay(100);
    INTFLAG1 = 0; //clear flag
  }

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
void runMotors() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);

        char moveCommand = receivedChars[0];
        Serial.print("Movement command: ");
        Serial.println(moveCommand);

        char velocityString[] = {receivedChars[1], receivedChars[2], receivedChars[3], '\0'};
        unsigned int velocity;
        sscanf(velocityString, "%d", &velocity); 
        Serial.print("Speed: ");
        Serial.println(velocity);
    
        newData = false;

        switch(moveCommand) {
          case 'W':
          motor1.setSpeed(velocity);
          motor2.setSpeed(velocity);
          Serial.print("Running motor forward at speed ");
          Serial.print(velocity);
          Serial.println();
          delay(10);
          break;

          case 'S':
          motor1.setSpeed(-velocity);
          motor2.setSpeed(-velocity);
          Serial.print("Running motor backwards at speed ");
          Serial.print(velocity);
          Serial.println();
          delay(10);
          break;
    
    }
  }
} 

void flag() {
  INTFLAG1 = 1;
  //add 1 to count for CW rotation
  if(digitalRead(CHA) && !digitalRead(CHB)) {
    master_count++;

  }
  //subtract 1 from count for CCW rotation
  if(digitalRead(CHA) && digitalRead(CHB)){
    master_count--;

  }
}
    
    
  
  



      
    
    
