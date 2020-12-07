 /********************************************************************************
 * DESCRIPTION:
 *
 * This code is inteded for the Arduino MEGA 2560. It receives data from a Jetson 
 * Nano via a serial connection and controls the motors of the Cerus mobile robot 
 * platform via two Cytron motor drivers. It is based on the incredible work 
 * of Daniel Snider (https://github.com/danielsnider) and Robin2 
 * (https://forum.arduino.cc/index.php?topic=396450.0). Please don't hesitate to
 * email me with suggestions or questions.
 * 
 * AUTHOR   : Johan Schwind
 * WEBSITE  : www.johanschwind.xyz
 * EMAIL    : info@johanschwind.com
 *
 *******************************************************************************/

#include "CytronMotorDriver.h"

// Define right and left encoder channels

#define RCHA 21
#define RCHB 20
#define LCHA 19
#define LCHB 18

// Configure the cytron motor driver

// Left motors
CytronMD motorFL(PWM_DIR, 3, 2);  // PWM 1 = Pin 3, DIR 1 = Pin 2
CytronMD motorRL(PWM_DIR, 5, 4); // PWM 2 = Pin 4, DIR 2 = Pin 5

// Right motors
CytronMD motorFR(PWM_DIR, 9, 8);  // PWM 1 = Pin 9, DIR 1 = Pin 8
CytronMD motorRR(PWM_DIR, 11, 10); // PWM 2 = Pin 11, DIR 2 = Pin 10

// Max reverse and forward speed of the Cytron motor driver
int cytronMin = -255;
int cytronMax = 255; 

// Variables for serial communication
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; // temporary array for use by strtok() function
boolean newData = false;

// Variables to hold the parsed data
float linearVelocity = 0.0;
float angularVelocity = 0.0;

// Encoder variables

volatile int right_count = 0; //right encoder count
volatile int left_count = 0; //left encoder count
volatile byte INTFLAG1 = 0; //interrupt status flag
volatile byte INTFLAG2 = 0; //second interrupt status flag


//============

void setup() {
    pinMode(RCHA, INPUT);
    pinMode(RCHB, INPUT);
    pinMode(LCHA, INPUT);
    pinMode(RCHA, INPUT);

    attachInterrupt(2, flag1, RISING);
    attachInterrupt(4, flag2, RISING);
    
    Serial.begin(115200);
    
    Serial.print(left_count);
    Serial.print(",");
    Serial.println(right_count);
    Serial.print(left_count);
    Serial.print(",");
    Serial.println(right_count);  
}


//============

void loop() {
    
    recvWithStartEndMarkers(); //look for serial data
    
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() replaces the commas with \0
        parseData();
        // showParsedData();
        runMotors();
        newData = false;
    }

    //check if the encoder flags have been raised by the hardware interrupts
    if(INTFLAG1) {
      Serial.print("<");
      Serial.print(left_count);
      Serial.print(",");
      Serial.print(right_count);
      Serial.println(">");      
      delay(50);
      INTFLAG1 = 0; //clear flag
      
    }
    if(INTFLAG2) {
      Serial.print("<");
      Serial.print(left_count);
      Serial.print(",");
      Serial.print(right_count);
      Serial.println(">");
      delay(50);
      INTFLAG2 = 0; //clear flag
      
  }
}


//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}


//============

void parseData() {

    // split the data into its parts
    char * strtokIndx; // this is used by strtok() as an index
   
    strtokIndx = strtok(tempChars, ",");   
    linearVelocity = atof(strtokIndx);     // convert to float

    strtokIndx = strtok(NULL, ",");
    angularVelocity = atof(strtokIndx);    // convert to float

}

//============

void showParsedData() {
   
    Serial.print("Linear Velocity ");
    Serial.println(linearVelocity);
    Serial.print("Angular Velocity ");
    Serial.println(angularVelocity);
}

void runMotors () {
  
  // For a differential drive, the sum of linear and angular velocity cannot be greater than 1
  if (abs(linearVelocity) + abs(angularVelocity) > 1.0) {
    float temp = abs(linearVelocity) + abs(angularVelocity);
    linearVelocity = linearVelocity / temp;
    angularVelocity = angularVelocity / temp;    
  }
 
  // Convert float values to an integer for left and right wheels
  int leftSpeed = (linearVelocity + angularVelocity) * 100;
  int rightSpeed = (linearVelocity - angularVelocity) * 100;


  // Map speed to left motors
  motorFL.setSpeed(map(leftSpeed, 100, -100, cytronMin, cytronMax));
  motorRL.setSpeed(map(leftSpeed, 100, -100, cytronMin, cytronMax));

  // Map speed to right motors
  motorFR.setSpeed(map(rightSpeed, 100, -100, cytronMin, cytronMax));
  motorRR.setSpeed(map(rightSpeed, 100, -100, cytronMin, cytronMax));
  
  /*
  //Mecanum mapping
  // Map speed to left motors
  motorFL.setSpeed(map(leftSpeed, -100, 100, cytronMin, cytronMax));
  motorRL.setSpeed(map(-leftSpeed, -100, 100, cytronMin, cytronMax));

  // Map speed to right motors
  motorFR.setSpeed(map(-rightSpeed, -100, 100, cytronMin, cytronMax));
  motorRR.setSpeed(map(rightSpeed, -100, 100, cytronMin, cytronMax));
  */
}

//flag for the right encoder; CCW is forward
void flag1() {
  INTFLAG1 = 1;
  //add 1 to count for CCW rotation
  if(digitalRead(RCHA) && digitalRead(RCHB)) {
    right_count++;

  }
  //subtract 1 from count for CW rotation
  if(digitalRead(RCHA) && !digitalRead(RCHB)){
    right_count--;

  }
}
//flag for the left encoder; CW is forward
void flag2() {
  INTFLAG2 = 1;
  //add 1 to count for CW rotation
  if(digitalRead(LCHA) && !digitalRead(LCHB)) {
    left_count++;

  }
  //subtract 1 from count for CCW rotation
  if(digitalRead(LCHA) && digitalRead(LCHB)){
    left_count--;

  }
}
