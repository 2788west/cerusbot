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
char tempChars[numChars];        // temporary array for use by strtok() function
boolean newData = false;

// Variables to hold the parsed data
float linearVelocity = 0.0;
float angularVelocity = 0.0;


//============

void setup() {
    Serial.begin(115200);
}


//============

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() replaces the commas with \0
        parseData();
        // showParsedData();
        runMotors();
        newData = false;
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

  

  // For testing only
  /*
  Serial.print("Left Speed: ");
  Serial.println(leftSpeed);
  Serial.print("Right Speed: ");
  Serial.println(rightSpeed);
  */  
  
  // Map speed to left motors
  motorFL.setSpeed(map(leftSpeed, -100, 100, cytronMin, cytronMax));
  motorRL.setSpeed(map(leftSpeed, -100, 100, cytronMin, cytronMax));

  // Map speed to right motors
  motorFR.setSpeed(map(rightSpeed, -100, 100, cytronMin, cytronMax));
  motorRR.setSpeed(map(rightSpeed, -100, 100, cytronMin, cytronMax));
  
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
