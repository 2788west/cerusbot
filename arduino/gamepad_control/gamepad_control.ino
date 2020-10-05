//This script accepts motor control data from a PC in the following format: 
//<motor1_speed, motor2_speed, motor3_speed, motor4_speed>

#include <CytronMotorDriver.h>

// Configure the motor driver
CytronMD motor1(PWM_DIR, 5, 4);  // PWM = Pin 5, DIR = Pin 4
CytronMD motor2(PWM_DIR, 6, 7);  // PWM = Pin 6 DIR = Pin 7

// Configure serial communication variables
const byte numChars = 18;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

int motor1_speed = 0;
int motor2_speed = 0;
int motor3_speed = 0;
int motor4_speed = 0;


boolean newData = false;

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
            // because strtok() used in parseData() replaces the commas with \0
        parseData();
        //showParsedData();
        newData = false;
    }

    runMotors();
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

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    motor1_speed = atoi(strtokIndx); // copy it to messageFromPC
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    motor2_speed = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    motor3_speed = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    motor4_speed = atof(strtokIndx);     // convert this part to a float

}

//============

void showParsedData() {
    Serial.print("Motor 1 Speed: ");
    Serial.println(motor1_speed);
    Serial.print("Motor 2 Speed: ");
    Serial.println(motor2_speed);
    Serial.print("Motor 3 Speed: ");
    Serial.println(motor3_speed);
    Serial.print("Motor 4 Speed: ");
    Serial.println(motor4_speed);
}

//============
//This motor control only runs motors forward for now
void runMotors() {
  motor1.setSpeed(motor1_speed);
  motor2.setSpeed(motor2_speed);
}
