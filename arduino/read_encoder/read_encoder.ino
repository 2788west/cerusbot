// Define right and left encoder channels
#define RCHA 21
#define RCHB 20
#define LCHA 19
#define LCHB 18

volatile int right_count = 0; //right encoder count
volatile int left_count = 0; //left encoder count
volatile byte INTFLAG1 = 0; //interrupt status flag
volatile byte INTFLAG2 = 0; //second interrupt status flag

void setup() {
  pinMode(RCHA, INPUT);
  pinMode(RCHB, INPUT);
  pinMode(LCHA, INPUT);
  pinMode(RCHA, INPUT);

  Serial.begin(115200);
  Serial.println(right_count);
  Serial.println(left_count);

  attachInterrupt(2, flag1, RISING);
  attachInterrupt(4, flag2, RISING);
}

void loop() {
  if(INTFLAG1) {
    Serial.print("Right Encoder: ");
    Serial.println(right_count);
    delay(50);
    INTFLAG1 = 0; //clear flag
  }
  if(INTFLAG2) {
    Serial.print("Left Encoder: ");
    Serial.println(left_count);
    delay(50);
    INTFLAG2 = 0; //clear flag
  }

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
