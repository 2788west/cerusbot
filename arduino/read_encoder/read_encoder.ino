#define CHA 2
#define CHB 3

volatile int master_count = 0; //universial count
volatile byte INTFLAG1 = 0; //interrupt status flag

void setup() {
  pinMode(CHA, INPUT);
  pinMode(CHB, INPUT);

  Serial.begin(9600);
  Serial.println(master_count);

  attachInterrupt(0, flag, RISING);
  //interrupt 0 digital pin 2 positive edge trigger
}

void loop() {
  if(INTFLAG1) {
    Serial.println(master_count);
    delay(500);
    INTFLAG1 = 0; //clear flag
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
