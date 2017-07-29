//Pin defintions:

#define  ENCA     0  //encoder pin1
#define  ENCB     1  //encoder pin2
#define  DIR      2
#define  STEP     3
#define  SLEEP    5
#define  RST      6
#define  MS3      9
#define  MS2      10
#define  MS1      11
#define  ENABLE   12

volatile long count = 0;
int x;

void encoderISR_A()
{
  count++; 
}

void encoderISR_B()
{
  count++; 
}
void setup() {
  //Motor output pins:
  pinMode(ENABLE, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(SLEEP, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);

  //Encoder input pins:
  pinMode(ENCA, INPUT); 
  pinMode(ENCB, INPUT); 
  
  //initialize sixteenth step
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);

  digitalWrite(DIR, LOW); //Pull pin low to move "forward"
  digitalWrite(MS1, HIGH); //Set MS1,MS2, and MS3 high to set logic to 1/16th microstep resolution
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);

  Serial.begin(9600);

  //Encoder interrupts:
  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR_A, RISING); 
  attachInterrupt(digitalPinToInterrupt(ENCB), encoderISR_B, RISING); 
}

void loop() {
  
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(STEP,HIGH); //Trigger one step forward
    delayMicroseconds(125);
    digitalWrite(STEP,LOW); //Pull step pin low so it can be triggered again
    delayMicroseconds(125);
  }
}
