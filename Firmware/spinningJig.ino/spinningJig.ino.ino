#define  ENABLE   2
#define  RST      3
#define  SLEEP    5
#define  MS1      6
#define  MS2      10
#define  MS3      11
#define  STEP     12
#define  DIR      13

int count = 0;
int x;

void setup() {
  pinMode(ENABLE, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(SLEEP, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  
  //initialize sixteenth step
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);

  Serial.begin(9600);

}

void loop() {
  
  // put your main code here, to run repeatedly:
  digitalWrite(DIR, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, HIGH); //Pull MS1,MS2, and MS3 high to set logic to 1/16th microstep resolution
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);
  
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(STEP,HIGH); //Trigger one step forward
    delayMicroseconds(125);
    digitalWrite(STEP,LOW); //Pull step pin low so it can be triggered again
    delayMicroseconds(125);
  }
}
