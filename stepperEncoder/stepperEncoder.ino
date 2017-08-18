int inPinA = 0; 
int inPinB = 1; 

volatile float encoderCount = 0; 
volatile double motorSpeed; 

void setup() {
  // put your setup code here, to run once:
  pinMode(inPinA, INPUT); 
  pinMode(inPinB, INPUT); 

  Serial.begin(9600); 

  cli(); 
  TCCR1A = 0; 
  TCCR1B = 0; 
  TCNT1 = 0x85EE; //since using 8MHz clock, arduino example is doubled to give 1Hz
  TCCR1B |= (1 << CS12);
  sei();
  
  attachInterrupt(digitalPinToInterrupt(inPinA), ISRPinA, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(inPinB), ISRPinB, CHANGE); 
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Speed:"); 
  Serial.println(motorSpeed); 
}

void ISRPinA()
{
  encoderCount++; 
}

void ISRPinB()
{
  encoderCount++;
}

ISR(TIMER1_OVF_vect){
  motorSpeed = (double)encoderCount*60/(2*16);  //   [revolutions/minute]
  encoderCount = 0;

  TCNT1 = 0x85EE; // 1 Hz (1 count per second)
}
