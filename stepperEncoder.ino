int inPinA = 0; 
int inPinB = 1; 

volatile float encoderCount = 0; 

void setup() {
  // put your setup code here, to run once:
  pinMode(inPinA, INPUT); 
  pinMode(inPinB, INPUT); 

  Serial.begin(9600); 
  
  attachInterrupt(digitalPinToInterrupt(inPinA), ISRPinA, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(inPinB), ISRPinB, CHANGE); 
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Encoder count:"); 
  Serial.println(encoderCount); 
}

void ISRPinA()
{
  encoderCount++; 
}

void ISRPinB()
{
  encoderCount++;
}

