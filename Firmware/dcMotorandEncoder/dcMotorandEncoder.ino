#define encoder 2   //interrupt digital 2 pin
#define motor 5    //digital pin 5 will write 0 - 255 to motor pin
#define pi 3.1415

volatile unsigned long motorW;
volatile unsigned long prevTime;
volatile unsigned long edgeCount;
volatile unsigned long theta;  

void setup() {
  // put your setup code here, to run once:
  pinMode(encoder, INPUT);
  pinMode(motor, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  Serial.begin(9600);
  Serial.println("Setup");
  attachInterrupt(digitalPinToInterrupt(encoder), getEncoderCount, CHANGE);
  prevTime = micros();    //get at beginning of execution rather than setting to 0.
}

void loop() {
  // put your main code here, to run repeatedly:

  analogWrite(motor, 255);
}

void getEncoderCount()
{
  //calculate the time between ISR entries:
  //counting one edge (rising) --> 16 counts per revolution
  

  edgeCount++; 
  theta = edgeCount*2*pi/(2*16);    

  if(theta >= 2*pi){

    motorW = theta*1E6/(micros()-prevTime);   //[rad/s]
    Serial.println(motorW);
    theta = 0;
    edgeCount = 0;  
    prevTime = micros();  
    
  }
}


