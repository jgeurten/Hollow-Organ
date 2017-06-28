#define encoder0PinA  2
#define motor 5    //digital pin 5 will write 0 - 255 to motor pin


volatile long encoder0Pos=0;
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
long vel;

void setup()
{
  pinMode(encoder0PinA, INPUT);
  
  pinMode(motor, OUTPUT);
  
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  
  
  
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, RISING);  // encoDER ON PIN 2
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
  analogWrite(motor, 255);
}

void loop()
{
newposition = encoder0Pos;
newtime = millis();
vel = (newposition-oldposition) * 1000 /(newtime-oldtime);
Serial.print ("speed = ");
Serial.println (vel);
oldposition = newposition;
oldtime = newtime;
delay(250);
}

void doEncoder()
{
  
    encoder0Pos++;
  
}
