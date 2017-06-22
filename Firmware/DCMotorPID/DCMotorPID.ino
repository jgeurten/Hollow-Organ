#define encoder 2
#define motor 5
#define voltage 3
#define pi 3.1415

volatile unsigned long edgeCount;
volatile unsigned long motorW;

void encoderISR()
{
  edgeCount++;
}

ISR(TIMER3_COMPA_vect){
  motorW = edgeCount / (2 * 16); //divided by 1 second [revolutions/second]
  edgeCount = 0;

  Serial.print("Speed:"); 
  Serial.println(motorW); 
}

void setup()
{
  Serial.begin(9600); 
  pinMode(encoder, INPUT);
  pinMode(motor, OUTPUT);
  pinMode(voltage, OUTPUT);
  digitalWrite(voltage, HIGH);

  cli();
  TCCR3A = 0; //change number 3 to 1 incase of non functioning
  TCCR3B = 0;
  OCR3A = 15624; //#timer counts + 1 = 1s/6.4e-5  [target time/timer res]

  TCCR3B |= (1 << WGM12); //set CTC mode
  //bit shifting for 1024 prescalar
  TCCR3B |= (1<< CS10); 
  TCCR3B |= (1<< CS12);    
  TIMSK3 |= (1 << OCIE3A);

  sei();
  attachInterrupt(digitalPinToInterrupt(encoder), encoderISR, CHANGE);
}

void loop()
{
  analogWrite(motor, 200);
}

