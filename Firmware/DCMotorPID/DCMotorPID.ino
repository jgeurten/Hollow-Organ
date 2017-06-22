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

ISR(TIMER1_OVF_vect) {
  motorW = edgeCount / (2 * 16); //divided by 1 second [revolutions/minute]
  edgeCount = 0;
  Serial.println(motorW); 
  TCNT1=0x85EE; //restart timer with value of 34286 to give 1Hz
}

void setup()
{
  Serial.begin(9600);
  pinMode(encoder, INPUT);
  pinMode(motor, OUTPUT);
  pinMode(voltage, OUTPUT);
  digitalWrite(voltage, HIGH);

  cli();
  TCCR1A = 0; 
  TCCR1B = 0;
  TIMSK1 |= (1 << TOIE1);
  TCNT1=0x85EE; //since using 8MHz clock, arduino example is doubled to give 1Hz
  TCCR1B |= (1 << CS12);
  sei();
  
  attachInterrupt(digitalPinToInterrupt(encoder), encoderISR, CHANGE);
  analogWrite(motor, 200);
}

void loop()
{
  /*for (int i = 100; i <= 255; i++) {
    analogWrite(motor, i);
    Serial.print("Voltage:");
    Serial.println(i);
    delay(500);
  }*/
}

