#include "TimerOne.h"

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

ISR(TIMER#_OVF_vect){
  cli();
  motorW = edgeCount / (2 * 16); //divided by 1 second
  edgeCount = 0;
  TCNT3 = 0x0BDC;
  sei(); 
}

void setup()
{
  pinMode(encoder, INPUT);
  pinMode(motor, OUTPUT);
  pinMode(voltage, OUTPUT);
  digitalWrite(voltage, HIGH);

  cli();
  TCCR3A = 0;
  TCCR3B = 0;

  TIMSK3 |= (1 << TOIE3);
  TCNT3 = 0x0BDC;
  TCCR3B |= (1 << CS12); // Sets bit CS12 in TCCR1B
  sei();
  attachInterrupt(digitalPinToInterrupt(encoder), encoderISR, CHANGE);
}

void loop()
{
  analogWrite(motor, 200); 
}

