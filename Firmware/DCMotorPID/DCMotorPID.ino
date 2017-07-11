////////////////////////////////////////////////////////////
// DC Motor PID Controller Code
// Version 1, July 10
// Jordan Geurten
////////////////////////////////////////////////////////////

#include <math.h>

// Motor pins

#define stopPin   0
#define startPin  1
#define encoder   2
#define voltage   3
#define motor     5

//Desired speed:
#define SPEED     30    //in RPM

//Global variables:

volatile byte Go = 0;                      //Interrupt flag for START
volatile byte Stop = 0;                    //Interrup flag for STOP
volatile double Error = 0;                 //Speed error
volatile double KpOutput = 0;              //Proportional output
volatile double KiOutput = 0;              //Integral output
volatile double KdOutput = 0;              //Derivative output

volatile unsigned long edgeCount = 0;      //Encoder edge count
volatile double motorSpeed = 0;            //Motor speed in RPM

//User defined variables:

double Kp = 0;                             //Proportional gain
double Ki = 0;                             //Integral gain
double Kd = 0;                             //Derivative gain

//ISR to increment count of encoder to count its square wave frequency
void encoderISR()
{
  edgeCount++;
}

//Every second, timer overflows and the speed of the motor is calculated. Encoder count is reset.
ISR(TIMER1_OVF_vect) {
  motorSpeed = (double) edgeCount * 60 / (2 * 16 * 50); // multiply by the gear ratio (50:1) divided by 1 second [revolutions/minute]
  edgeCount = 0;
  
  calculateError(); 
  TCNT1 = 0x85EE; //restart timer with value of 34286 to give 1Hz
}

//Function to stop motor:
void StopMotor()
{
  analogWrite(motor, 0);
  digitalWrite(voltage, LOW);
}

//Interrupt function to start system:
void START()
{
  Stop = 0; 
  Go = 1; 
}

//Interrupt function to stop system:
void STOP ()
{
  Stop = 1; 
  Go = 0; 
  StopMotor();
}

//call during Timer1 OVF ISR

void calculateError()
{
Error = desiredSpeed - motorSpeed;
if(Kp > 0)
{
  KpOutput = Kp*Error;
  ControllerOutput = KpOutput;
}

if (Kd > 0)
{
  KdOutput = Kd*(Error-prevError)*Kdfreq;
  ControllerOutput += KdOutput;
}
  
if(Ki > 0)
{
  Integral += Error*Period;
   //Period =1s
   KiOutput = Ki*Integral;
  ControllerOutput += KiOutput;
}

  prevError = Error;
  ControllerOutput *= VtoPWM;
  //VtoPWM = 255/V_in 
}

void RunMotor(float ControllerOutput)
{
  if(!STOP && Go)
  {

    ControllerOutput =abs(ControllerOutput);
    if(ControllerOutput > 255)
       ControllerOutput = 255;

    analogWrite(motor, ControllerOutput);
}

void OpenLoopStep()
{
  ControllerOutput = Ref_Input;
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
  TCNT1 = 0x85EE; //since using 8MHz clock, arduino example is doubled to give 1Hz
  TCCR1B |= (1 << CS12);
  sei();

  attachInterrupt(digitalPinToInterrupt(encoder), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder), encoderISR, CHANGE);

  analogWrite(motor, 120);
}

void loop()
{

}



