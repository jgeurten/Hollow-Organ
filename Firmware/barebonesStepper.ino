#include <MsTimer2.h>

///////////////////////////////////////////////////////////////////////////////////////////
// DC Motor PID Controller Code
// Version 1, July 10
// Jordan Geurten
///////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <MsTimer2.h>

///////////////////////////////////////////////////////////////////////////////////////////
// Motor pins
///////////////////////////////////////////////////////////////////////////////////////////


#define  ENCA     0  //Interrupt encoder pin1
#define  ENCB     1  //Interrupt encoder pin2
#define  GOPIN    2  //Interrupt go button pin
#define  STOPPIN  3  //Interrupt Stop button pin
#define  SLEEP    5
#define  RST      6
#define  MS3      9
#define  MS2      10
#define  MS1      11
#define  ENABLE   12
#define  DIR      14  //miso
#define  STEP     16  //mosi pin (also digital)

unsigned int Motor_Current;               //Pin to read current input from motor controller. WAit, we dont have a motor controller.

//Global variables:

volatile byte Go = 0;                      //Interrupt flag for START
volatile byte Stop = 0;                    //Interrup flag for STOP
volatile double Error = 0;                 //Ref_Speed Error
volatile double prevError = 0;             //Prev Ref_Speed Error
volatile double KpOutput = 0;              //Proportional output
volatile double KiOutput = 0;              //Integral output
volatile double KdOutput = 0;              //Derivative output
volatile double Controller_Input = 0;       //Output from Error calc
volatile double VtoPWM = 0;                //Define relation between voltage and pwm
volatile double Integral = 0;              //Integral of the error

volatile unsigned long edgeCount = 0;      //Encoder edge count
volatile double Motor_Speed = 0;            //Motor Ref_Speed in RPM
volatile double Elapsed_Time = 0;           //Elapsed time of the test
volatile double goTime = 0;                 //time of last go button push
volatile double stopTime = 0;               //Time of last stop button push


///////////////////////////////////////////////////////////////////////////////////////////
//User defined variables:
///////////////////////////////////////////////////////////////////////////////////////////

double Kp = 0;                             //Proportional gain
double Ki = 0;                             //Integral gain
double Kd = 0;                             //Derivative gain

//Define input voltage
float V_in  = 12.0;

//Timer timeout period in ms
double Period = 50.0;

//Desired Speed [in RPM]
static unsigned int Ref_Speed =  30;

//Open loop variables [in RPM]
volatile float Ref_Input = 0;

//Open loop variable - step input [PWM]
volatile float Step_Input = 0;

//Duration of the test [in seconds]:
volatile float Time = 15;



//
//////////////////////////////////////////////////////////////////////////////////////////
// Mode of Operation:
//
//      0 = Open Loop Step    -- applies a constant voltage to motor
//                            Inputs  ---------------------------------------------------
//                            Step_Input -- sets the duty cycle of PWM output; an integer
//                                          value between 0 and 255
//                            I_Gain -- Gain of Input Filter function I(s) in encoder counts/radians
//                            Time -- sets the duration of the test in seconds
//
//                            Outputs ---------------------------------------------------
//                            Reference_Input -- duty cycle of PWM output; an integer value
//                                               between 0 and 255
//                            Time -- sets the duration of the test in seconds

//
//      1 = Closed Loop Step    -- PID Controller
//                            Inputs  ---------------------------------------------------
//                            Kp, Ki, Kd -- Proportional, integral, derivative gains, respectively
//                            Time -- sets the duration of the test in seconds

//                            Outputs ---------------------------------------------------

//                            Reference_Input -- duty cycle of PWM output; an integer value
//                                               between 0 and 255
//                            Time -- sets the duration of the test in seconds


///////////////////////////////////////////////////////////////////////////////////////////
//Declare mode of operation:

static byte mode = 0;

///////////////////////////////////////////////////////////////////////////////////////////
//Functions (forward declare):
void  encoderISR_A();
void  encoderISR_B();
void  TimerISR();
void  goISR();
void  stopISR();
void  RunMotor();

void setup()
{
  Serial.begin(9600);

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

  //Buton input pins
  pinMode(GOPIN, INPUT);
  pinMode(STOPPIN, INPUT);

  //initialize sixteenth step
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);

  digitalWrite(DIR, LOW); //Pull pin low to move "forward"
  digitalWrite(MS1, HIGH); //Set MS1,MS2, and MS3 high to set logic to 1/16th microstep resolution
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);

  //Encoder interrupts:
  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), encoderISR_B, RISING);

  //Button interrupts:
  attachInterrupt(digitalPinToInterrupt(GOPIN), goISR, RISING);
  attachInterrupt(digitalPinToInterrupt(STOPPIN), stopISR, RISING);

  MsTimer2::set(Period, TimerISR);

}
//ISR to increment count of encoder to count its square wave frequency

void TimerISR()
{
  if (Go)
  {
    Serial.println("In ISR");
    Elapsed_Time++;
    RunMotor();

    edgeCount = 0;                          //Reset encoder count
  }
}

//Function to stop motor:
void StopMotor()
{
  digitalWrite(RST, LOW);   //When set LOW, all STEP commands are ignored and all FET functionality is turned off
  digitalWrite(ENABLE, HIGH); //If set to HIGH, all FETs will be disabled, restricting motor control.
  Serial.println("Stop Motor");
  digitalWrite(SLEEP, LOW);     //When set to low, A498 enters sleep mode


}

//External interrupt to increment encoder count:
void encoderISR_A()
{
  edgeCount++;
}

void encoderISR_B()
{
  edgeCount++;
}

//Interrupt function to start system:
void goISR()
{
  if ( millis() - goTime > 1500)  //debounce button to ensure timer isn't started again
  {
    digitalWrite(RST, HIGH);   //This must be set HIGH to enable functionality of the motor driver.
    digitalWrite(ENABLE, LOW); //If set to LOW, all FETs will be enabled, allowing motor control.
    digitalWrite(SLEEP, HIGH); 
    Go = 1;
    Stop = 0;
    Serial.println("Go Pressed");
    MsTimer2::start();
    goTime = millis();
  }
}

void stopISR()
{
  if (millis() - stopTime > 1000)     //less of delay for safety
  {
    Serial.println("Stop pressed");  
    Stop = 1;
    Go = 0;
    StopMotor();
    stopTime = millis();
  }

}

//call during Timer1 OVF ISR


void RunMotor()
{
  if (Go)
  {
    digitalWrite(STEP, LOW);           //stepping is triggered by rising edge
    delayMicroseconds(100);               //change to input if ever want PID
    digitalWrite(STEP, HIGH);
    delayMicroseconds(100);
    Serial.println("Running motor"); 
  }
}


void loop()
{
  RunMotor(); 
   delay(5); 
   if(Elapsed_Time*Period/1000 > Time)
   {
      
        Serial.println("Time finished");
  
   }
}

