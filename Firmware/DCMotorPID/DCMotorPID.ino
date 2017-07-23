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
  
  unsigned int stopPin =     0;             //feather interrupt pin2
  unsigned int startPin =    1;             //feather interrupt pin3
  unsigned int encoderPin =  2;             //feather interrupt pin1
  unsigned int voltagePin =  3;             //feather interrupt pin0
  unsigned int motorPin =    5;
  
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
  volatile double Integral= 0;               //Integral of the error
  
  volatile unsigned long edgeCount = 0;      //Encoder edge count
  volatile double Motor_Speed = 0;            //Motor Ref_Speed in RPM
  volatile float Elapsed_Time = 0;           //Elapsed time of the test
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //User defined variables:
  ///////////////////////////////////////////////////////////////////////////////////////////
  
  double Kp = 0;                             //Proportional gain
  double Ki = 0;                             //Integral gain
  double Kd = 0;                             //Derivative gain
  
  //Define input voltage
  float V_in  = 12.0;
  
  //Timer timeout period in ms
  static unsigned int Period = 50;
  
  //Desired Speed [in RPM]
  static unsigned int Ref_Speed =  30;
  
  //Open loop variables [in RPM]
  volatile float Ref_Input = 0;

  //Open loop variable - step input [PWM]
  volatile float Step_Input = 0;

  //Duration of the test [in seconds]:
  volatile float Time = 0; 

  

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
  //Functions:
void  encoderISR(); 
void  TimerISR();
void  StopMotor(); 
void  START(); 
void  STOP(); 
void  ClosedLoopStep(); 
void  RunMotor(); 
void  OpenLoopStep(); 
  
  
  //ISR to increment count of encoder to count its square wave frequency
  void encoderISR()
  {
    edgeCount++;
  }
  
  void TimerISR()
  {
    Elapsed_Time++; 
    Motor_Speed = (double) edgeCount * 60 / (2 * 16 * 50) / Period;
    // multiply by the gear ratio (50:1) divided by 1 second [revolutions/minute]
  
    if (mode == 0)
      OpenLoopStep();                       //get constant voltage value
  
    if (mode == 1)
      ClosedLoopStep();                     //get error from motor speed and ref_speed

    RunMotor(Controller_Input); 
  
    edgeCount = 0;                          //Reset encoder count
  }
  
  //Function to stop motor:
  void StopMotor()
  {
    analogWrite(motorPin, 0);                  //Cut voltage to motor
    digitalWrite(voltagePin, LOW);
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
  
  void ClosedLoopStep()
  {
    Error = Ref_Speed - Motor_Speed;
    if (Kp > 0)
    {
      KpOutput = Kp * Error;
      Controller_Input = KpOutput;
    }
  
    if (Kd > 0)
    {
      KdOutput = Kd * (Error - prevError)/Period; // dx/dt = delta error/delta time(a.k.a. period)
      Controller_Input += KdOutput;
    }
  
    if (Ki > 0)
    {
      Integral += Error * Period;                 //
      KiOutput = Ki * Integral;
      Controller_Input += KiOutput;
    }
  
    prevError = Error;
    Controller_Input *= VtoPWM;
  }
  
  void RunMotor(float input)
  {
    if (!Stop && Go)
    {
      input = abs(input);
      if (input > 255)
        input = 255;
  
      analogWrite(motorPin, input);
    }
  }
  
  void OpenLoopStep()
  {
    Controller_Input = Step_Input;
    Ref_Input = Controller_Input;
  }
  
  void setup()
  {
    Serial.begin(9600);
    pinMode(encoderPin, INPUT);         //encoder ISR
    pinMode(startPin, INPUT);           //START ISR
    pinMode(stopPin, INPUT);            //STOP ISR
    pinMode(motorPin, OUTPUT);
    pinMode(voltagePin, OUTPUT);
    digitalWrite(voltagePin, HIGH);
    
    MsTimer2::set(Period, TimerISR);
    attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(stopPin), STOP, RISING);
    attachInterrupt(digitalPinToInterrupt(startPin), START, RISING);
  
    VtoPWM = 255 / V_in;
    
    MsTimer2::start();
  }
  
  void loop()
  {
    if (Stop)
    {
      STOP(); 
      while (1) {};    //Require MCU reset -- safety measure
    }

    if(Elapsed_Time*Period > Time)             //Elapsed time (counts)*time/tick = elapsed time
    {
      STOP(); 
      while(1){};   //Require MCU reset -- safety measure
    }
        
    Serial.println(Time); 
    Serial.println(Motor_Speed); 
    Serial.println(Controller_Input); 
     
  }
