///////////////////////////////////////////////////////////
//  Base Code v1
//  May 25, 2016
//  By Chris McClellan
///////////////////////////////////////////////////////////

#include <MsTimer2.h>
#include <math.h>

// Setup Motor Controller 1 Pins
static byte PWM1_Out = 4;      // Motor Controller PWM1 assigned to Digital Pin 4 
static byte PWM2_Out = 5;      // Motor Controller PWM2 assigned to Digital Pin 5 
static byte Disable_Out = 6;   // Motor Controller Disable assigned to Digital Pin 6 
static byte En_Out = 7;        // Motor Controller Enable assigned to Digital Pin 7
static byte FB_AIn = 0;        // Motor Controller Current Feedback assigned to Analog Pin 0

// Setup Encoder Counter 1 Pins
static byte SEL1 = 38;         // Select 1 output signal assigned to Pin 38
static byte OE = 39;           // Output Enable signal assinged to Pin 39
static byte RST = 40;          // Reset output signal assigned to Pin 40
static byte SEL2 = 41;         // Select 2 output signal assinged to Pin 41

// Declare Contstants
static float pi = 3.141593;

// Setup ISR Global Variables
volatile byte Go = 0;                         // Interrupt flag for START
volatile byte Timer_Go = 0;                   // Interrupt flag for timer
volatile byte Stop = 0;                       // Interrupt flag for E-STOP  
volatile byte Stop_bit = 0;                   // E-Stop input flag
volatile signed long int Reference_Input_Encoder = 0;   // Reference Input Signal in encoder counts (R(s)*I(s))
volatile signed long int Encoder_Count = 0;   // Current Encoder position in encoder counts
volatile float VtoPWM = 0;                    // Conversion factor for Volt to PWM  
volatile float KdxFreq = 0;                   // Combined gain of Kd*Freq  
volatile float PeriodinSeconds = 0;           // Control Loop Period in seconds
volatile float Freq = 0;                      // Control Loop Frequency
volatile float Ramp_Slope = 0;                // Slope of Open Loop Ramp function
volatile float Chirp_Rate = 0;                // Rate at which the chirp will change frequency 
volatile float t = 0;                         // Time counter in seconds
volatile float temp = 0;                      // temp variable used as intermediary for assigning Ramp Slope to Reference Input
volatile long unsigned int Cnt = 0;           // Counter variable for control loop 
volatile signed long int Error = 0;           // Position error
volatile signed long int Old_Error = 0;       // Previous position error
volatile float Controller_Output = 0;         // Control loop Controller_Output
volatile float Kp_Output = 0;                 // Proportional output
volatile float Ki_Output = 0;                 // Integral output
volatile float Integral = 0;                  // Integral error term for Integral Control 
volatile float Kd_Output = 0;                 // Derivative output

//////////////////////////////////////////////////////////////////////////////////////////
//                   User Adjustable Variables
//////////////////////////////////////////////////////////////////////////////////////////
//
// Declare Mode of Operation where:
//
//      0 = Open Loop Step    -- applies a constant voltage to motor  
//                            Inputs  ---------------------------------------------------
//                            Step_Input -- sets the duty cycle of PWM output; an integer  
//                                          value between 0 and 255
//                            I_Gain -- Gain of Input Filter function I(s) in encoder counts/radians
//                            Time -- sets the duration of the test in seconds  
//                            Period -- sets the control loop period in milli seconds; an
//                                      integer value between 1 and 1000
//                            Outputs ---------------------------------------------------
//                            Freq -- frequency at which the control loop is run in Hertz
//                            Time -- duration of test in seconds
//                            Reference_Input -- duty cycle of PWM output; an integer value
//                                               between 0 and 255
//                            Count -- encoder count; an integer value between 0 and 2^32
//                            Current_Feedback -- current flowing through the motor's 
//                                                armature; a 10-bit ADC value 
//
//      1 = Open Loop Ramp    -- applies linearly varying voltage to motor                 
//                            Inputs  ---------------------------------------------------
//                            Ramp_Final -- sets the final PWM output when t = Time; an
//                                          integer value between 0 and 255; Note:  the 
//                                          initial PWM output is assumed to be 0 at t = 0
//                            I_Gain -- Gain of Input Filter function I(s) in encoder counts/radians
//                            Time -- sets the duration of the test in seconds  
//                            Period -- sets the control loop period in milli seconds; an
//                                      integer value between 1 and 1000
//                            Outputs ---------------------------------------------------
//                            Freq -- frequency at which the control loop is run in Hertz
//                            Time -- duration of test in seconds
//                            Reference_Input -- duty cycle of PWM output; an integer value
//                                               between 0 and 255
//                            Count -- encoder count; an integer value between 0 and 2^32
//                            Current_Feedback -- current flowing through the motor's 
//                                                armature; a 10-bit ADC value 
//                               
//      2 = Open Loop Chirp   -- applies a sinusoidal voltage in which the frequency
//                               linearly increases with time
//                            Inputs  ---------------------------------------------------
//                            Freq_Final -- sets the final frequency at t = Time; 
//                                          the start frequency is assumed to be 0 rad/s
//                                          at t = 0
//                            PWM_Amp -- sets the PWM output amplitude of the sinusoidal 
//                                       chirp signal; an integer value between 0 and 255
//                            I_Gain -- Gain of Input Filter function I(s) in encoder counts/radians
//                            Time -- sets the duration of the test in seconds  
//                            Period -- sets the control loop period in milli seconds; an
//                                      integer value between 1 and 1000
//                            Outputs ---------------------------------------------------
//                            Freq -- frequency at which the control loop is run in Hertz
//                            Time -- duration of test in seconds
//                            Reference_Input -- duty cycle of PWM output; an integer value
//                                               between 0 and 255
//                            Count -- encoder count; an integer value between 0 and 2^32
//                            Current_Feedback -- current flowing through the motor's 
//                                                armature; a 10-bit ADC value 
//                               
//      3 = Closed Loop Step with PID Controller -- applies a discrete PID Controller based 
//                                                    on Kp, Ki,and Kd values for a step input
//                            Inputs  ---------------------------------------------------
//                            Kp -- Proportional gain value
//                            Ki -- Integral gain value
//                            Kd -- Derivative gain value
//                            Reference_Input -- Gain of step input in radians
//                            I_Gain -- Gain of Input Filter function I(s) in encoder counts/radians
//                            V_In -- Power supplied to Motor Controller in volts
//                            Time -- sets the duration of the test in seconds  
//                            Period -- sets the control loop period in milli seconds; an
//                                      integer value between 1 and 1000
//                            Outputs ---------------------------------------------------
//                            Freq -- frequency at which the control loop is run in Hertz
//                            Time -- duration of test in seconds
//                            Reference_Input -- rotational position in radians
//                            Count -- encoder count; an integer value between 0 and 2^32
//                            Current_Feedback -- current flowing through the motor's 
//                                                armature; a 10-bit ADC value 
//
//      4 = User Defined Function
//
//
//
////////////////////////////////////////////////////////////////////////////////////////// 

// Declare Mode of Operation
static byte Mode = 2;

// Declare Step Input PWM output (0-255)
static float Step_Input = 0;

// Declare Final Ramp PWM output (0-255) 
static float Ramp_Final = 0;

// Declare Final Frequency of Chirp
static float Freq_Final = 2;

// Declare PWM output amplitude of Chirp (0-255)
static float PWM_Amp = 50;

// Declare PID Gains 
static float Kp = 0;                // Set Proportioanl Gain     
static float Ki = 0;                // Set Integral Gain
static float Kd = 0;                // Set Derivative Gain

// Declare Desired Input Value (radians)
float Reference_Input = 0;            // Set Input reference signal

// Declare Input Filter Function I(s) (assumed to be a gain)
float I_Gain = 651.9;

// Declare Voltage Input (volts)
float V_in = 12;

// Declare Control Loop Period in milli seconds (1-1000)
static unsigned int Period = 10;

// Declare Test Duration in seconds
static float Time = 10;

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////
// User Defined Global Variables
//////////////////////////////////////////////////////////////////////////////////////////






//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// Read Encoder Position Function
signed long int Read_Encoder()
{
  signed long int Count = 0;     // encoder position total
  signed long int temp0;         // temp variable used for reading in encoder value
  signed long int temp1;         // temp variable used for reading in encoder value 
  signed long int temp2;         // temp variable used for reading in encoder value
  signed long int temp3;         // temp variable used for reading in encoder value
    
  // initiate sequence to retrieve encoder counter values    
  digitalWrite(OE, HIGH);
  digitalWrite(OE, LOW);

  // retreive byte 0 from encoder counter 
  digitalWrite(SEL1, HIGH);              
  digitalWrite(SEL2, LOW);
  temp0 = PINA;               // Read in values from Port A

  // retreive byte 1 from encoder counter  
  digitalWrite(SEL1, LOW);
  digitalWrite(SEL2, LOW);
  temp1 = PINA;               // Read in values from Port A
    
  // retreive byte 2 from encoder counter  
  digitalWrite(SEL1, HIGH);
  digitalWrite(SEL2, HIGH);
  temp2 = PINA;               // Read in values from Port A    
    
  // retreive byte 3 from encoder counter  
  digitalWrite(SEL1, LOW);
  digitalWrite(SEL2, HIGH);
  temp3 = PINA;               // Read in values from Port A  
  
  Count = temp0 + (temp1 << 8)+ (temp2 << 16)+ (temp3 << 24);     // calculate encoder count 
  
  // end encoder counter read sequence
  digitalWrite(OE, HIGH);  
 
  return Count;
} 

// Reset Encoder Counter Function
void Reset_Encoder_Counter()
{
  digitalWrite(RST, LOW);              // Reset Encoder Counter by toggling RST pin
  digitalWrite(RST, HIGH);  
}

// Ready Motor Function
void Ready_Motor()
{
  digitalWrite(En_Out, HIGH);        // Set Motor Controller Enable pin HIGH
  digitalWrite(Disable_Out, HIGH);   // Set Motor Controller Disable pin HIGH
}

// Stop Motor Function
void Stop_Motor()
{
  // Disable Motor Controller thereby disabling Motor 
  digitalWrite(En_Out, LOW);           // Set Motor Controller Enable low to disable motor
  digitalWrite(PWM1_Out, LOW);         // Set Motor Controller PWM 1 low to disable motor
  digitalWrite(PWM2_Out, LOW);         // Set Motor Controller PWM 2 low to disable motor
  digitalWrite(Disable_Out, LOW);      // Set Motor Controller Disable low to disable motor 
}

// Move Motor Function
void Move_Motor(float PWM_Value)
{
  if(PWM_Value >= 0)               
  {
    if(PWM_Value > 255)                 // Limit Controller_Output to 0 - 255
      PWM_Value = 255;
    digitalWrite(PWM1_Out, LOW);        // Set PWM1_Out pin low
    analogWrite(PWM2_Out, PWM_Value);   // Set PWM2_Out to PWM signal
  }
  else
  {    
    PWM_Value = -PWM_Value;             // Invert signal to keep positive
    if(PWM_Value > 255)                 // Limit Controller_Output to 0 - 255
      PWM_Value = 255;
    analogWrite(PWM1_Out, PWM_Value);   // Set PWM1_Out to PWM signal 
    digitalWrite(PWM2_Out, LOW);        // Set PWM2_Out pin low
  }  
}

// Open Loop Step Function
void OLStep()
{
  Controller_Output = Step_Input;
  Reference_Input = Controller_Output;
}

// Open Loop Ramp Function
void OLRamp()
{ 
  temp += Ramp_Slope;             // When setting Reference_Input += Ramp_Slope directly, Arduino pins  
  Reference_Input = temp;         // would change Modes; had to add temp variable to act as intermediary; now works
  Controller_Output = Reference_Input;
}

// Open Loop Chirp Function
void OLChirp()
{
  t = Cnt/Freq;                                 // Calculate current time in seconds from start
  temp = pi*Chirp_Rate*t*t;        
  Reference_Input = PWM_Amp*sin(pi + temp);     // Calculate Chirp Sigal
  Controller_Output = Reference_Input;
} 

// Closed Loop Step Function with PID Controller
// Note: it is assumed that every Controller has a Proportional term
void CLStep(signed long int Target_Position)
{
  Error = Target_Position - Encoder_Count;            // Calculate new error value 
  
  Kp_Output = Kp*(float)Error;                        // Calculate Proportional Output 
  Controller_Output = Kp_Output;                      // Set Controller Output to Proportional Output
  
  if(Ki > 0)
  {  
    Integral += float(Error)*PeriodinSeconds;         // Calculate Integral error 
    Ki_Output = Ki*Integral;                          // Calculate Integral Output 
    Controller_Output += Ki_Output;                   // Add Integral Output to Controller Output 
  }
  if(Kd > 0)
  {
    Kd_Output = (float(Error-Old_Error))*KdxFreq;     // Calculate Derivative Output
    Controller_Output += Kd_Output;                   // Add Derivative Output to Controller Output
  }
 
  Controller_Output = Controller_Output*VtoPWM;       // Convert from volts to PWM duty cycle  
  Old_Error = Error;                                  // Save old error value
} 

//////////////////////////////////////////////////////////////////////////////////////////
// User Defined Subroutines/Functions
//////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// Main Setup Routine
void setup() 
{
  Serial.begin(115200);          // Setup serial output to 115,200 bps (bits per second)
     
  // Setup Data input pins from Encoder as inputs
  DDRA = 0;                      // Set pins 22-29 as inputs

  // Setup Motor Controller 1 pins as Outputs
  pinMode(PWM1_Out, OUTPUT);     // Setup digital pin 4 as an output
  pinMode(PWM2_Out, OUTPUT);     // Setup digital pin 5 as an output
  pinMode(Disable_Out, OUTPUT);  // Setup digital pin 6 as an output
  pinMode(En_Out, OUTPUT);       // Setup digital pin 7 as an output
  
  // Setup Encoder 1 Counter pins as Outputs
  pinMode(SEL1, OUTPUT);         // Setup digital pin 38 as an output 
  pinMode(OE, OUTPUT);           // Setup digital pin 39 as an output  
  pinMode(RST, OUTPUT);          // Setup digital pin 40 as an output
  pinMode(SEL2, OUTPUT);         // Setup digital pin 41 as an output   
  
  // Setup Interrupt Service Routine driven by a rising edge on int 0(Pin2) for E-Stop
  pinMode(2, INPUT);                     // Setup digital pin 2 as an input
  attachInterrupt(0,STOP,RISING);        // Setup 'STOP' interrupt to begin on RISING edge
  
  // Setup Interrupt Service Routine driven by a rising edge on int 1(Pin3) for Start Button
  pinMode(3, INPUT);                     // Setup digital pin 3 as an input
  attachInterrupt(1,START,RISING);       // Setup 'START' interrupt to begin on RISING edge
 
  Stop_Motor();
  
  // Reset Encoder Counter by toggling RST pin
  digitalWrite(RST, LOW);          
  delay(100);
  digitalWrite(RST, HIGH); 
  
  // Setup Timer for Control Loop Frequency
  MsTimer2::set(Period, Timer_ISR);   // Set Control Loop Frequency (Hz) to 1000/Period  
}

void loop()
{  
  // Define Local Variables  
  unsigned long int Current_Feedback_Total;  // accumulator variable for filtering Current Feedback
  unsigned int Current_Feedback;             // average value of Current Feedback
  int i;                                     // counter variable used for filtering Current Feedback    
  long unsigned int Cnt_Max = 0;             // Number of iterations to be preformed

  // Reset Global Variables  
  Go = 0;                               // Reset START flag      
  Stop = 0;                             // Reset E-STOP flag    
  Old_Error = 0;                        // Reset previous position error 
  Integral = 0;                         // Reset Integral error term for PID Control  
  temp = 0;                             // Reset temp variable
  
  //////////////////////////////////////////////////////////////////////////////////////
  // User Defined Local Variables and Global Variable Reset
  //////////////////////////////////////////////////////////////////////////////////////
  
  
  //////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////  

  // Precalculate Loop Variables to speed up execuation time
  PeriodinSeconds = (float)Period/1000; // Control Loop Period converted to seconds from milliseconds
  Freq = 1/PeriodinSeconds;             // Calculate Control Loop Frequency in Hz
  Reference_Input_Encoder = int(Reference_Input * I_Gain);     // Convert Reference Input from radians to encoder counts
  Cnt_Max = Freq * Time;                // Calculate number of interations to be performed
  KdxFreq = Kd*Freq;                    // Combined gain of Kd*Freq 
  VtoPWM = 255/V_in;                    // Conversion factor of Volts to PWM 
  Ramp_Slope = Ramp_Final/(Time*Freq);  // Calculate Ramp Slope (final value / number of iterations)  
  Chirp_Rate = Freq_Final/Time;         // Calculate Chirp Rate (final frequency / time) 
  
  // Send out initial settings
  Serial.println(Freq);                 // Send Freq value out serially
  Serial.println(Time);                 // Send Time value out serially
  Serial.println(I_Gain);               // Send I_Gain value out serially
    
  Stop_Motor();
    
  while(!Go);                    // Wait for START push button to be pressed 
  delay(100);                    // Set delay of 100ms to debounce switch
  Go = 0;                        // Reset START flag 
     
  Reset_Encoder_Counter();       // Reset Encoder Counter to position 0
  Ready_Motor();                 // Get motor ready
  MsTimer2::start();             // Start timer  
  
  //////////////////////////////////////////////////////////////////////////////////////////
  // User Defined Pre-Loop Code 
  //////////////////////////////////////////////////////////////////////////////////////////



  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////   
  
  for(Cnt=0;Cnt <= Cnt_Max;Cnt++)     
  { 
    i = 0;
    Current_Feedback_Total = 0;
    Current_Feedback = 0;
    
    while(Timer_Go == 0)            // Wait for next Timer iteration`      
    {  
      // Continuously read in and accumulate Motor Controller Current Feedback while waiting for timer
      // in an attempt to reduce noise
      Current_Feedback_Total += analogRead(FB_AIn);
      i++;
    }
    
    Timer_Go = 0;                   // Reset Timer flag

    if(i > 0)
      Current_Feedback = Current_Feedback_Total/i;   // Divide by number of samples to get average
    
    Serial.println(Reference_Input);     // Send Reference_Input value out serially
    Serial.println(Encoder_Count);       // Send Encoder_Count value out serially
    Serial.println(Current_Feedback);    // Send Current_Feedback value out serially    
    
    if((Stop) || (Stop_bit))             // If E-Stop has been depressed, break from loop
      break;
  }
  
  Stop_Motor();                  // Stop motor  
  MsTimer2::stop();              // Stop timer
  delay(100);                    // Set delay of 100ms to debounce switch
} 

// Timer Interrupt Service Routine trigger by MsTimer2 function
void Timer_ISR()
{ 
  Encoder_Count = Read_Encoder();     // Read in current encoder position   
  
  // Open Loop Step Mode
  if(Mode == 0)                     
    OLStep();   
  
  // Open Loop Ramp Mode
  else if(Mode == 1)                
    OLRamp();  

  // Open Loop Chirp Mode
  else if(Mode == 2)
    OLChirp();
  
  // Closed Loop Step Mode with PID Controller
  else if(Mode == 3)  
    CLStep(Reference_Input_Encoder);
  
  //////////////////////////////////////////////////////////////////////////////////////
  //  User Defined Mode
  //////////////////////////////////////////////////////////////////////////////////////
  else if(Mode == 4)
  {

    
    
  } 
  //////////////////////////////////////////////////////////////////////////////////////   
  //////////////////////////////////////////////////////////////////////////////////////          
    
  Stop_bit = digitalRead(2);         // Read in status of E-Stop   
  if((!Stop) && (!Stop_bit))         // If the E-Stop has not been pressed then...
    Move_Motor(Controller_Output);   // Call Move Motor Function 
  else                               // Else...
    Stop_Motor();                    // Call Stop Motor Function 
 
  Timer_Go = 1;                      // Set Timer flag
}  

// Interrupt Service Rountine triggered by RISING edge from user activated
// momentary push button (Start Button)
void START()
{
  Go = 1;                   // Set Push Button flag  
}   
       
// Interrupt Service Rountine triggered by RISING edge from E-Stop
void STOP()
{
  Stop = 1;                 // Set STOP flag
  Stop_Motor();             // Call Stop Motor Function 
}
