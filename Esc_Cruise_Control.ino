/*ESC cruise control:
Using pot input for speed setting and rpm feedback from esc, maintain speed/power output through range of battery voltage

Using interrupt to catch rpm/Pole pulse from castle creations ESC Aux output.

Board Digital Pins Usable For Interrupts 
Uno, Nano, Mini, other 328-based 2, 3 
Mega, Mega2560, MegaADK 2, 3, 18, 19, 20, 21 
Micro, Leonardo, other 32u4-based 0, 1, 2, 3, 7 
Zero all digital pins, except 4 
Due all digital pins 
*/

/* 
 Using Servo library:
 Controlling a servo position using a potentiometer (variable resistor) 
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob

 And a PID library: form somewhere.
*/

#include <Servo.h>
#include <PID_v1.h>

Servo escSpeed;  // create servo object to control a servo
int MAXRPM = 12000;
int motorPoles = 7;       // number of magnetic poles in motor divided by 2. required as rpm output from castle is 1 pulse per pole pair.
int speedSettingPin = 0;  // analog pin used to connect the potentiometer
const int escPin=9;             // connect to esc
const int rpmPin=3;             // connect rpm input * only some digital pins usable as interrupt.
int speedSetting;    // variable to read the value from the analog pin
unsigned long rpmCheckInterval =500000; //in microseconds.

unsigned long currentTime=micros();     //current time in microseconds since arduino started running
unsigned long oldTime=currentTime;      //timer for rpm calculation
unsigned long oldTimePID;               //timer for slow PID/speed setting 
double rpm;
bool toggle=0;                          //debugging
unsigned long interval=1000;            //measured time in microseconds between rpm inputs. start at large value so rpm is initially at zero.

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

const double consKp=.001, consKi=.005, consKd=0; //Pids to use when input is close to setpoint
const double aggKp=.01, aggKi=.05, aggKd=0;      //Pids to use when input is far from setpoint

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

int OutputPWM=0;

void isr() //interrupt service routine to count rpm every time pulse is given.
{
  currentTime=micros();
  interval = currentTime-oldTime;
  oldTime=micros();  
  
  //DEBUG*********************************************
  //pulse arduino led if recieving rpm input
  if(toggle==0)
  {
    digitalWrite(13,HIGH);
    toggle=1;
  }
  else
  {
    digitalWrite(13,LOW);
    toggle=0;
  }
  //DEBUG*********************************************
}

void computePID()
{

}

void setup()
{
  myPID.SetSampleTime(1);
  pinMode(rpmPin, INPUT);
  pinMode(13,OUTPUT);  // debug onboard led
  
  attachInterrupt(digitalPinToInterrupt(rpmPin),isr,RISING);  //attaching the interrupt
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200); //used for debugging.
  
  escSpeed.attach(escPin);  // attaches the servo on pin 9 to the servo object

  //initialize the PID variables we're linked to
  Input = 0;
  Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(900, 2000);
  
}

void loop() 
{ 
    currentTime=micros();

     double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 100)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
 
  rpm = 1*60000000/interval/motorPoles;
  speedSetting = analogRead(speedSettingPin);            // reads the value of the potentiometer (value between 0 and 1023) 
  speedSetting = map(speedSetting, 0, 1023, 0, MAXRPM);  // scale it to rpm range required.
  Input = rpm;
  //debug..
  Setpoint=speedSetting;
 

 // OutputPWM=map(Output,0,255,1200,2023);                   
 escSpeed.writeMicroseconds(Output);                  // sets the esc speed according to the scaled value  
 myPID.Compute();

  if(currentTime-oldTimePID>rpmCheckInterval)
  {
    if(Setpoint==0)
    Input=0; //needed as getting huge value sometimes for input when motor is slowed to a stop, and untill motor is started, interrupt is not called and input isn't adjusted.

   Serial.print("\n Setpoint: ");
    Serial.print( Setpoint);
    Serial.print(" Input: ");
    Serial.print(Input);
    Serial.print(" Output: ");
    Serial.print(Output);

    oldTimePID=currentTime;   
  }
} 


