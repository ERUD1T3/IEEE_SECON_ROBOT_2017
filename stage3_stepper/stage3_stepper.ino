#include <AccelStepper.h>
#define CYCLE 4
//
// Motor pin definitions
#define motorPin1  2   
#define motorPin2  3     
#define motorPin3  4    
#define motorPin4  5  

#define motorPin5  8
#define motorPin6  9
#define motorPin7  10
#define motorPin8  11

long opTime(10000);

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepper1(CYCLE, motorPin1, motorPin3, motorPin2, motorPin4);
AccelStepper stepper2(CYCLE, motorPin5, motorPin6, motorPin7, motorPin8);

void setup() {

  Serial.begin(9600);
  Serial.print("Init");
  
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(100.0);
  stepper1.setSpeed(200);
  stepper1.moveTo(1000);

  stepper2.setMaxSpeed(1000.0);
  stepper2.setAcceleration(100.0);
  stepper2.setSpeed(200);
  stepper2.moveTo(1000);
  // 360 degrees is 2200 steps
}

void loop()
{ 
  Serial.write("Running");
  while(millis() <= opTime)
  {
    if(stepper1.distanceToGo() == 0) //Change direction for #1
    stepper1.moveTo(-stepper1.currentPosition());

    if(stepper2.distanceToGo() == 0) //Change direction for #2
    stepper2.moveTo(-stepper2.currentPosition());
    
    stepper1.run(); //Only works in void loop
    stepper2.run(); //Only works in void loop
  }
}