//Main stepepr control progarm

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Wire.h>

int x=200;
int y=200;
int Max_speed = 1000;

Adafruit_MotorShield AFMSbot = Adafruit_MotorShield(0x60); 
Adafruit_MotorShield AFMStop = Adafruit_MotorShield(0x61); 

Adafruit_StepperMotor *myMotor3 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *myMotor2 = AFMSbot.getStepper(200, 2);
Adafruit_StepperMotor *myMotor4 = AFMStop.getStepper(200, 1);
Adafruit_StepperMotor *myMotor1 = AFMStop.getStepper(200, 2);

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  
  AFMSbot.begin();
  myMotor3->setSpeed(Max_speed);
  myMotor2->setSpeed(Max_speed);

  AFMStop.begin();
  myMotor4->setSpeed(Max_speed);
  myMotor1->setSpeed(Max_speed);

int i;
  // put your main code here, to run repeatedly:
  for (i = 0; i <= x; i++)
  {
  myMotor3->step(1,FORWARD,DOUBLE);
  myMotor2->step(1,BACKWARD,DOUBLE);
  }
  delay(3);
  for (i = 0; i <= y; i++)
  {
  myMotor4->step(1,BACKWARD,DOUBLE);
  myMotor1->step(1,FORWARD,DOUBLE);
  }
}



void loop() {
  Serial.write("Running");
  }