//Main stepepr control progarm

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Wire.h>

int Max_speed = 1000;

int i;
int count;

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
  myMotor1->setSpeed(Max_speed);
  myMotor2->setSpeed(Max_speed);

  AFMStop.begin();
  myMotor3->setSpeed(Max_speed);
  myMotor4->setSpeed(Max_speed);
  count = 0;
}


void goBackward(int d) {        //d in millimeters

  Serial.write("Moving backward");
  for (i = 0; i <= d; i++)
  {
  myMotor2->step(1,BACKWARD,DOUBLE);
  myMotor3->step(1,FORWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done moving backward");

}

void goForward(int d) {        //d in millimeters

  Serial.write("Moving forward");
  for (i = 0; i <= d; i++)
  {
  myMotor2->step(1,FORWARD,DOUBLE);
  myMotor3->step(1,BACKWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done moving forward");

}

void goLeft(int d) {        //d in millimeters

  Serial.write("Moving left");
  for (i = 0; i <= d; i++)
  {
  myMotor1->step(1,FORWARD,DOUBLE);
  myMotor4->step(1,BACKWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done moving left");

}



void goRight(int d) {        //d in millimeters

  Serial.write("Moving right");
  for (i = 0; i <= d; i++)
  {
  myMotor1->step(1,BACKWARD,DOUBLE);
  myMotor4->step(1,FORWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done moving right");

}


void loop() {
  Serial.write("Running");
  while (count<1) {
    goForward(20);
    goLeft(10);
    goBackward(20);
    goRight(10);
    count++;
  }
  
  }