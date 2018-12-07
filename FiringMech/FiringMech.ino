#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Servo.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Servo trigger;
int DCmPin(13);

void Fire_dart(int n) //fires n darts 
{
     for (int i(1); i <= n; i++)
     {
          trigger.write(50);
          delay(1000);
          trigger.write(150);
          delay(500);
          trigger.write(50);
     }
}

//Adafruit_MotorShield AFMStrigger = Adafruit_MotorShield(0x63);

void setup() 
{
  Serial.begin(9600);
  Serial.println("Init Firing");
  pinMode(DCmPin, OUTPUT); //Set pin as output
  
  trigger.attach(10);
  

  digitalWrite(DCmPin, HIGH); //start motors 
  Fire_dart(2);
  delay(2500);
  Fire_dart(1);
  digitalWrite(DCmPin, LOW); //stop the motor
}

void loop() {Serial.print("Running");}