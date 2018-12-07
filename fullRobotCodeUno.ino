///This program controls the robot for IEEE Robotics
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#define reset 0

/*VARIABLES***************************************************************/
Servo trigger;
int DCmPin(13);
LiquidCrystal_I2C lcd(0x27,16,2);

// limit switch variables
//pin numbers
int sense1 = 9;  //left 1
int sense2 = 11;  //left 2
int sense3 = 8;  //right 1
int sense4 = 12;  //right 2
int limright = 6;
int limleft = 7;

int sensestate1 = 0;
int sensestate2 = 0;
int sensestate3 = 0;
int sensestate4 = 0;

int movestep;   //variable to control speed
int code;
bool haha(true);
int output;

int fieldlength = 0;
int fieldwidth = 0;


//Gyro variables
unsigned long currentMillis, previousMillis = 0, currentTime, delta_time, previousTime = 0;
float gx, gy, gz, ax, ay, az;
float gx_anew, gy_anew, gz_anew;
float thresh1 = 0.8;                    //offset threshold for gyroscope
float gx_velocity, gy_velocity, gz_velocity;
float gx_angle = 0, gy_angle = 0, gz_angle = 0; 
float gx_sum, gy_sum,gz_sum, zSum = 0.0;


//Rotation Variables!
int rotateCorrection = 5;                           //rotation correction factor
float rotateAngle = 0.6159936776;

//might need to delete unsigned long time and time_passed
unsigned long time;
long time_passed;
int Max_speed = 1000;
int i;
int count;

bool stage1;      //stage control
bool stage2;
bool stage3;
bool stage4;

Adafruit_MotorShield AFMSbot = Adafruit_MotorShield(0x60);    //motor control
Adafruit_MotorShield AFMSmid = Adafruit_MotorShield(0x61); 
Adafruit_MotorShield AFMStop = Adafruit_MotorShield(0x63); 

Adafruit_StepperMotor *myMotor3 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *myMotor2 = AFMSbot.getStepper(200, 2);
Adafruit_StepperMotor *myMotor4 = AFMSmid.getStepper(200, 1);
Adafruit_StepperMotor *myMotor1 = AFMSmid.getStepper(200, 2);
Adafruit_StepperMotor *myMotor5 = AFMStop.getStepper(200, 1);
Adafruit_StepperMotor *myMotor6 = AFMStop.getStepper(200, 2);

//stage3 variables
int turn1,turn2,turn3,turn4,turn5;
int first = 3, second = 3, third = 3, fourth = 3, fifth = 3;

/*FUNCTIONS*****************************************************************************/


void goForwardMicro(int);

int lastValRight = 0;
int lastValLeft = 0; 

// Sonar stuff
int trig[] = {4,3};
int echo[] = {5,2};

//check lateral orientation
void correctLateral (int distance = 72) {
  double offset = distance-10.5;
  double average = (sonarRight()+sonarLeft())/2;
  double error = offset-average;
  double range = 1;
  if((error<-range)) { //less than target
    goForwardMicro(10);
     
  }
  if((error>range)) { //more than target
    goBackwardMicro(10);
      
  }
}



long microsecondsToCentimeters(long microseconds)
{

return microseconds / 29 / 2;
}

int sonarRight () {

long duration, inches, cm;

  pinMode(trig[0], OUTPUT);
  digitalWrite(trig[0], LOW);
  delayMicroseconds(2);
  digitalWrite(trig[0], HIGH);
  delayMicroseconds(5);
  digitalWrite(trig[0], LOW);

  pinMode(echo[0],INPUT);
  duration = pulseIn(echo[0], HIGH);

  cm = microsecondsToCentimeters(duration);

  if(abs(cm)>1000) {
    cm = lastValRight;
  }
  else {
    lastValRight = cm;
  }
  
  return cm;
}

int sonarLeft () {

long duration, inches, cm;

  pinMode(trig[1], OUTPUT);
  digitalWrite(trig[1], LOW);
  delayMicroseconds(2);
  digitalWrite(trig[1], HIGH);
  delayMicroseconds(5);
  digitalWrite(trig[1], LOW);

  pinMode(echo[1],INPUT);
  duration = pulseIn(echo[1], HIGH);

  cm = microsecondsToCentimeters(duration);

  if(abs(cm)>1000) {
    cm = lastValLeft;
  }
  else {
    lastValLeft = cm;
  }
  
  return cm;
}

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

int slave_work()
{
  while (haha == true){
  Serial.println("exterminate!");
  //Wire.beginTransmission(8); 
  //Wire.write('s');       
  //Wire.endTransmission(); 
  
  Wire.requestFrom(8, 1);   // the first byte
 while(Wire.available())
 { 
   unsigned char received = Wire.read();
   output = received;
 }
 
 for (i = 0 ; i < 3 ; i++) // next 3 bytes
 {
    Wire.requestFrom(8, 1);   
    while(Wire.available())
    { 
       unsigned char received = Wire.read();
       output |= (received << 8);
    }
 }
 Serial.println(output);
 time = millis();
 if (time > 10000){
  haha = false;
  }
 }
 
 if(output == 0)
  haha = true;

  translate(output);
  Serial.print(first);
  Serial.print(second);
  Serial.print(third);
  Serial.print(fourth);
  Serial.print(fifth);
 lcd.println(output);
 lcd.clear();
}

void move_to_stage1() 
{
  lcd.clear();
  lcd.print("STAGE 1");
  delay(500);
  check_time();
  //goLeft(700);
  int c = checkLimits();
  movestep = 100;
  short flag = 0;
  while(c != 120) {
    if (millis()>60000) break;
    c = checkLimits();
    if (c==120) break;
    if (c==12) movestep = 1;
    if (c==4) {         //correction case 1
      rotate(1.0);
      continue;
    }
    if (c==8) {         //correction case 2
      rotate(-1.0);
      continue;
    }
    goRight(movestep);
    if (movestep == 100)  { 
      movestep = 10;
      
      //parallelize();
      correctLateral();
      
      
    }
    fieldwidth+=movestep;
  }
  
  
}

void do_stage1() 
{
  lcd.clear();
  lcd.print("Doing STAGE 1");
  check_time();
  /*while(time_passed < 90000)
  {
     code = slave_work();
  }
  translate(code);
  */
  lcd.clear();
  code = 33333;
  lcd.print("33333");
  delay(1000);
  
  delay(2000);
  fieldwidth = fieldwidth*2;
  //goRight(700);
  goLeft(fieldwidth);
  stage1 = false;
  stage3 = true;
}

//No stage 2!!!! >.<
void move_to_stage2() 
{
  lcd.clear();
  lcd.print("STAGE 2");
  delay(2000);
  check_time();
  
  //goBackward(100);
  goLeft(fieldlength);
  goBackward(100);
  
  delay(2000); 
}

void do_stage2() 
{
  lcd.clear();
  lcd.print("Doing STAGE 2");
  delay(2000);
  check_time();
  goForward(100);
  stage2 = false;
  stage3 = true;
}

void platform_turn(int d);

void move_to_stage3() 
{
  lcd.clear();
  lcd.print("STAGE 3");
  delay(500);
  check_time();
  //goRight(100);
  //platform_turn(50);

  int c = checkLimits();
  movestep = 100;
  short flag = 0;
  
  while(c != 30) {
    if (millis()>100000) break;
    c = checkLimits();
    if (c==30) break;
    if (c==3) movestep = 1;
    if (c==1) {         //correction case 1
      rotate(-1.0);
      continue;
    }
    if (c==2) {         //correction case 2
      rotate(1.0);
      continue;
    }
    goLeft(movestep);
    if (movestep ==100) { 
      movestep = 10;
      
      //parallelize();
      correctLateral();
      
    }
    check_time();
  }

  
  delay(3000);
}

void do_stage3() 
{
  lcd.clear();
  lcd.print("Doing STAGE 3");
  delay(500);
  turn_knob();
  delay(1000);
  check_time();
  //goRight(100);
  delay(500);
  
  stage3 = false;
  stage4 = true;
}

void move_to_stage4() 
{
  goRight(fieldwidth);
  delay(2000);
}

void do_stage4() 
{
  delay(2000);
  //Firing one dirt
  digitalWrite(DCmPin, HIGH); //start motors 
  Fire_dart(1);
  digitalWrite(DCmPin, LOW); //stop the motor
  stage4 = false;     //end test
}

void check_time()     //skip to stage 4 after a certain amount of time passed
{
  lcd.clear();
  time_passed = millis();
  lcd.print(time_passed);
  
  if (time_passed > 210000){
  stage1 = false;
  stage2 = false;
  stage3 = false;
  stage4 = true;
  }

}

float getAbs(float x) {
  if (x >=0.0) return x;
  else return -1.0*x;
}

float find_angle(int d){    
    //find time difference between measures
 currentTime = millis();
 delta_time = (currentTime - previousTime);
 if (delta_time>0) 
 {
   previousTime = currentTime;
   //find angular velocity
   //CurieIMU.readGyroScaled(gx, gy, gz);
   //CurieIMU.readGyro(gx, gy, gz);
   
   if(getAbs(gz)>=thresh1) 
   {
     zSum+=gz;
   }
   else 
   {
    // zSum = 0.0;
   }
     
   gx_velocity = (gx/32768.9)*250;
   gy_velocity = (gy/32768.9)*250;
   gz_velocity = (gz/32768.9)*250;
        
        //find small angle, time should be in seconds

   delta_time = delta_time/1000.0;
        
   gx_anew = gx_velocity*(delta_time);
   gy_anew = gy_velocity*(delta_time);
   gz_anew = gz_velocity*(delta_time);
        //sum the small angles to find total angular displacement
        //gx_sum = gx_anew + gx_angle;
        //gy_sum = gy_anew + gy_angle;
        //gz_sum = gz_anew + gz_angle;
        
    gx_angle += gx_anew;
    gy_angle += gy_anew;
    gz_angle += gz_anew;
   }
  
  currentMillis = millis();
  if (currentMillis - previousMillis >= 1000)
  {
    previousMillis = currentMillis;
    // Serial.print(gx);
     //Serial.print("\t");

     //Serial.print(gy);
     //Serial.print("\t");
    // Serial.print(gz);
    // Serial.print("\t");
    
     //Serial.print(gx_velocity);
     //Serial.print("\t");

    // Serial.print(gx_anew);
    // Serial.print("\t");
  
    // Serial.print(currentTime);
    // Serial.print("\t");

   //  Serial.print(previousTime);
    // Serial.print("\t");
  
    // Serial.print(delta_time);
    // Serial.print("\t");
  
     Serial.print("g:\t");         //display angle offset
     Serial.print(gx_angle);
     Serial.print("\t");
     Serial.print(gy_angle);
     Serial.print("\t");
     Serial.print(gz_angle);
     Serial.print("\t");
     Serial.print(zSum/1000.0);
     Serial.println();

    if( d== 0)
        {
          gx_angle = 0;
          gz_angle = 0;
         }          gy_angle = 0;


     return gz_angle;
  }
}

void goLeft(int d) {        //d in millimeters

  Serial.write("Moving left");
  for (i = 0; i <= d; i++)
  {
  myMotor3->step(1,FORWARD,DOUBLE);
  myMotor2->step(1,BACKWARD,DOUBLE);

 // int j = i%200;
   // while(j == 0)
    //{
       //Serial.println("Finding Angle");
       //float a = -find_angle();
       //Serial.println("Correcting!")
       //rotate(a);
       //j = 1;
    //}
    
  }
  delay(20);
  Serial.write("Done moving left");
  find_angle(reset);
}

void goLeftMicro(int d) {        //d in millimeters

  Serial.write("Moving left");
  for (i = 0; i <= d; i++)
  {
  myMotor3->step(1,FORWARD,MICROSTEP);
  myMotor2->step(1,BACKWARD,MICROSTEP);

 // int j = i%200;
   // while(j == 0)
    //{
       //Serial.println("Finding Angle");
       //float a = -find_angle();
       //Serial.println("Correcting!")
       //rotate(a);
       //j = 1;
    //}
    
  }
  delay(20);
  Serial.write("Done moving left");
  find_angle(reset);
}




void goRight(int d) {        //d in millimeters

  Serial.write("Moving right");
  for (i = 0; i <= d; i++)
  {
  myMotor3->step(1,BACKWARD,DOUBLE);
  myMotor2->step(1,FORWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done moving right");

}


void goRightMicro(int d) {        

  Serial.write("Moving right");
  for (i = 0; i <= d; i++)
  {
  myMotor3->step(1,BACKWARD,MICROSTEP);
  myMotor2->step(1,FORWARD,MICROSTEP);
  }
  delay(20);
  Serial.write("Done moving right");

}


void goForward(int d) {        //d in millimeters

  Serial.write("Moving forward");
  for (i = 0; i <= d; i++)
  {
  myMotor4->step(1,FORWARD,DOUBLE);
  myMotor1->step(1,BACKWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done moving forward");

}


void goForwardMicro(int d) {        //d in millimeters

  Serial.write("Moving forward");
  for (i = 0; i <= d; i++)
  {
  myMotor4->step(1,FORWARD,MICROSTEP);
  myMotor1->step(1,BACKWARD,MICROSTEP);
  }
  delay(20);
  Serial.write("Done moving forward");

}


void goBackward(int d) {        //d in millimeters

  Serial.write("Moving backward");
  for (i = 0; i <= d; i++)
  {
  myMotor4->step(1,BACKWARD,DOUBLE);
  myMotor1->step(1,FORWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done moving backward");

}

void goBackwardMicro(int d) {        //d in millimeters

  Serial.write("Moving backward");
  for (i = 0; i <= d; i++)
  {
  myMotor4->step(1,BACKWARD,MICROSTEP);
  myMotor1->step(1,FORWARD,MICROSTEP);
  }
  delay(20);
  Serial.write("Done moving backward");

}


void rotate(float a) {   //angle in degrees

 float x = a/rotateAngle;
 int d = (int) x;
 d+=rotateCorrection;                           //correction factor

  if (a > 0)
  {
     for (i = 0; i <= d; i++)
     {  
     myMotor1->step(1,FORWARD,DOUBLE);
     myMotor2->step(1,FORWARD,DOUBLE);
     myMotor3->step(1,FORWARD,DOUBLE);
     myMotor4->step(1,FORWARD,DOUBLE);
     }
  }
   if (a < 0)
  {
    for (i = 0; i <= d; i++)
     {  
     myMotor1->step(1,BACKWARD,DOUBLE);
     myMotor2->step(1,BACKWARD,DOUBLE);
     myMotor3->step(1,BACKWARD,DOUBLE);
     myMotor4->step(1,BACKWARD,DOUBLE);
     }
  }
  
}


void rotateMicro(float a) {   //angle in degrees

 float x = a/rotateAngle;
 int d = (int) x;
 d+=rotateCorrection;                           //correction factor

  if (a > 0)
  {
     for (i = 0; i <= d; i++)
     {  
     myMotor1->step(1,FORWARD,MICROSTEP);
     myMotor2->step(1,FORWARD,MICROSTEP);
     myMotor3->step(1,FORWARD,MICROSTEP);
     myMotor4->step(1,FORWARD,MICROSTEP);
     }
  }
   if (a < 0)
  {
    for (i = 0; i <= d; i++)
     {  
     myMotor1->step(1,BACKWARD,MICROSTEP);
     myMotor2->step(1,BACKWARD,MICROSTEP);
     myMotor3->step(1,BACKWARD,MICROSTEP);
     myMotor4->step(1,BACKWARD,MICROSTEP);
     }
  }
  
}

//limit switch stuff

int checkLimits() {

 /* encode response
  0 = no limit switches
  rest are encoded as binary
  assume switches 1 and 2 are connected on the left
  and switches 3 and 4 are connected on the right
  left border = 3
  right border = 12
  need corrections = 1, 2, 4 and 8
  
  */
  
   //delay(20);
 
 sensestate1 = digitalRead(sense1);
 sensestate2 = digitalRead(sense2);
 sensestate3 = digitalRead(sense3);
 sensestate4 = digitalRead(sense4);

 
 
 int right = digitalRead(limright);
 int left =  digitalRead(limleft);

 if (right == HIGH) return 120;
 if (left == HIGH) return 30;
 
/* 
 if (sensestate1 == LOW && sensestate2 == LOW)
 {
      Serial.println("limit switches report 3 - left border");
      lcd.clear();
      delay(50);
      lcd.print("switches -  3");
      return 3;
 }
 if (sensestate3 == LOW && sensestate4 == LOW)
 {
      Serial.println("limit switches report 12 - right border");
      lcd.clear();
      delay(50);
      lcd.print("switches - 12");
      return 12;
 }
*/

 if (sensestate1 == HIGH && sensestate2 == HIGH && sensestate3 == HIGH && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 0");
      lcd.clear();
      //delay(50);
      lcd.print("no contact");
      return 0;
 }

 if (sensestate1 == LOW && sensestate2 == HIGH && sensestate3 == HIGH && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 1");
      lcd.clear();
      delay(50);
      lcd.print("switches - 1");
      return 1;
 }

 
 if (sensestate1 == HIGH && sensestate2 == LOW && sensestate3 == HIGH && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 2");
      lcd.clear();
      delay(50);
      lcd.print("switches-2");
      return 2;
 }

 
 if (sensestate1 == LOW && sensestate2 == LOW && sensestate3 == HIGH && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 3");
      lcd.clear();
      delay(50);
      lcd.print("switches 3 left");
      return 3;
 }

 
 if (sensestate1 == HIGH && sensestate2 == HIGH && sensestate3 == LOW && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 4");
      lcd.clear();
      delay(50);
      lcd.print("switch - 4");
      return 4;
 }

 
 if (sensestate1 == LOW && sensestate2 == HIGH && sensestate3 == LOW && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 5");
      lcd.clear();
      delay(50);
      lcd.print("rotate");
      return 5;
 }

 
 if (sensestate1 == HIGH && sensestate2 == LOW && sensestate3 == LOW && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 6");
      lcd.clear();
      delay(50);
      lcd.print("rotate");
      return 6;
 }

 
 if (sensestate1 == LOW && sensestate2 == LOW && sensestate3 == LOW && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 7");
      lcd.clear();
      delay(50);
      lcd.print("three are ON");
      return 7;
 }

 
 if (sensestate1 == HIGH && sensestate2 == HIGH && sensestate3 == HIGH && sensestate4 == LOW)
 {
      Serial.println("limit switches report 8");
      lcd.clear();
      delay(50);
      lcd.print("switch 8");
      return 8;
 }

 if (sensestate1 == LOW && sensestate2 == HIGH && sensestate3 == HIGH && sensestate4 == LOW)
 {
      Serial.println("limit switches report 9");
      lcd.clear();
      delay(50);
      lcd.print("pin9,10 is ON");
      return 9;
 }

 
 if (sensestate1 == HIGH && sensestate2 == LOW && sensestate3 == HIGH && sensestate4 == LOW)
 {
      Serial.println("limit switches report 10");
      lcd.clear();
      delay(50);
      lcd.print("pin11,12 is ON");
      return 10;
 }

 
 if (sensestate1 == LOW && sensestate2 == LOW && sensestate3 == HIGH && sensestate4 == LOW)
 {
      Serial.println("limit switches report 11");
      lcd.clear();
      delay(50);
      lcd.print("pin9,11,10 is ON");
      return 11;
 }

 
 if (sensestate1 == HIGH && sensestate2 == HIGH && sensestate3 == LOW && sensestate4 == LOW)
 {
      Serial.println("limit switches report 12");
      lcd.clear();
      delay(50);
      lcd.print("switches 12 right");
      return 12;
 }

 
 if (sensestate1 == LOW && sensestate2 == HIGH && sensestate3 == LOW && sensestate4 == LOW)
 {
      Serial.println("limit switches report 13");
      lcd.clear();
      delay(50);
      lcd.print("pin9,11,12");
      return 13;
 }

 
 if (sensestate1 == HIGH && sensestate2 == LOW && sensestate3 == LOW && sensestate4 == LOW)
 {
      Serial.println("limit switches report 14");
      lcd.clear();
      delay(50);
      lcd.print("pin10,11,12");
      return 14;
 }

 
 if (sensestate1 == LOW && sensestate2 == LOW && sensestate3 == LOW && sensestate4 == LOW)
 {
      Serial.println("limit switches report 15");
      lcd.clear();
      delay(50);
      lcd.print("all ON");
      return 15;
 }

 
}


//calibrate movement

void calibrateField1() {   //calibrate left right orientation on the field

  movestep = 500;
  int c = checkLimits();
  //first go to the left end
  while(c != 30) {         //3 is left border
    c = checkLimits();
    if (c==30) break;
    if (c==3) movestep = 1;
    lcd.print("check");
    //delay(50);
    lcd.clear();
    lcd.print(c);
    if (c==1) {         //correction case 1
      rotate(-1.0);
      continue;
    }
    if (c==2) {         //correction case 2
      rotate(1.0);
      continue;
    }
    
    if (movestep>1){
      goLeft(movestep);
    } else {
      goLeftMicro(movestep);
    }

    if (movestep ==500) movestep = 10;
    
  }

  //now go right
  movestep = 1000;
  c = checkLimits();
  
  while(c != 120) {
    c = checkLimits();
    if (c==120) break;
    if (c==12) movestep = 1;
    if (c==4) {         //correction case 1
      rotate(1.0);
      continue;
    }
    if (c==8) {         //correction case 2
      rotate(-1.0);
      continue;
    }
    if (movestep>1){
      goRight(movestep);
    } else {
      goRightMicro(movestep);
    }
    
    fieldwidth+=movestep;   //increment field measurement
    if (movestep ==1000) movestep = 10;
    
  }
  //move to centre
  goLeft(fieldwidth/2);
  lcd.clear();
}

void platform_turn(int d){
  Serial.write("turning platform");
  for (int i=0; i <= d; i++)
  {
    myMotor5 -> step(1,BACKWARD,DOUBLE);
    delay(20);
  }
}

void turnC(int d) {        //d in millimeters
  Serial.write("Moving right");
  for (int i = 0; i <= d; i++)
  {
    myMotor6->step(1,FORWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done turning right");
}

void turnCCW(int d) {        //d in millimeters
  Serial.write("Moving right");
  for (int i = 0; i <= d; i++)
  {
    myMotor6->step(1,BACKWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done turning left");
}

void turn_knob() {
  turn1 = first*200;
  turn2 = second*200;
  turn3 = third*200;
  turn4 = fourth*200;
  turn5 = fifth*200;
  
  turnC(turn1);
  delay(500);
  turnCCW(turn2);
  delay(500);
  turnC(turn3);
  delay(500);
  turnCCW(turn4);
  delay(500);
  turnC(turn5);
}

void translate(unsigned int r) {        //translate recieved int into digits
  fifth = r %10;
  fourth = (r % 100)/10;
  third = (r % 1000)/100;
  second = (r % 10000)/1000;
  first = (r % 100000)/10000;
  
}


void parallelize() {
  double error = sonarRight()-sonarLeft();
  
  while((error>1) || (error<-1)) {
  if((sonarRight()<sonarLeft()) ) { //if right is longer
    myMotor1->step(1,FORWARD,DOUBLE);
     myMotor2->step(1,FORWARD,DOUBLE);
     myMotor3->step(1,FORWARD,DOUBLE);
     myMotor4->step(1,FORWARD,DOUBLE);
  }
  else if((sonarLeft()<sonarRight()) ) { //if left is longer
    myMotor1->step(1,BACKWARD,DOUBLE);
     myMotor2->step(1,BACKWARD,DOUBLE);
     myMotor3->step(1,BACKWARD,DOUBLE);
     myMotor4->step(1,BACKWARD,DOUBLE);
  }

  }
  
}

/*MAIN SETUP*******************************************************************/
void setup() {
  Serial.begin(9600);
  Wire.begin();
  lcd.init();
  lcd.begin(16,2);
  lcd.backlight();
  lcd.setCursor(0,0);
  //while (!Serial);    // wait for the serial port to open

//limit switch initialize
  pinMode(sense1, INPUT);
  pinMode(sense2, INPUT);
  pinMode(sense3, INPUT);
  pinMode(sense4, INPUT);
  pinMode(limleft, INPUT);
  pinMode(limright, INPUT);


//testing stuff 1

//  while (1)
//  { int x = checkLimits();
//     delay(1000);
//     lcd.clear();
//     lcd.print(x);
//     if(x==15) break;
//  }
 
//testingg



  
  pinMode(DCmPin, OUTPUT); //Set pin as output
  trigger.attach(10);
  
  //Firing two darts
  delay(3000);
  Serial.println("Init Firing");
  digitalWrite(DCmPin, HIGH); //start motors 
  Fire_dart(2);
  digitalWrite(DCmPin, LOW); //stop the motor
  lcd.clear();
 

  // initialize IMU or Gyro device
  Serial.println("Initializing IMU device...");
  
  

  // Set the accelerometer range to 250 degrees/second

  
  //Initilize Stepper Motors for Movement!
  Serial.write("Init Motion");
  AFMSbot.begin();
  myMotor1->setSpeed(Max_speed);
  myMotor2->setSpeed(Max_speed);

  AFMSmid.begin();
  myMotor3->setSpeed(Max_speed);
  myMotor4->setSpeed(Max_speed);
  count = 0;
  
  AFMStop.begin();
  myMotor5->setSpeed(Max_speed);
  myMotor6->setSpeed(Max_speed);

  //get platform to be in position
  platform_turn(50);
  delay(2000);
  //slave_work();
  //delay(1000);
  /*
  //calibrate field 1
  lcd.print("calibrating");
  delay(2000);
  calibrateField1();
  lcd.clear();
  */
  //Starting Stages after firing the two darts!
  stage1 = true;
  stage2 = false;
  stage3 = false;
  stage4 = false;
 }
 

 
 
/*MAIN LOOP**************************************************************************************/
void loop() {
  Serial.print("Running");
  // put your main code here, to run repeatedly:
if(stage1) {
    move_to_stage1();
    do_stage1();
  }
  if(stage2){
    move_to_stage2();
    do_stage2();
  }
  if(stage3){
    move_to_stage3();
    do_stage3();
  }
  if(stage4){
    move_to_stage4();
    do_stage4();
  }
}