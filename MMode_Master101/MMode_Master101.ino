//This program controls the robot for IEEE Robotics
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include "CurieIMU.h"
#define reset 0

/*VARIABLES***************************************************************/
Servo trigger;
Servo platform;
int DCmPin(13);
LiquidCrystal_I2C lcd(0x27,16,2);

//Gyro variables
unsigned long currentMillis, previousMillis = 0, currentTime, delta_time, previousTime = 0;
float gx, gy, gz, ax, ay, az;
float gx_anew, gy_anew, gz_anew;
float thresh1 = 0.8;                    //offset threshold for gyroscope
float gx_velocity, gy_velocity, gz_velocity;
float gx_angle = 0, gy_angle = 0, gz_angle = 0; 
float gx_sum, gy_sum,gz_sum, zSum = 0.0;

//Variables form slave!
unsigned short first, second, third, fourth, fifth;
unsigned int output = 0;

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
bool getcode;

Adafruit_MotorShield AFMSbot = Adafruit_MotorShield(0x60);    //motor control
Adafruit_MotorShield AFMSmid = Adafruit_MotorShield(0x61); 
Adafruit_MotorShield AFMStop = Adafruit_MotorShield(0x63); 

Adafruit_StepperMotor *myMotor3 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *myMotor2 = AFMSbot.getStepper(200, 2);
Adafruit_StepperMotor *myMotor4 = AFMSmid.getStepper(200, 1);
Adafruit_StepperMotor *myMotor1 = AFMSmid.getStepper(200, 2);
Adafruit_StepperMotor *knob_turner = AFMStop.getStepper(200, 1);


/*FUNCTIONS*****************************************************************************/
void Fire_dart(int n) //fires n darts 
{
  lcd.print("FIRE!!!!");
     for (int i(1); i <= n; i++)
     {
          trigger.write(50);
          delay(1000);
          trigger.write(150);
          delay(500);
          trigger.write(50);
     }
   lcd.print("TargetDestroyed");
}

void move_to_stage1() 
{
  lcd.clear();
  lcd.print("STAGE 1");
  delay(2000);
  check_time();
  goLeft(100);
}

void do_stage1() 
{
  lcd.clear();
  lcd.print("Doing STAGE 1");
  getcode = true;
  
  //Request values from the slave unit
  while(getcode == true){
    Wire.requestFrom(8,1);
    while(Wire.available())
    {
      unsigned char received = Wire.read();
      output = received;
    }
    for (int i = 0; i< 3; i++)
    {
       Wire.requestFrom(8, 1);   
      while(Wire.available())
      { 
         unsigned char received = Wire.read();
         output |= (received << 8);
  
      }
    }
    Serial.println(output);
    getcode = false;
  }
  //translate output
  translate(output);
  
  unsigned int sum = first*10000 + second*1000 + third*100 + fourth*10 + fifth;
  lcd.print(sum);
  delay(2000);    //display value for 2 seconds
  lcd.clear();
  delay(2000);
  check_time();
  goRight(100);
  stage1 = false;
  stage2 = true;
}

//No stage 2!!!! >.<
void move_to_stage2() 
{
  lcd.clear();
  lcd.print("STAGE 2");
  delay(2000);
  check_time();
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

void move_to_stage3() 
{
  lcd.clear();
  lcd.print("STAGE 3");
  delay(2000);
  check_time();
  goRight(100);
  delay(5000);
  platform.write(90);
  delay(2000);
}

void do_stage3() 
{
  lcd.clear();
  lcd.print("Doing STAGE 3");
  delay(2000);
  check_time();
  turn_knob();
  stage3 = false;
  stage4 = true;
}

void move_to_stage4() 
{
  lcd.clear();
  lcd.print("STAGE 4");
  delay(2000);
  check_time();
  goForward(100);
  delay(5000);
}

void do_stage4() 
{
  lcd.clear();
  lcd.print("Doing STAGE 4");
  delay(2000);
  check_time();
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

void translate(unsigned int r) {        //translate recieved int into digits in stage 1
  first = r %10;
  second = (r % 100)/10;
  third = (r % 1000)/100;
  fourth = (r % 10000)/1000;
  fifth = (r % 100000)/10000;
  
}

float getAbs(float x) {
  if (x >=0.0) return x;
  else return -1.0*x;
}

float find_angle(int d){    
    //find time difference between measures
 currentTime = millis();
 delta_time = (currentTime - previousTime);
 if (delta_time > 0) 
 {
   previousTime = currentTime;
   //find angular velocity
   //CurieIMU.readGyroScaled(gx, gy, gz);
   //CurieIMU.readGyro(gx, gy, gz);
   CurieIMU.readMotionSensorScaled(ax,ay,az,gx, gy, gz);
   
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
     Serial.print(gx);
     Serial.print("\t");

     Serial.print(gy);
     Serial.print("\t");
     Serial.print(gz);
     Serial.print("\t");
    
     Serial.print(gx_velocity);
     Serial.print("\t");

     Serial.print(gx_anew);
     Serial.print("\t");
  
     Serial.print(currentTime);
     Serial.print("\t");

     Serial.print(previousTime);
     Serial.print("\t");
  
     Serial.print(delta_time);
     Serial.print("\t");
  
     Serial.print("g:\t");         //display angle offset
     Serial.print(gx_angle);
     Serial.print("\t");
     Serial.print(gy_angle);
     Serial.print("\t");
     Serial.print(gz_angle);
     Serial.print("\t");
     Serial.print(zSum/1000.0);
     Serial.println();

    if( d == 0)
        {
          gx_angle = 0;
          gy_angle = 0;
          gz_angle = 0;
         }

     return gz_angle;
  }
}

void goLeft(int d) {        //d in millimeters

  Serial.write("Moving left");
  for (i = 0; i <= d; i++)
  {
  myMotor1->step(1,FORWARD,DOUBLE);
  myMotor4->step(1,BACKWARD,DOUBLE);

  int j = i%200;
    while(j == 0)
    {
       Serial.println("Finding Angle");
       float a = -find_angle();
       Serial.println("Correcting!")
       rotate(a);
       j = 1;
    }
    
  }
  delay(20);
  Serial.write("Done moving left");
  find_angle(reset)
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

//Stage 3: Turning the Knob Functions
void turnC(int d) {        //d in millimeters
  Serial.write("Turning clockwise");
  for (int i = 0; i <= d; i++)
  {
  knob_turner -> step(1,FORWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done turning right");
}

void turnCCW(int d) {        //d in millimeters
  Serial.write("turning counterclockwise");
  for (int i = 0; i <= d; i++)
  {
  knob_turner -> step(1,BACKWARD,DOUBLE);
  }
  delay(20);
  Serial.write("Done turning left");
}

void turn_knob() {
  turnC(first*200);
  delay(500);
  turnCCW(second*200);
  delay(500);
  turnC(third*200);
  delay(500);
  turnCCW(fourth*200);
  delay(500);
  turnC(fifth*200);
}

/*MAIN SETUP*******************************************************************/

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.begin(16,2);
  lcd.backlight();
  while (!Serial);    // wait for the serial port to open
  
  pinMode(DCmPin, OUTPUT); //Set pin as output
  trigger.attach(10);
  platform.attach(9);
  
  //Firing two darts
  delay(3000);
  Serial.println("Init Firing");
  digitalWrite(DCmPin, HIGH); //start motors 
  Fire_dart(2);
  digitalWrite(DCmPin, LOW); //stop the motor
 
  // initialize IMU or Gyro device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();
  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  
  Serial.println(CurieIMU.getGyroOffset(X_AXIS));
  Serial.println(CurieIMU.getGyroOffset(Y_AXIS));
  Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  

  // Set the accelerometer range to 250 degrees/second
  CurieIMU.setGyroRange(250);
  CurieIMU.setGyroRate(1600);
  CurieIMU.setAccelerometerRange(2);
  
  //Initilize Stepper Motors for Movement!
  Serial.write("Init Motion");
  AFMSbot.begin();
  myMotor1->setSpeed(Max_speed);
  myMotor2->setSpeed(Max_speed);

  AFMSmid.begin();
  myMotor3->setSpeed(Max_speed);
  myMotor4->setSpeed(Max_speed);
  
  AFMtop.begin();
  knob_turner -> setSpeed(Max_speed);
  count = 0;
  
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
  }s
  if(stage4){
    move_to_stage4();
    do_stage4();
  }
}