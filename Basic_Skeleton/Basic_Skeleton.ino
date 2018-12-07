#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);
unsigned long time;
long time_passed;

bool stage1;
bool stage2;
bool stage3;
bool stage4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  stage1 = true;
  stage2 = false;
  stage3 = false;
  stage4 = false;
  
 }

void move_to_stage1() 
{
  lcd.clear();
  lcd.print("STAGE 1");
  delay(2000);
  check_time();
}

void do_stage1() 
{
  lcd.clear();
  lcd.print("Doing STAGE 1");
  delay(2000);
  check_time();
  stage1 = false;
  stage2 = true;
}

void move_to_stage2() 
{
  lcd.clear();
  lcd.print("STAGE 2");
  delay(2000);
  check_time();
  delay(2000); 
}

void do_stage2() 
{
  lcd.clear();
  lcd.print("Doing STAGE 1");
  delay(2000);
  check_time();
  stage2 = false;
  stage3 = true;
}

void move_to_stage3() 
{
  lcd.clear();
  lcd.print("STAGE 3");
  delay(2000);
  check_time();
  delay(5000);
}

void do_stage3() 
{
  lcd.clear();
  lcd.print("Doing STAGE 1");
  delay(2000);
  check_time();
  stage3 = false;
  stage4 = true;
  
}

void move_to_stage4() 
{
  lcd.clear();
  lcd.print("STAGE 4");
  delay(2000);
  check_time();
  delay(5000);
}

void do_stage4() 
{
  lcd.clear();
  lcd.print("Doing STAGE 1");
  delay(2000);
  check_time();
  stage4 = false;     //end test
}

void check_time()     //skip to stage 4 after a certain amount of time passed
{
  lcd.clear();
  time_passed = millis();
  lcd.println(time_passed);
  
  if (time_passed > 210000){
  stage1 = false;
  stage2 = false;
  stage3 = false;
  stage4 = true;
  }

}

void loop()     //change between stages
{
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

