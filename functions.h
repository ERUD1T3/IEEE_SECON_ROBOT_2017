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
  //slave work!
  //get_5_code
  while (haha == true)
  {
    Serial.println("looking for values!");
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
  if (time > 10000)
   {
     haha = false;
   }
 }
 
 if(output == 0)
 {
  haha = true;
 }
 
  translate(output);
  Serial.print(first);
  Serial.print(second);
  Serial.print(third);
  Serial.print(fourth);
  Serial.print(fifth);
  Serial.println();
 
}
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
  delay(500);
  platform_turn();
  delay(1000);
  check_time();
  goRight(100);
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
  goLeft(100);
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
  lcd.clear();
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
  
   delay(20);
 
 sensestate1 = digitalRead(sense1);
 sensestate2 = digitalRead(sense2);
 sensestate3 = digitalRead(sense3);
 sensestate4 = digitalRead(sense4);

 
 
 int right = digitalRead(limright);
 int left =  digitalRead(limleft);

 if (right == HIGH) return 12;
 if (left == HIGH) return 3;
 
 
 if (sensestate1 == HIGH && sensestate2 == HIGH)
 {
      Serial.println("limit switches report 3 - left border");
      lcd.clear();
      delay(50);
      lcd.print("left side");
      return 3;
 }
 if (sensestate3 == HIGH && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 12 - right border");
      lcd.clear();
      delay(50);
      lcd.print("right side");
      return 12;
 }

 if (sensestate1 == LOW && sensestate2 == LOW && sensestate3 == LOW && sensestate4 == LOW)
 {
      Serial.println("limit switches report 0");
      lcd.clear();
      delay(50);
      lcd.print("all low");
      return 0;
 }

 if (sensestate1 == HIGH && sensestate2 == LOW && sensestate3 == LOW && sensestate4 == LOW)
 {
      Serial.println("limit switches report 1");
      lcd.clear();
      delay(50);
      lcd.print("pin9 is ON");
      return 1;
 }

 
 if (sensestate1 == LOW && sensestate2 == HIGH && sensestate3 == LOW && sensestate4 == LOW)
 {
      Serial.println("limit switches report 2");
      lcd.clear();
      delay(50);
      lcd.print("pin11 is ON");
      return 2;
 }

 
 if (sensestate1 == HIGH && sensestate2 == HIGH && sensestate3 == LOW && sensestate4 == LOW)
 {
      Serial.println("limit switches report 3");
      lcd.clear();
      delay(50);
      lcd.print("left side");
      return 3;
 }

 
 if (sensestate1 == LOW && sensestate2 == LOW && sensestate3 == HIGH && sensestate4 == LOW)
 {
      Serial.println("limit switches report 4");
      lcd.clear();
      delay(50);
      lcd.print("pin10 is ON");
      return 4;
 }

 
 if (sensestate1 == HIGH && sensestate2 == LOW && sensestate3 == HIGH && sensestate4 == LOW)
 {
      Serial.println("limit switches report 5");
      lcd.clear();
      delay(50);
      lcd.print("rotate");
      return 5;
 }

 
 if (sensestate1 == LOW && sensestate2 == HIGH && sensestate3 == HIGH && sensestate4 == LOW)
 {
      Serial.println("limit switches report 6");
      lcd.clear();
      delay(50);
      lcd.print("rotate");
      return 6;
 }

 
 if (sensestate1 == HIGH && sensestate2 == HIGH && sensestate3 == HIGH && sensestate4 == LOW)
 {
      Serial.println("limit switches report 7");
      lcd.clear();
      delay(50);
      lcd.print("three are ON");
      return 7;
 }

 
 if (sensestate1 == LOW && sensestate2 == LOW && sensestate3 == LOW && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 8");
      lcd.clear();
      delay(50);
      lcd.print("pin12 is ON");
      return 8;
 }

 if (sensestate1 == HIGH && sensestate2 == LOW && sensestate3 == LOW && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 9");
      lcd.clear();
      delay(50);
      lcd.print("pin9,10 is ON");
      return 9;
 }

 
 if (sensestate1 == LOW && sensestate2 == HIGH && sensestate3 == LOW && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 10");
      lcd.clear();
      delay(50);
      lcd.print("pin11,12 is ON");
      return 10;
 }

 
 if (sensestate1 == HIGH && sensestate2 == HIGH && sensestate3 == LOW && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 11");
      lcd.clear();
      delay(50);
      lcd.print("pin9,11,10 is ON");
      return 11;
 }

 
 if (sensestate1 == LOW && sensestate2 == LOW && sensestate3 == HIGH && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 12");
      lcd.clear();
      delay(50);
      lcd.print("rightside");
      return 12;
 }

 
 if (sensestate1 == HIGH && sensestate2 == LOW && sensestate3 == HIGH && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 13");
      lcd.clear();
      delay(50);
      lcd.print("pin9,11,12");
      return 13;
 }

 
 if (sensestate1 == LOW && sensestate2 == HIGH && sensestate3 == HIGH && sensestate4 == HIGH)
 {
      Serial.println("limit switches report 14");
      lcd.clear();
      delay(50);
      lcd.print("pin10,11,12");
      return 14;
 }

 
 if (sensestate1 == HIGH && sensestate2 == HIGH && sensestate3 == HIGH && sensestate4 == HIGH)
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

  int c = checkLimits();
  //first go to the left end
  while(c != 3) {
    c = checkLimits();
    if (c==1) {         //correction case 1
      rotate(1.0);
      continue;
    }
    if (c==2) {         //correction case 2
      rotate(-1.0);
      continue;
    }
    //goLeft(1);
    
  }

  //now go right
  c = checkLimits();
  
  while(c != 12) {
    c = checkLimits();
    if (c==4) {         //correction case 1
      rotate(-1.0);
      continue;
    }
    if (c==8) {         //correction case 2
      rotate(1.0);
      continue;
    }
    //goRight(1);
    fieldwidth++;   //and the
  }
  //move to centre
  //goLeft(fieldwidth/2);
  lcd.clear();
  

  
}

void platform_turn(){
  Serial.write("turning platform");
  for (int i = 0; i <= 50; i++)
  {
    myMotor5 -> step(1,FORWARD,DOUBLE);
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