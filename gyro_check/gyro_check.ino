#include "CurieIMU.h"
#define reset 0
unsigned long currentMillis, previousMillis = 0, currentTime, delta_time, previousTime = 0;

float gx, gy, gz, ax, ay, az;
float gx_anew, gy_anew, gz_anew;
float thresh1 = 0.8;                    //offset threshold for gyroscope
float gx_velocity, gy_velocity, gz_velocity;
float gx_angle = 0, gy_angle = 0, gz_angle = 0; 
float gx_sum, gy_sum,gz_sum, zSum = 0.0;

float getAbs(float x) {
  if (x >=0.0) return x;
  else return -1.0*x;
}

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  
  // initialize device
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
}

void loop() {
  find_angle(reset);
  //find_angle(1);
}

void find_angle(int d){
  
    
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

        

        if(getAbs(gz)>=thresh1) {
          zSum+=gz;
        }
        else {
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
  if (currentMillis - previousMillis >= 1000){
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

    if( d== 0)
        {
          gx_angle = 0;
          gy_angle = 0;
          gz_angle = 0;
          }


  
  }
  


}
