//Master_Transmitter
 #include <Wire.h>

short first, second, third, fourth, fifth;

void translate(unsigned int r) {        //translate recieved int into digits
  first = r %10;
  second = (r % 100)/10;
  third = (r % 1000)/100;
  fourth = (r % 10000)/1000;
  fifth = (r % 100000)/10000;
  
}

void setup() 
{
  Wire.begin();
   Serial.begin(9600);
}

void loop() 
{
  //Wire.beginTransmission(8); 
  
  //Wire.write('s');       
  //Wire.endTransmission(); 
  unsigned int output = 0;
  
  Wire.requestFrom(8, 1);   // the first byte
 while(Wire.available())
 { 
   unsigned char received = Wire.read();
   output = received;
 }
 
 for (int i = 0 ; i < 3 ; i++) // next 3 bytes
 {
    Wire.requestFrom(8, 1);   
    while(Wire.available())
    { 
       unsigned char received = Wire.read();
       output |= (received << 8);

    }
 }
 Serial.println(output);

  
}