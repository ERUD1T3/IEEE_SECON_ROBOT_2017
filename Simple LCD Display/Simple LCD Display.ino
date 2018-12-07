#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  //lcd.init();                      // initialize the lcd 
  lcd.init();
  lcd.begin(16,2);
  // Print a message to the LCD.
  lcd.setRGB(50,0,0);
  lcd.setCursor(0,0);
  lcd.print("starting...");
  //lcd.backlight();
  
  
}


void loop()
{

  delay(2000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("11111");
  delay(4000);
  lcd.backlight();
  
}