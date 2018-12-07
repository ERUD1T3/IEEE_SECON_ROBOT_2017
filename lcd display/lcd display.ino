//### What is this repository for?  -- starter code for lcd display###

// Code

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  //lcd.init();                      // initialize the lcd 
  lcd.init();
  lcd.begin(16,2);
  // Print a message to the LCD.
  lcd.setCursor(0,0);
  lcd.print("starting...");
  //lcd.backlight();
  
  
}

void displayCode(int c);

void loop() {
  
  lcd.clear();
  lcd.noBacklight();
  delay(2000);
  lcd.setCursor(0,0);
  lcd.print("11111");
  lcd.backlight();
}


void displayCode(int c) {

  lcd.clear();
  lcd.noBacklight();
  delay(2000);
  lcd.setCursor(0,0);
  lcd.print(c);
  lcd.backlight();

}