#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F,20,4);
//LiquidCrystal_I2C lcd(0x27,20,4);
void setup()
{
 lcd.init();
 lcd.init();
 lcd.backlight();
}
void loop()
{
 lcd.setCursor(0,0);
 lcd.print("Hello World");
 lcd.setCursor(0,1);
 lcd.print("Kolom ke 2");
 lcd.scrollDisplayRight();
 delay(500);
 //lcd.scrollDisplayLeft();
 //delay(500);
}
