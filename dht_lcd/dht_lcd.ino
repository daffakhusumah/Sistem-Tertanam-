#include <LiquidCrystal_I2C.h>
#include "DHT.h"

#define DHTPIN 2          // what digital pin the DHT11 is connected to
#define DHTTYPE DHT11     // DHT11 sensor is being used

LiquidCrystal_I2C lcd(0x3F, 20, 4);  // set the LCD address to 0x3F for a 16 chars and 2 line display
DHT dht(DHTPIN, DHTTYPE);

unsigned long previousMillis = 0;        // will store last time temperature was updated
const long interval = 2000;              // interval at which to read sensor (milliseconds, 2000ms = 2s)

void setup() {
  dht.begin();              // initialize the sensor
  lcd.init();               // initialize the lcd
  lcd.backlight();          // turn on the backlight
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you read the sensor
    previousMillis = currentMillis;

    // Reading temperature or humidity takes about 250 milliseconds
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float humi = dht.readHumidity();          // Read humidity
    float tempC = dht.readTemperature();      // Read temperature as Celsius

    lcd.clear();

    // Check if any reads failed and exit early (to try again).
    if (isnan(humi) || isnan(tempC)) {
      lcd.print("Failed to read from DHT sensor!");
      return;
    }

    // Display the temperature
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(tempC);     // print the temperature
    lcd.print((char)223); // print degree symbol
    lcd.print("C");

    // Display the humidity
    lcd.setCursor(0, 1);
    lcd.print("Humi: ");
    lcd.print(humi);      // print the humidity
    lcd.print("%");
  }
}
