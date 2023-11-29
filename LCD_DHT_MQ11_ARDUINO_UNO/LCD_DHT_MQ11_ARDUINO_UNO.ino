#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"

#define DHTPIN 2          // what digital pin the DHT11 is connected to
#define DHTTYPE DHT11     // DHT11 sensor is being used

LiquidCrystal_I2C lcd(0x3F,20,4);
DHT dht(DHTPIN, DHTTYPE);

const long interval = 2000; // interval at which to read sensor (milliseconds, 2000ms = 2s)
unsigned long previousMillis = 0; // stores last time temperature was updated
int sensorThres = 400; // Adjust the threshold value based on your sensor's characteristics
int sensorSafeRange = 50; // Range of safe values to prevent flickering between states

// New variables to handle the debouncing effect
bool isGasDetected = false;
unsigned long gasDetectedStartTime = 0;
unsigned long minTimeGasDetected = 2000; // e.g., gas must be detected for at least 2 seconds

// smoke detector pins and threshold
int redLed = 10;
int greenLed = 13;
int buzzer = 8;
int smokeA0 = A0;


void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(smokeA0, INPUT);

  dht.begin(); // Initialize DHT sensor
  Serial.begin(9600); // Initialize serial communication
  lcd.init();
  lcd.backlight(); // turn on the backlight of LCD
  lcd.begin(16,2);
}

void loop() {
  unsigned long currentMillis = millis();

  // Only attempt to read from the DHT11 at the interval
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the current time to compare in the next loop iteration

    float humi = dht.readHumidity();          // Read humidity
    float tempC = dht.readTemperature();      // Read temperature as Celsius

    lcd.clear();

    if (isnan(humi) || isnan(tempC)) {
      Serial.println("Failed to read from DHT sensor!"); // Log failure
      lcd.setCursor(0, 0);
      lcd.print("DHT Reading failed");
    } else {
      // When the readings are valid, display them
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(tempC);
      lcd.print((char)223);
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print("Humi: ");
      lcd.print(humi);
      lcd.print("%");
    }
  }

  // Smoke/gas detection functionality
  int analogSensor = analogRead(smokeA0);
  Serial.print("Pin A0: ");
  Serial.println(analogSensor);

  // Check gas levels and handle the alert system with debouncing
  if (analogSensor > sensorThres) {
    // If gas is detected and it's the first detection, record the time
    if (!isGasDetected) {
      isGasDetected = true;
      gasDetectedStartTime = millis();
    }
    // Check if gas has been detected for longer than the minimum time
    else if (millis() - gasDetectedStartTime > minTimeGasDetected) {
      activateAlert();
    }
  }
  else if (isGasDetected && analogSensor < sensorThres - sensorSafeRange) {
    // If the level drops to a safe range, deactivate alert
    isGasDetected = false;
    deactivateAlert();
  }

  delay(100); // delay for stability
}

void activateAlert() {
  // Activate warning devices
  digitalWrite(redLed, HIGH);   // Red LED indicates danger
  digitalWrite(greenLed, LOW);  // Green LED off
  tone(buzzer, 1000, 200);       // Sound buzzer
}

void deactivateAlert() {
  // Deactivate warning devices
  digitalWrite(redLed, LOW);    // No danger, so red LED off
  digitalWrite(greenLed, HIGH); // Indicate safe with green LED
  noTone(buzzer);               // Stop buzzer
}
