#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <Arduino_FreeRTOS.h>
#include <task.h>

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
QueueHandle_t xMutex;



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
  xMutex = xSemaphoreCreateMutex();  // create mutex for LCD

  // Create tasks
  xTaskCreate(tempAndHumiTask, "Temperature and Humidity", 1000, NULL, 1, NULL);
  xTaskCreate(smokeDetectionTask, "Smoke Detection", 1000, NULL, 2, NULL);
  xTaskCreate(displayUpdateTask, "Display Update", 1000, NULL, 1, NULL);
  xMutex = xQueueCreate(1, 1);  // This creates a queue of length 1 and size 1 byte, effectively behaving like a mutex.
  uint8_t dummy = 1;
  xQueueSend(xMutex, &dummy, portMAX_DELAY);
  uint8_t dummy;
  xQueueReceive(xMutex, &dummy, 0);

}


void loop() {
vTaskDelay(portMAX_DELAY);
}

void tempAndHumiTask(void* pvParameters) {
  for(;;) {
    unsigned long currentMillis = millis();

  // Only attempt to read from the DHT11 at the interval
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the current time to compare in the next loop iteration

    float humi = dht.readHumidity();          // Read humidity
    float tempC = dht.readTemperature();      // Read temperature as Celsius

    
    }
  }

    
    vTaskDelay(pdMS_TO_TICKS(interval)); // Delay for the interval
  }
}

void smokeDetectionTask(void* pvParameters) {
  for(;;) {
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

    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for stability
  }
}

void displayUpdateTask(void* pvParameters) {
  for(;;) {
    if (xMutex != NULL) {
      if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE) {
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

        xSemaphoreGive(xMutex);  // Give the mutex back
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for half a second
  }
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
