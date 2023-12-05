#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <DHT.h>

#define DHTPIN 2  // Define the pin for DHT11 data
#define DHTTYPE DHT11  // Define the type of DHT sensor

LiquidCrystal_I2C lcd(0x3F, 20, 4);

int redLed = 10;
int greenLed = 13;
int buzzer = 8;
int smokeA0 = A0;
int sensorThres = 100;

volatile int analogSensorValue = 0;

SemaphoreHandle_t lcdSemaphore;
SemaphoreHandle_t serialSemaphore;

DHT dht(DHTPIN, DHTTYPE);

int buttonPin = 3;  // Define the push button pin
int buttonState = 0;
int lastButtonState = 0;
bool priorityChanged = false;

void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(smokeA0, INPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Configure the button pin as an input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(buttonPin), changePriority, FALLING); // Attach the interrupt
  
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.begin(16, 2);

  dht.begin();

  // Create semaphores
  lcdSemaphore = xSemaphoreCreateMutex();
  serialSemaphore = xSemaphoreCreateMutex();

  // Creating tasks
  xTaskCreate(ReadSensorTask, "ReadSensor", 128, NULL, 1, NULL);
  xTaskCreate(DisplayTask, "Display", 128, NULL, 2, NULL);
  xTaskCreate(AlertTask, "Alert", 128, NULL, 3, NULL);

  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty. Tasks will handle everything.
}

void changePriority() {
  buttonState = digitalRead(buttonPin);
  
  if (buttonState == LOW && lastButtonState == HIGH) {
    priorityChanged = !priorityChanged;
  }

  lastButtonState = buttonState;
}

void ReadSensorTask(void *pvParameters) {
  for (;;) {
    if (priorityChanged) {
      // Read temperature and humidity from DHT11
      float humidity = dht.readHumidity();
      float temperature = dht.readTemperature();

      // Access serial with semaphore
      if (xSemaphoreTake(serialSemaphore, (TickType_t)10) == pdTRUE) {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print("°C, Humidity: ");
        Serial.print(humidity);
        Serial.println("%");
        xSemaphoreGive(serialSemaphore);
      }
    } else {
      analogSensorValue = analogRead(smokeA0);

      // Access serial with semaphore
      if (xSemaphoreTake(serialSemaphore, (TickType_t)10) == pdTRUE) {
        Serial.print("Pin A0: ");
        Serial.println(analogSensorValue);
        xSemaphoreGive(serialSemaphore);
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void DisplayTask(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(lcdSemaphore, (TickType_t)10) == pdTRUE) {
      lcd.clear();
      if (priorityChanged) {
        lcd.print("Temp & Humidity:");
        // Display temperature and humidity on the LCD
        float humidity = dht.readHumidity();
        float temperature = dht.readTemperature();
        lcd.setCursor(0, 1);
        lcd.print("Temp: ");
        lcd.print(temperature);
        lcd.print("C");
        lcd.setCursor(0, 2);
        lcd.print("Humidity: ");
        lcd.print(humidity);
        lcd.print("%");
      } else {
        lcd.print("Smoke Level:");

        int scaledValue = map(analogSensorValue, 0, 1023, 0, 100); // Scale the value to 0-100
        lcd.setCursor(0, 1);  // Move cursor to the second line
        lcd.print(scaledValue);
        lcd.print("%   ");  // Print percentage and clear any previous characters
      }
      xSemaphoreGive(lcdSemaphore);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay to allow for easier reading
  }
}

void AlertTask(void *pvParameters) {
  for (;;) {
    if (analogSensorValue - 400 > sensorThres) {
      digitalWrite(redLed, HIGH);

      if (xSemaphoreTake(lcdSemaphore, (TickType_t)10) == pdTRUE) {
        lcd.setCursor(0, 3);
        lcd.print("Alert....!!!");
        xSemaphoreGive(lcdSemaphore);
      }

      // Serial monitor printing
      if (xSemaphoreTake(serialSemaphore, (TickType_t)10) == pdTRUE) {
        Serial.println("Alert....!!!");
        xSemaphoreGive(serialSemaphore);
      }

      digitalWrite(greenLed, LOW);  // Turn off the green LED
      tone(buzzer, 1000, 200);
    } else {
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, HIGH);  // Turn on the green LED at full brightness

      if (xSemaphoreTake(lcdSemaphore, (TickType_t)10) == pdTRUE) {
        lcd.setCursor(0, 3);
        lcd.print(".....Normal.....");
        xSemaphoreGive(lcdSemaphore);
      }

      // Serial monitor printing
      if (xSemaphoreTake(serialSemaphore, (TickType_t)10) == pdTRUE) {
        Serial.println(".....Normal.....");
        xSemaphoreGive(serialSemaphore);
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
