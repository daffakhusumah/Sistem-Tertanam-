
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

LiquidCrystal_I2C lcd(0x3F,20,4);

int redLed = 10;
int greenLed = 13;
int buzzer = 8;
int smokeA0 = A0;
int sensorThres = 100;

volatile int analogSensorValue = 0;

SemaphoreHandle_t lcdSemaphore;
SemaphoreHandle_t serialSemaphore;

void setup() {
    pinMode(redLed, OUTPUT);
    pinMode(buzzer, OUTPUT);
    pinMode(smokeA0, INPUT);
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();
    lcd.begin(16,2);

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

void ReadSensorTask(void *pvParameters) {
    for (;;) {
        analogSensorValue = analogRead(smokeA0);

        // Access serial with semaphore
        if (xSemaphoreTake(serialSemaphore, (TickType_t)10) == pdTRUE) {
            Serial.print("Pin A0: ");
            Serial.println(analogSensorValue);
            xSemaphoreGive(serialSemaphore);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void DisplayTask(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(lcdSemaphore, (TickType_t)10) == pdTRUE) {
            lcd.clear();
            lcd.print("Smoke Level:");
            
            int scaledValue = map(analogSensorValue, 0, 1023, 0, 100); // Scale the value to 0-100
            lcd.setCursor(0, 1);  // Move cursor to second line
            lcd.print(scaledValue);
            lcd.print("%   ");  // Print percentage and clear any previous characters

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
                lcd.setCursor(0, 2);
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
                lcd.setCursor(0, 2);
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
