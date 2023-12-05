#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>

LiquidCrystal_I2C lcd(0x3F, 20, 4);

int redLed = 10;
int greenLed = 13;
int buzzer = 8;
int smokeA0 = A0;
int tempPin = A1;  // Temperature sensor pin
int buttonPin = 2;  // Button pin
int sensorThres = 100;

volatile int analogSensorValue = 0;
volatile bool showTemperature = false;  // Flag to switch between smoke level and temperature

SemaphoreHandle_t lcdSemaphore;
SemaphoreHandle_t serialSemaphore;
QueueHandle_t mailbox;

void setup() {
    pinMode(redLed, OUTPUT);
    pinMode(greenLed, OUTPUT);
    pinMode(buzzer, OUTPUT);
    pinMode(smokeA0, INPUT);
    pinMode(tempPin, INPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(buttonPin), changeDisplay, FALLING);
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();
    lcd.begin(20, 4);

    lcdSemaphore = xSemaphoreCreateMutex();
    serialSemaphore = xSemaphoreCreateMutex();
    mailbox = xQueueCreate(1, sizeof(int));

    if (mailbox == NULL) {
        Serial.println("Error creating the mailbox");
        return;
    }

    xTaskCreate(ReadSensorTask, "ReadSensor", 128, NULL, 1, NULL);
    xTaskCreate(DisplayTask, "Display", 128, NULL, 2, NULL);
    xTaskCreate(AlertTask, "Alert", 128, NULL, 3, NULL);
    xTaskCreate(MailboxTask, "Mailbox", 128, NULL, 1, NULL);

    vTaskStartScheduler();
}

void loop() {
    // Empty. Tasks will handle everything.
}

void changeDisplay() {
    showTemperature = !showTemperature;
}

void ReadSensorTask(void *pvParameters) {
    for (;;) {
        analogSensorValue = analogRead(smokeA0);
        xQueueSend(mailbox, &analogSensorValue, portMAX_DELAY);

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
            if (showTemperature) {
                int tempReading = analogRead(tempPin);
                float temperature = tempReading * 5.0 * 100.0 / 1024.0;
                lcd.print("Temperature:");
                lcd.setCursor(0, 1);
                lcd.print(temperature, 2);
                lcd.print(" C");
            } else {
                lcd.print("Smoke Level:");
                int scaledValue = map(analogSensorValue, 0, 1023, 0, 100);
                lcd.setCursor(0, 1);
                lcd.print(scaledValue);
                lcd.print("%   ");
            }
            xSemaphoreGive(lcdSemaphore);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void AlertTask(void *pvParameters) {
    for (;;) {
        if (analogSensorValue > sensorThres) {
            digitalWrite(redLed, HIGH);
            tone(buzzer, 1000, 200);

            if (xSemaphoreTake(lcdSemaphore, (TickType_t)10) == pdTRUE) {
                lcd.setCursor(0, 2);
                lcd.print("Alert....!!!");
                xSemaphoreGive(lcdSemaphore);
            }

            if (xSemaphoreTake(serialSemaphore, (TickType_t)10) == pdTRUE) {
                Serial.println("Alert....!!!");
                xSemaphoreGive(serialSemaphore);
            }

            digitalWrite(greenLed, LOW);
        } else {
            digitalWrite(redLed, LOW);
            digitalWrite(greenLed, HIGH);

            if (xSemaphoreTake(lcdSemaphore, (TickType_t)10) == pdTRUE) {
                lcd.setCursor(0, 2);
                lcd.print(".....Normal.....");
                xSemaphoreGive(lcdSemaphore);
            }

            if (xSemaphoreTake(serialSemaphore, (TickType_t)10) == pdTRUE) {
                Serial.println(".....Normal.....");
                xSemaphoreGive(serialSemaphore);
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void MailboxTask(void *pvParameters) {
    int receivedValue;
    for (;;) {
        if (xQueueReceive(mailbox, &receivedValue, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(serialSemaphore, (TickType_t)10) == pdTRUE) {
                Serial.print("Mailbox received: ");
                Serial.println(receivedValue);
                xSemaphoreGive(serialSemaphore);
            }
        }
    }
}
