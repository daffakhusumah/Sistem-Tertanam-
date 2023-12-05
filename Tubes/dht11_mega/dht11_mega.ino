#include <Arduino_FreeRTOS.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 3     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);

void TaskDHT11(void *pvParameters);

void setup() {
  Serial.begin(9600);
  dht.begin();
  
  xTaskCreate(
    TaskDHT11,
    "DHT11",
    128,
    NULL,
    2,
    NULL
  );
}

void loop() {
  // Empty. Things are done in Tasks.
}

void TaskDHT11(void *pvParameters) {
  (void) pvParameters;
  
  sensors_event_t event;  
  for (;;) {
    // Get temperature event and print its value.
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println("Error reading temperature!");
    }
    else {
      Serial.print("Temperature: ");
      Serial.print(event.temperature);
      Serial.println(" *C");
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println("Error reading humidity!");
    }
    else {
      Serial.print("Humidity: ");
      Serial.print(event.relative_humidity);
      Serial.println("%");
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
