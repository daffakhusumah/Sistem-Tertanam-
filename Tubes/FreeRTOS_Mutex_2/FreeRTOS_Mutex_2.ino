#include <Arduino_FreeRTOS.h>
#include <semphr.h>

SemaphoreHandle_t mutex_v;
SemaphoreHandle_t interruptSemaphore;
TaskHandle_t HandleTaskGen = NULL;
TaskHandle_t HandleLedYellow;
TaskHandle_t HandleLedRed;
TaskHandle_t HandleLedGreen;
TaskHandle_t HandleTask1;
TaskHandle_t HandleTask2;

void Task1( void *pvParameters );
void Task2( void *pvParameters );
void TaskLedRed( void *pvParameters ); //LED merah
void TaskLedGreen( void *pvParameters ); // LED hijau

void setup() {
  Serial.begin(9600);
  mutex_v = xSemaphoreCreateMutex();

  if (mutex_v == NULL) {
    Serial.println("Mutex can not be created");
  }
  xTaskCreate(Task1, "Task 1", 128, NULL, 1, &HandleTask1);
  xTaskCreate(Task2, "Task 2", 128, NULL, 1, &HandleTask2);
  xTaskCreate(TaskLedRed,  "LedRed", 128, NULL, 3, &HandleLedRed ); // interrupt LED Merah
  xTaskCreate(TaskLedGreen,  "LedGreen", 128, NULL, 1, &HandleLedGreen ); // LED Hijau
  xTaskCreate(TaskLedYellow, "LedYellow",   128, NULL, 1, &HandleLedYellow ); // LED Kuning

  interruptSemaphore = xSemaphoreCreateBinary();
  if (interruptSemaphore != NULL) {
    attachInterrupt(digitalPinToInterrupt(2), debounceInterrupt, LOW);
  }
  vTaskStartScheduler();
}

void loop() {}

long debouncing_time = 150;
volatile unsigned long last_micros;
void debounceInterrupt() {
  if ((long)(micros() - last_micros) >= debouncing_time * 1000) {
    interruptHandler();
    last_micros = micros();
  }
}

void interruptHandler() {
  xSemaphoreGiveFromISR(interruptSemaphore, NULL);
}

void Task1(void *pvParameters) {
  while (1) {
    xSemaphoreTake(mutex_v, 200 / portTICK_PERIOD_MS);
    Serial.println("Task1");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    xSemaphoreGive(mutex_v);
  }
}

void Task2(void *pvParameters) {
  while (1) {
    xSemaphoreTake(mutex_v, 200 / portTICK_PERIOD_MS);
    Serial.println("Task2");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    xSemaphoreGive(mutex_v);
  }
}

void TaskLedRed(void *pvParameters) //LED Merah
{
  (void) pvParameters;
  pinMode(10, OUTPUT);
  while (1) {
    Serial.println("CekISR");
    if (xSemaphoreTake(interruptSemaphore, portMAX_DELAY) == pdPASS) 
    {
      Serial.println("ISR Semaphore taken");
      digitalWrite(10, !digitalRead(10));
      Serial.println("LED Red");
      Serial.println("ISR Semaphore returned");
      vTaskDelay(200 / portTICK_PERIOD_MS);
      //xSemaphoreGive(interruptSemaphore);
    }
  }
}

void TaskLedGreen(void *pvParameters)
{
  (void) pvParameters;
  pinMode(12, OUTPUT);
  while (1) {
    xSemaphoreTake(mutex_v, 200 / portTICK_PERIOD_MS);
    Serial.println("LED Green");
    digitalWrite(12, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(12, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xSemaphoreGive(mutex_v);
  }
}

void TaskLedYellow(void *pvParameters)
{
  (void) pvParameters;
  pinMode(11, OUTPUT);
  while (1) {
    xSemaphoreTake(mutex_v, 20 / portTICK_PERIOD_MS);
    Serial.println("LED Yellow");
    digitalWrite(11, HIGH);
    vTaskDelay(150 / portTICK_PERIOD_MS);
    digitalWrite(11, LOW);
    vTaskDelay(150 / portTICK_PERIOD_MS);
    xSemaphoreGive(mutex_v);
  }
}
