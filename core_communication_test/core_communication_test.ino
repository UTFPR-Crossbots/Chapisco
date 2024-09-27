#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Shared data example (atomic access)
volatile int sharedData = 0;   // Shared memory variable
QueueHandle_t dataQueue;       // FreeRTOS queue handle

// Core 0 Task - Writer
void writerTask(void *param) {
    int counter = 0;
    while (true) {
        // Increment the counter and write to shared memory
        sharedData = counter;
        Serial.printf("Core 0 (Writer): Writing to shared memory: %d\n", sharedData);

        // Send the same data to the FreeRTOS queue
        if (xQueueSend(dataQueue, &counter, portMAX_DELAY) == pdPASS) {
            Serial.printf("Core 0 (Writer): Sent %d to the queue\n", counter);
        } else {
            Serial.println("Core 0 (Writer): Failed to send data to queue");
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second
    }
}

// Core 1 Task - Reader
void readerTask(void *param) {
    int receivedValue;
    while (true) {
        // Read from shared memory
        int currentSharedData = sharedData;
        Serial.printf("Core 1 (Reader): Read from shared memory: %d\n", currentSharedData);

        // Attempt to read from the queue
        if (xQueueReceive(dataQueue, &receivedValue, 500 / portTICK_PERIOD_MS) == pdPASS) {
            Serial.printf("Core 1 (Reader): Received from queue: %d\n", receivedValue);
        } else {
            Serial.println("Core 1 (Reader): No data received from queue in last 500ms");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second
    }
}

void setup() {
    Serial.begin(115200);

    // Create the queue capable of holding 10 integers
    dataQueue = xQueueCreate(10, sizeof(int));
    if (dataQueue == NULL) {
        Serial.println("Failed to create the queue");
        while (true); // Stop execution if the queue fails to create
    }

    // Create tasks pinned to specific cores
    xTaskCreatePinnedToCore(writerTask, "Writer Task", 2048, NULL, 1, NULL, 0); // Core 0
    xTaskCreatePinnedToCore(readerTask, "Reader Task", 2048, NULL, 1, NULL, 1); // Core 1
}

void loop() {
    // Nothing to do in the loop
}
