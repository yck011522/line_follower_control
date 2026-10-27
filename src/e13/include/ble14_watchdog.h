#pragma once
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <ble14_protocol.h>

// Initializes the ESP task watchdog and registers the calling foreground task.
inline void initializeWatchdog() {
    esp_task_wdt_config_t configuration{Ble14::WATCHDOG_MS, 0, true};
    esp_err_t result = esp_task_wdt_init(&configuration);
    if (result == ESP_ERR_INVALID_STATE) result = esp_task_wdt_reconfigure(&configuration);
    configASSERT(result == ESP_OK);
    configASSERT(esp_task_wdt_add(nullptr) == ESP_OK);
}
// Registers an additional BLE worker task with the shared watchdog.
inline void watchThisTask() { configASSERT(esp_task_wdt_add(nullptr) == ESP_OK); }

// Feeds the watchdog from a task that has completed a bounded unit of work.
inline void feedWatchdog() { esp_task_wdt_reset(); }
