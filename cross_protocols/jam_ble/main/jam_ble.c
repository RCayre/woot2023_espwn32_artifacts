#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esperanto.h"


/*
* This example demonstrates the simultaneous jamming of Bluetooth Low Energy advertising channels.
* It only works on ESP32 for now (need some minor fixes to make it work on ESP32S3/ESP32C3.
*/



void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    // Init the radio.
    init_radio(NULL);

    // Start the jammer mode: transmit glitches over the three BLE advertising channels.
    start_radio(MODE_JAMMER);

    while (1) {
      vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
