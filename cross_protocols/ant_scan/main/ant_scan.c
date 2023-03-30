#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esperanto.h"

/* ANT+ protocol specific constants */
#define ANT_PLUS_PREAMBLE                 (0xa6c5)
#define HEART_RATE_MONITOR_DEVICE_TYPE    (0x78)
#define ANT_PACKET_SIZE                   (0x17)
#define ANT_CRC16_INIT_VALUE              (0xFFFF)

void ant_scan_callback(uint8_t *packet,size_t size, int8_t rssi, int frequency) {
  /* ANT+ scan reception callback. */
    if (rssi > -90 && size >= 7) {
      if (packet[6] == HEART_RATE_MONITOR_DEVICE_TYPE) {
        esp_rom_printf("[frequency=%d, rssi=%d] Candidate device number: ", frequency,rssi);
        for (int i=4;i<6;i++) {
            esp_rom_printf("%02x",packet[i]);
        }
        esp_rom_printf("\n");
      }
    }
}

void configure_ant_plus_scan() {
    /* Configure ANT+ scan mode to detect candidate device number. */
    init_radio(ant_scan_callback);
    set_frequency(2457);
    set_swapping(true);
    uint32_t synchronization_word = ((0xFFFF << 16) | ANT_PLUS_PREAMBLE);
    set_sync_word(synchronization_word);
    set_data_rate(DATARATE_1M);
}


void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    // Configure the ANT+ scan mode.
    configure_ant_plus_scan();

    // Start reception.
    start_radio(MODE_RX);
    while (1) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
