#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esperanto.h"


/*
* This example demonstrates the reception of Riitek packets from ESP32, ESP32S3 and ESP32C3.
* It is configured to work with the address 02:20:20:86:68 on channel 26 (2426MHz).
*/

/* Riitek specific constants. */
#define RIITEK_PREAMBLE                   (0xAA)
#define RIITEK_CRC16_INIT_VALUE           (0x8b83)

void riitek_rx_callback(uint8_t *packet,size_t size, int8_t rssi, int frequency) {
    /* Riitek packet reception callback. */
    esp_rom_printf("[frequency=%d, rssi=%d] ", frequency,rssi);
    for (int i=0;i<17;i++) {
        esp_rom_printf("%02x",packet[i]);
    }

    // Checks the CRC validity.
    uint16_t received_crc = packet[15] << 8 | packet[16];
    if (crc16(packet+6, 9, RIITEK_CRC16_INIT_VALUE) == received_crc) {
        esp_rom_printf(" [CRC OK]\n");
    }
    else {
        esp_rom_printf(" [CRC KO]\n");
    }
}

void configure_riitek(uint8_t *address) {
    /* Configure radio according to riitek protocol with a specific address. */
    init_radio(riitek_rx_callback);
    set_frequency(2426);
    set_swapping(true);
    uint32_t synchronization_word = (
      (RIITEK_PREAMBLE << 24) |
      (address[0] << 16) |
      (address[1] << 8) |
      address[2]
    );
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

    // Address to use.
    uint8_t address[5] = {0x02, 0x20, 0x20, 0x86, 0x68};

    // Configure radio.
    configure_riitek(address);

    // Start reception mode.
    start_radio(MODE_RX);
    while (1) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
