#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esperanto.h"

/* ANT+ protocol specific constants */
#define ANT_PLUS_PREAMBLE                 (0xa6c5)
#define HEART_RATE_MONITOR_DEVICE_TYPE    (0x78)
#define ANT_PACKET_SIZE                   (0x17)
#define ANT_CRC16_INIT_VALUE              (0xFFFF)

void ant_rx_callback(uint8_t *packet,size_t size, int8_t rssi, int frequency) {
    /* ANT+ Reception callback. */

    /* Display the packet and its associated metadata. */
    esp_rom_printf("[frequency=%d, rssi=%d] ", frequency,rssi);
    for (int i=0;i<17;i++) {
        esp_rom_printf("%02x",packet[i]);
    }

    uint16_t received_crc = packet[15] << 8 | packet[16];
    /* Checks the CRC validity. */
    if (crc16(packet,15, 0xFFFF) == received_crc) {
        esp_rom_printf(" [CRC OK]\n");
    }
    else {
        esp_rom_printf(" [CRC KO]\n");
    }
}

void configure_ant_plus(uint16_t device_number) {
    /* Configure radio to match ANT+ communications according to a specific device number.*/
    init_radio(ant_rx_callback);
    set_frequency(2457);
    set_swapping(true);
    uint32_t synchronization_word = ((ANT_PLUS_PREAMBLE << 16) | device_number);
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
    // Listen the packets received by a device with device number 0xe81e.
    uint16_t device_number = 0xe81e;

    configure_ant_plus(device_number);

    start_radio(MODE_RX);
    while (1) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
