#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esperanto.h"
/*
* This example demonstrates the reception of ANT+ packets from ESP32, ESP32C3 and ESP32S3.

* By default, it matches communications with 0xe81e as device number.
* You can refer to the ant_scan example if you don't know the device number of the communication.
*/

/* ANT+ protocol specific constants */
#define ANT_PLUS_PREAMBLE                 (0xa6c5)
#define HEART_RATE_MONITOR_DEVICE_TYPE    (0x78)
#define ANT_PACKET_SIZE                   (0x17)
#define ANT_CRC16_INIT_VALUE              (0xFFFF)

void ant_rx_callback(uint8_t *packet,size_t size, int8_t rssi, int frequency) {
    /* ANT+ Reception callback: triggered when a packet is successfully received. */

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

    // Initialize radio and configure the reception callback
    init_radio(ant_rx_callback);

    // Configure the frequency on channel 57 (2457MHz)
    set_frequency(2457);

    // ANT packets requires big endian, swap the packets.
    set_swapping(true);

    // Configure the reception word
    uint32_t synchronization_word = ((ANT_PLUS_PREAMBLE << 16) | device_number);
    set_sync_word(synchronization_word);

    // Use a 1MBps datarate
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

    // Configure the radio
    configure_ant_plus(device_number);

    // Start the radio in reception mode
    start_radio(MODE_RX);
    while (1) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
