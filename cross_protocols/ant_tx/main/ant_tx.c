#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esperanto.h"

/*
* This example demonstrates the transmission of ANT+ packets from ESP32, ESP32C3 and ESP32S3.
* It imitates an ANT+ Heart Rate Monitor device.
*
* By default, it matches communications with 0xe81e as device number.
* You can refer to the ant_scan example if you don't know the device number of the communication.
*/


/* ANT+ specific constants. */
#define ANT_PLUS_PREAMBLE                 (0xa6c5)
#define HEART_RATE_MONITOR_DEVICE_TYPE    (0x78)
#define ANT_PACKET_SIZE                   (0x17)
#define ANT_CRC16_INIT_VALUE              (0xFFFF)

void ant_rx_callback(uint8_t *packet,size_t size, int8_t rssi, int frequency) {
    /* ANT+ reception callback. */
    esp_rom_printf("[frequency=%d, rssi=%d] ", frequency,rssi);
    for (int i=0;i<17;i++) {
        esp_rom_printf("%02x",packet[i]);
    }

    uint16_t received_crc = packet[15] << 8 | packet[16];
    if (crc16(packet,15, 0xFFFF) == received_crc) {
        esp_rom_printf(" [CRC OK]\n");
    }
    else {
        esp_rom_printf(" [CRC KO]\n");
    }
}

void configure_ant_plus(uint16_t device_number) {
    /* Configure radio to match communication. */
    init_radio(ant_rx_callback);
    set_frequency(2457);
    set_swapping(true);
    uint32_t synchronization_word = ((ANT_PLUS_PREAMBLE << 16) | device_number);
    set_sync_word(synchronization_word);
    set_data_rate(DATARATE_1M);
}

void build_ant_hrm_packet(uint8_t* ant_packet, uint16_t device_number, uint8_t transmission_type, uint8_t counter, uint8_t hrm_value) {
    /* Build an HRM packet. */

    uint8_t ant_payload[11] = {0x78,0x01,0x0a,0x84,0x00,0xce,0x1a,0x00,0x00,0x00,0xF0};
    uint8_t device_type = HEART_RATE_MONITOR_DEVICE_TYPE;

    ant_packet[0] = (uint8_t)((ANT_PLUS_PREAMBLE & 0xFF00) >> 8);
    ant_packet[1] = (uint8_t)(ANT_PLUS_PREAMBLE & 0x00FF);
    ant_packet[2] = (uint8_t)((device_number & 0xFF00) >> 8);
    ant_packet[3] = (uint8_t)(device_number & 0x00FF);
    ant_packet[4] = device_type;
    ant_packet[5] = transmission_type;
    memcpy(ant_packet+6, ant_payload, 11);

    ant_packet[13] = counter;
    ant_packet[14] = hrm_value;

    uint16_t crc = crc16(ant_packet,15, ANT_CRC16_INIT_VALUE);
    ant_packet[15] = (crc & 0xFF00) >> 8;
    ant_packet[16] = (crc & 0x00FF);
}

void app_main(void)
{


    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    /* Imitate a Heart Rate Monitor with device number 0xe81e. */
    uint16_t device_number = 0xe81e;
    uint8_t transmission_type = 0x01;

    configure_ant_plus(device_number);

    start_radio(MODE_TX);
    uint8_t counter = 0;

    uint8_t ant_packet[17];
    while (1) {
      // Build and transmit HRM packets regularly
      build_ant_hrm_packet(ant_packet, device_number, transmission_type, counter,240 + (counter % 3));
      transmit_packet(ant_packet, 17);
      counter = (counter < 255 ? counter + 1 : 0);
      vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
