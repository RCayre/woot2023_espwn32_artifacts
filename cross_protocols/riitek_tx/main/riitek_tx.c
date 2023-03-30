#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esperanto.h"

/* Riitek specific constants. */
#define RIITEK_PREAMBLE                   (0xAA)
#define RIITEK_KEYSTROKE_TYPE             (0x0B)
#define RIITEK_CRC16_INIT_VALUE           (0x8b83)

void riitek_rx_callback(uint8_t *packet,size_t size, int8_t rssi, int frequency) {
  /* Riitek packet reception callback. */
    esp_rom_printf("[frequency=%d, rssi=%d] ", frequency,rssi);
    for (int i=0;i<17;i++) {
        esp_rom_printf("%02x",packet[i]);
    }

    uint16_t received_crc = (packet[15] << 8) | packet[16];
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


void build_riitek_keystroke_packet(uint8_t* riitek_packet, uint8_t *address, uint8_t *hid_data) {
    /* Build a Riitek keystroke packet according to an address and a specific HID data to transmit. */
    riitek_packet[0] = (uint8_t)(RIITEK_PREAMBLE);
    memcpy(riitek_packet+1, address, 5);
    riitek_packet[6] = 0x40;
    riitek_packet[7] = RIITEK_KEYSTROKE_TYPE;
    memcpy(riitek_packet+8, hid_data, 7);
    uint16_t crc = crc16(riitek_packet+6, 9, RIITEK_CRC16_INIT_VALUE);
    riitek_packet[15] = (crc & 0xFF00) >> 8;
    riitek_packet[16] = (crc & 0x00FF);
}

void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }


    // Address in use
    uint8_t address[5] = {0x02, 0x20, 0x20, 0x86, 0x68};

    configure_riitek(address);

    // Q key press
    uint8_t hid_data_press[7] = {0x00, 0x04, 0x00,0x00,0x00,0x00,0x00};

    // key release
    uint8_t hid_data_release[7] = {0x00, 0x00, 0x00,0x00,0x00,0x00,0x00};

    uint8_t riitek_packet[17];

    start_radio(MODE_TX);
    while (1) {
      // Transmit a Q keystroke packet
      build_riitek_keystroke_packet(riitek_packet, address, hid_data_press);
      transmit_packet(riitek_packet, 17);
      vTaskDelay(100 / portTICK_PERIOD_MS);

      // Transmit a key release packet
      build_riitek_keystroke_packet(riitek_packet, address, hid_data_release);
      transmit_packet(riitek_packet, 17);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
