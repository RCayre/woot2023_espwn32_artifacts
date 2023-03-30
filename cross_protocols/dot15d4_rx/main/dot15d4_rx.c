#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esperanto.h"

/* 802.15.4 specific constants. */
#define DOT15D4_RX_PREAMBLE                 (0xa7)
#define DOT15D4_MATCHING_PATTERN            (0x03f73a9b)

void dot15d4_rx_callback(uint8_t *packet,size_t size, int8_t rssi, int frequency) {
    /* 802.15.4 reception callback. */
    // Let's assume we have a full buffer (best case)
    uint8_t msk_buffer[255];
    memset(msk_buffer, 0, 255);
    // Copy packet into our "ideal" buffer
    memcpy(msk_buffer,packet, size);

    uint32_t dot15d4_size;

    // max size of received frame after decoding
    uint8_t oqpsk_buffer[50];

    // Convert MSK modulation to OQPSK modulation
    msk_to_oqsk(oqpsk_buffer,&dot15d4_size, msk_buffer);

    // If packet size is coherent, process the frame
    if (dot15d4_size >= 6) {
      for (int i=0;i<dot15d4_size;i++) esp_rom_printf("%02x", oqpsk_buffer[i]);

      if (check_fcs_dot15d4(oqpsk_buffer,dot15d4_size)) {
          esp_rom_printf("[FCS_OK]\n");
      }
      else {
          esp_rom_printf("[FCS_KO]\n");
      }
    }
}

void configure_dot15d4(uint8_t channel) {
    /* Configuration of 802.15.4 mode on a specific channel. */
    init_radio(dot15d4_rx_callback);
    set_frequency(2405 + 5 * (channel - 11));
    set_sync_word(DOT15D4_MATCHING_PATTERN);
    set_data_rate(DATARATE_2M);
}

void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    /* Receive 802.15.4 packets on channel 12. */
    uint8_t channel_number = 12;

    // Configure radio in 802.15.4 mode.
    configure_dot15d4(channel_number);

    // Start reception mode.
    start_radio(MODE_RX);
    while (1) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
