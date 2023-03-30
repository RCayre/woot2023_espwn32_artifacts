#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esperanto.h"

/*
* This example demonstrates the transmission of 802.15.4 packets from ESP32C3 and ESP32S3.
* It requires support of LE 2M Physical Layer, so it won't work on ESP32.
*
* It uses WazaBee attack to convert OQPSK symbols to the corresponding MSK symbols.
* By default, it transmits packets on channel 12.
*/


/* 802.15.4 specific constants. */
#define DOT15D4_RX_PREAMBLE                 (0xa7)
#define DOT15D4_MATCHING_PATTERN            (0x03f73a9b)
#define LEADING_ZEROES_LENGTH               (4)
#define SFD_LENGTH                          (1)
#define FCS_LENGTH                          (2)

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
    /* Configure radio to match 802.15.4 communications. */

    init_radio(dot15d4_rx_callback);
    set_frequency(2405 + 5 * (channel - 11));
    set_sync_word(DOT15D4_MATCHING_PATTERN);
    set_data_rate(DATARATE_2M);
}

void build_dot15d4_packet(uint8_t* packet, size_t packet_size, size_t *output_packet_size, uint8_t *output_packet) {
    /* Build a 802.15.4 packet. */

    // Compute the size.
    int total_size = packet_size + LEADING_ZEROES_LENGTH + SFD_LENGTH + FCS_LENGTH;

    // Prepare a buffer
    uint8_t prepared_packet[total_size];
    memset(prepared_packet, 0, total_size);

    // Fill it with leading zeros, Start of Frame Delimiter and packet content
    prepared_packet[LEADING_ZEROES_LENGTH] = DOT15D4_RX_PREAMBLE;
    memcpy(prepared_packet+LEADING_ZEROES_LENGTH+SFD_LENGTH, packet, packet_size);

    // Compute the FCS and append it at the end of the packet
    uint16_t fcs = calculate_fcs_dot15d4(packet-SFD_LENGTH,packet_size+SFD_LENGTH+FCS_LENGTH);
    prepared_packet[packet_size + LEADING_ZEROES_LENGTH + SFD_LENGTH ] = (uint8_t)((fcs & 0xFF00) >> 8);
    prepared_packet[packet_size + LEADING_ZEROES_LENGTH + SFD_LENGTH + 1] = (uint8_t)(fcs & 0x00FF);


    // Convert OQPSK modulation sequence to an equivalent MSK modulation sequence
    memset(output_packet, 0, 255);
    size_t msk_size;
    oqpsk_to_msk(output_packet,&msk_size, prepared_packet, total_size);
    *output_packet_size = msk_size;
}

void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    // Transmit a Hello World packet on channel 12 every second.
    uint8_t channel_number = 12;

    // Configure radio in 802.15.4 mode
    configure_dot15d4(channel_number);

    // Start TX
    start_radio(MODE_TX);

    // Build a 802.15.4 packet
    size_t dot15d4_packet_size = 16;
    uint8_t dot15d4_packet[16] = {
      dot15d4_packet_size+1,0x61,0x88,0xf9,0x32,0x33,0x00,0x00, 0x00, 0x00, 0xd3, 0x00, 0x68, 0x65, 0x6c, 0x6c
    };
  	uint8_t msk_packet[255];
    size_t msk_packet_size;
    build_dot15d4_packet(dot15d4_packet, dot15d4_packet_size, &msk_packet_size, msk_packet);

    while (1) {
      // Transmit the packet regularly
      transmit_packet(msk_packet, msk_packet_size);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
