#include "radio_hooks.h"

// Structure storing radio configuration parameters
radio_config_t radio_config;

// Reception and transmission queues
static QueueHandle_t reception_queue;
static QueueHandle_t transmission_queue;

// Reception and transmission handles
TaskHandle_t rxProcessHandle = NULL;
TaskHandle_t txProcessHandle = NULL;


// Callbacks
#ifdef ESP32
F_r_lld_evt_rx old_r_lld_evt_rx = NULL;
F_r_llm_set_scan_en old_r_llm_start_scan_en = NULL;
F_r_lld_pdu_tx_push old_r_lld_pdu_tx_push = NULL;
#endif

#if defined(ESP32S3) || defined(ESP32C3)
F_r_lld_scan_process_rx_pkt old_r_lld_scan_process_rx_pkt = NULL;
F_r_lld_scan_evt_start_cbk old_r_lld_scan_evt_start_cbk = NULL;
F_r_lld_test_freq2chnl old_r_lld_test_freq2chnl = NULL;
#endif

F_phy_rom_loopback_mode_en old_phy_rom_loopback_mode_en = NULL;

#ifdef ESP32
void *r_emi_get_mem_addr_by_offset(uint16_t offset) {
    return (void*)(0x3ffb0000+offset);
}
#endif

#ifdef ESP32
int rx_custom_callback_esp32(void* evt, uint32_t nb_rx_desc) {
  pkt_header_t *p_pkt = (pkt_header_t *)(r_emi_get_mem_addr_by_offset(HEADER_IN_EM_OFFSET));
  uint16_t pkt_status = *(uint16_t *)r_emi_get_mem_addr_by_offset(PACKET_STATUS_IN_EM_OFFSET);
  if (nb_rx_desc == 0 || ((pkt_status & 0x13f) != 0)) {
      return old_r_lld_evt_rx(evt,nb_rx_desc);
  }
  int j;
  if ((*((uint8_t *)evt + 0x72) & 0x10) == 0) {
    uint8_t fifo_index = ((uint8_t *)CURRENT_RX_FIFO_ADDRESS)[FIFO_INDEX_OFFSET];

    for (int k=0; k<nb_rx_desc; k++) {

      j = (fifo_index + k) % 8;
      uint32_t pkt_header = p_pkt[j].header;

      int8_t rssi = (pkt_header>>16) & 0xff;
      uint8_t pkt_size = (pkt_header >> 8) & 0xff;
      if (pkt_size > 0 && pkt_size < 255) {
        uint8_t opcode = pkt_header & 0xFF;

        uint16_t *buffer_offset = (uint16_t*)(r_emi_get_mem_addr_by_offset(RX_DESCRIPTOR_IN_EM_OFFSET) + 12*j);
        uint8_t *p_pdu = (uint8_t *)(r_emi_get_mem_addr_by_offset(*buffer_offset));
        rx_packet_t packet;
        packet.total_size = 4+2+pkt_size;
        packet.rssi = rssi;
        packet.frequency = radio_config.frequency;
        packet.packet = (uint8_t*)malloc(sizeof(uint8_t)*packet.total_size);
        if (packet.packet != NULL) {
          memcpy(packet.packet,&radio_config.sync_word,4);
          packet.packet[4] = opcode;
          packet.packet[5] = pkt_size;
          memcpy(&packet.packet[6],p_pdu, pkt_size);

          if (radio_config.swapping) {
            for (int i=0;i<pkt_size;i++) packet.packet[i] = swap(packet.packet[i]);
          }
          packet.packet[4] = packet.packet[4] ^ (1 << 3);

          if (xQueueSend(reception_queue, (void *)&packet, ( TickType_t ) 0) != pdTRUE) {
              ESP_LOGD(TAG, "Failed to enqueue received packet. Queue full.");
              free(packet.packet);
          }
        }

      }
      p_pkt[j].header = 0;
    }
  }
  return old_r_lld_evt_rx(evt,nb_rx_desc);
}

int rx_custom_config_esp32(uint8_t *scan_param) {
  int ret = old_r_llm_start_scan_en(scan_param);
  if (radio_config.mode == MODE_RX && radio_config.enabled) {
    configure_format(LLD_TEST_MODE_RX);
    configure_sync_word(radio_config.sync_word);// ant: 0x65a31778; // scan ant ? 0x555565a3
    configure_frequency(radio_config.frequency);
    disable_frequency_hopping();
    configure_max_rx_buffer_size(0xFF);

    disable_crc_checking();
    disable_whitening();
    radio_config.enabled = true;
  }
  return ret;
}
#endif

#if defined(ESP32S3) || defined(ESP32C3)

void rx_custom_callback_esp32c3(void* evt) {
    if (radio_config.mode == MODE_RX && radio_config.enabled) {
      pkt_header_t *p_pkt = (pkt_header_t *)(r_emi_get_mem_addr_by_offset(HEADER_IN_EM_OFFSET));
      uint8_t fifo_index = ((uint8_t *)CURRENT_RX_FIFO_ADDRESS)[FIFO_INDEX_OFFSET];

      uint32_t pkt_header = p_pkt[fifo_index].header;

      if (pkt_header == 0) {
          return;
      }

      int8_t rssi = (pkt_header>>16) & 0xff;
      uint8_t pkt_size = (pkt_header >> 8) & 0xff;
      uint8_t opcode = pkt_header & 0xFF;

      uint16_t buffer_offset = p_pkt[fifo_index].buffer_offset;

      uint8_t *p_pdu = (uint8_t *)(r_emi_get_mem_addr_by_offset(buffer_offset));

      rx_packet_t packet;
      packet.total_size = 4+2+pkt_size;
      packet.rssi = rssi;
      packet.frequency = radio_config.frequency;
      packet.packet = (uint8_t*)malloc(sizeof(uint8_t)*packet.total_size);
      if (packet.packet != NULL) {
        memcpy(packet.packet,&radio_config.sync_word,4);
        packet.packet[4] = opcode;
        packet.packet[5] = pkt_size;
        memcpy(&packet.packet[6],p_pdu, pkt_size);
        // For some reason, hardware no whitening mode doesn't work on ESP32C3, let's perform the dewhitening manually
        dewhiten_ble(&packet.packet[4], pkt_size+2,39);

        if (radio_config.swapping) {
          for (int i=0;i<pkt_size;i++) packet.packet[i] = swap(packet.packet[i]);
        }
        if (xQueueSend(reception_queue, (void *)&packet, ( TickType_t ) 0) != pdTRUE) {
            ESP_LOGD(TAG, "Failed to enqueue received packet. Queue full.");
            free(packet.packet);
        }
      }

      p_pkt[fifo_index].header = 0;

    }
    old_r_lld_scan_process_rx_pkt(evt);
}
void rx_custom_config_esp32c3(int param_1,int param_2,int param_3) {
    old_r_lld_scan_evt_start_cbk(param_1,param_2,param_3);
    if (radio_config.mode == MODE_RX) {
      configure_format(LLD_TEST_MODE_RX);
      configure_data_rate(radio_config.data_rate);
      configure_sync_word(radio_config.sync_word);
      configure_frequency(radio_config.frequency);
      disable_frequency_hopping();
      configure_max_rx_buffer_size(0xFF);

      disable_crc_checking();
      disable_whitening();
      radio_config.enabled = true;
    }
}
#endif

void set_data_rate(datarate_t data_rate) {
    radio_config.data_rate = data_rate;
    #ifdef ESP32
    if (data_rate == DATARATE_2M) {
      ESP_LOGE(TAG, "ESP32 does not support LE 2M, use ESP32C3 or ESP32S3 instead.");
    }
    #endif
}

void set_frequency(uint32_t frequency) {
    radio_config.frequency = frequency;
}

void set_sync_word(uint32_t sync_word) {
    radio_config.sync_word = sync_word;
}
void set_swapping(bool swapping) {
    radio_config.swapping = swapping;
}

#if defined(ESP32S3) || defined(ESP32C3)
void configure_data_rate(datarate_t data_rate) {
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);
    int rate = (data_rate == DATARATE_1M ? 0 : 1);
    control_structure[THRCNTL_RATECNTL_REG_IN_CS_OFFSET] = (rate << 2) | rate;
    control_structure[THRCNTL_RATECNTL_REG_IN_CS_OFFSET+1] = 0x10;
}
#endif

void configure_sync_word(uint32_t sync) {
    if (radio_config.swapping) {
        sync =  swap((sync & 0xFF000000) >> 24) << 24 |
                swap((sync & 0x00FF0000) >> 16) << 16 |
                swap((sync & 0x0000FF00) >> 8) << 8 |
                swap(sync & 0x000000FF);
    }
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);
    control_structure[SYNC_REG_IN_CS_OFFSET] = (sync & 0xFF000000) >> 24;
    control_structure[SYNC_REG_IN_CS_OFFSET+1] = (sync & 0x00FF0000) >> 16;
    control_structure[SYNC_REG_IN_CS_OFFSET+2] = (sync & 0x0000FF00) >> 8;
    control_structure[SYNC_REG_IN_CS_OFFSET+3] = (sync & 0x000000FF);
    radio_config.sync_word = switch_endianness(sync);
}

void configure_format(uint8_t format) {
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);
    control_structure[CNTL_REG_IN_CS_OFFSET] = format;
}

void configure_max_rx_buffer_size(uint8_t max_size) {
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);
    control_structure[RXMAXBUF_REG_IN_CS_OFFSET] = max_size;
}

void disable_frequency_hopping() {
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);
    control_structure[HOPCTRL_REG_IN_CS_OFFSET+1] = 0;
    control_structure[HOPCTRL_REG_IN_CS_OFFSET] = 39;
}

void configure_frequency(uint32_t frequency) {
    uint8_t offset = frequency - 2402;
    uint8_t *frequency_table = r_emi_get_mem_addr_by_offset(FREQUENCY_TABLE_IN_EM_OFFSET);
    frequency_table[39] = offset;
    radio_config.frequency = frequency;
}
void disable_crc_checking() {
    uint32_t *rwblecntl_reg = (uint32_t*)RWBLECNTL_REG_ADDRESS;
    *rwblecntl_reg = *rwblecntl_reg | (1 << 16)| (1 << 17);
}

void disable_whitening() {
    uint32_t *rwblecntl_reg = (uint32_t*)RWBLECNTL_REG_ADDRESS;
    *rwblecntl_reg = *rwblecntl_reg | (1 << 18);
}


struct em_buf_tx_desc * get_em_tx_desc() {
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);

    uint16_t txdescptr = control_structure[TXDESCPTR_REG_IN_CS_OFFSET] | (control_structure[TXDESCPTR_REG_IN_CS_OFFSET+1] << 8);
    return (struct em_buf_tx_desc *)r_emi_get_mem_addr_by_offset(txdescptr);
}



void reception_packet_process(void *pvParameters)
{
  rx_packet_t *rcv_packet = (rx_packet_t *)malloc(sizeof(rx_packet_t));
  if (rcv_packet == NULL) {
      ESP_LOGE(TAG, "Malloc rcv_packet failed!");
      return;
  }

  while (1) {
    if (xQueueReceive(reception_queue, rcv_packet, portMAX_DELAY) != pdPASS) {
      ESP_LOGE(TAG, "Queue receive error");
    }
    else {
      radio_config.rx_callback(rcv_packet->packet,rcv_packet->total_size, rcv_packet->rssi, rcv_packet->frequency);
      memset(rcv_packet, 0, sizeof(rx_packet_t));
    }
  }
}

void transmission_packet_process(void *pvParameters)
{
  tx_packet_t *tx_packet = (tx_packet_t *)malloc(sizeof(tx_packet_t));
  if (tx_packet == NULL) {
      ESP_LOGE(TAG, "Malloc tx_packet failed!");
      return;
  }

  while (1) {

    if (xQueueReceive(transmission_queue, tx_packet, portMAX_DELAY) != pdPASS) {
      ESP_LOGE(TAG, "Queue receive error");
    }
    else {
    struct em_buf_tx_desc *txdesc = get_em_tx_desc();
    uint8_t *dataptr = r_emi_get_mem_addr_by_offset(txdesc->txdataptr);
    memset(dataptr, 0, 0xFF);

    memcpy(dataptr, tx_packet->packet, tx_packet->size);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    memset(dataptr, 0, 0xFF);

    memset(tx_packet, 0, sizeof(tx_packet_t));

    }
  }
}


#ifdef ESP32
void tx_custom_config_esp32(int param_1,uint8_t* param_2) {
  configure_frequency(radio_config.frequency);
  configure_format(LLD_TEST_MODE_TX);
  old_r_lld_pdu_tx_push(param_1,param_2);
  radio_config.enabled = true;
}

#endif


#if defined(ESP32S3) || defined(ESP32C3)
int tx_custom_config_esp32c3(int freq) {
    configure_frequency(radio_config.frequency);
    configure_format(LLD_TEST_MODE_TX);

    // Reset the transmit buffer to zero
    struct em_buf_tx_desc *txdesc = get_em_tx_desc();
    uint8_t *dataptr = r_emi_get_mem_addr_by_offset(txdesc->txdataptr);
    memset(dataptr, 0, 0xFF);

    int chnl = old_r_lld_test_freq2chnl(freq);
    radio_config.enabled = true;
    return chnl;
}
#endif

#ifdef ESP32
uint32_t jamming_callback(uint8_t en) {
  esp_task_wdt_reset();
   if (radio_config.mode == MODE_JAMMER && radio_config.enabled) {
      phy_dis_hw_set_freq();
      int i=0;
      while (radio_config.mode == MODE_JAMMER && radio_config.enabled) {
          set_chan_freq_sw_start(2,0,0);
          ram_start_tx_tone(1,0,20,0,0,0);
          set_chan_freq_sw_start(80,0,0);
          ram_start_tx_tone(1,0,20,0,0,0);
          set_chan_freq_sw_start(26,0,0);
          ram_start_tx_tone(1,0,20,0,0,0);
          i++;
          if ((i % 10000) == 0) {
            vTaskDelay(10 / portTICK_RATE_MS);
          }
      }
  }
   return old_phy_rom_loopback_mode_en(en);
}
#endif

void setup_esperanto_hooks() {
    #ifdef ESP32
    /* Setup RX callback hook */
    // Save the old function pointer to old_r_lld_evt_rx
    old_r_lld_evt_rx = (*r_ip_funcs_p)[RX_SCAN_CALLBACK_INDEX];
    // Inject our rx_custom_callback hook
    (*r_ip_funcs_p)[RX_SCAN_CALLBACK_INDEX] = (void*)rx_custom_callback_esp32;

    /* Setup Scan configuration hook */
    // Save the old function pointer to old_r_lld_scan_start
    old_r_llm_start_scan_en = (*r_ip_funcs_p)[RX_SCAN_CONFIG_INDEX];
    // Inject our rx_custom_config hook
    (*r_ip_funcs_p)[RX_SCAN_CONFIG_INDEX] = (void*)rx_custom_config_esp32;


    /* Setup TX Test configuration hook */
    old_r_lld_pdu_tx_push = (*r_ip_funcs_p)[LLD_PDU_TX_PUSH_CALLBACK_INDEX];
    // Inject our tx_custom_config hook
    (*r_ip_funcs_p)[LLD_PDU_TX_PUSH_CALLBACK_INDEX] = (void*)tx_custom_config_esp32;
    #endif

    #if defined(ESP32S3) || defined(ESP32C3)
    /* Setup RX callback hook */
    // Save the old function pointer to old_r_lld_scan_process_rx_pkt
    old_r_lld_scan_process_rx_pkt =  (*r_ip_funcs_p)[RX_SCAN_CALLBACK_INDEX];
    // Inject our rx_custom_callback hook
    (*r_ip_funcs_p)[RX_SCAN_CALLBACK_INDEX] = (void*)rx_custom_callback_esp32c3;

    /* Setup Scan configuration hook */
    // Save the old function pointer to old_r_lld_scan_start
    old_r_lld_scan_evt_start_cbk = (*r_ip_funcs_p)[RX_SCAN_CONFIG_INDEX];
    // Inject our rx_custom_config hook
    (*r_ip_funcs_p)[RX_SCAN_CONFIG_INDEX] = (void*)rx_custom_config_esp32c3;

    /* Setup TX Test configuration hook */
    old_r_lld_test_freq2chnl = (*r_ip_funcs_p)[LLD_FREQ_TO_CHANNEL_CALLBACK_INDEX];
    // Inject our tx_custom_config hook
    (*r_ip_funcs_p)[LLD_FREQ_TO_CHANNEL_CALLBACK_INDEX] = (void*)tx_custom_config_esp32c3;
    #endif
}

void restore_esperanto_hooks() {
    #ifdef ESP32
    if (old_r_lld_evt_rx != NULL) {
      (*r_ip_funcs_p)[RX_SCAN_CALLBACK_INDEX] = old_r_lld_evt_rx;
    }
    if (old_r_llm_start_scan_en != NULL) {
      (*r_ip_funcs_p)[RX_SCAN_CONFIG_INDEX] = old_r_llm_start_scan_en;
    }
    if (old_r_lld_pdu_tx_push != NULL) {
      (*r_ip_funcs_p)[LLD_PDU_TX_PUSH_CALLBACK_INDEX] = old_r_lld_pdu_tx_push;
    }
    #endif

    #if defined(ESP32S3) || defined(ESP32C3)
    if (old_r_lld_scan_process_rx_pkt != NULL) {
      (*r_ip_funcs_p)[RX_SCAN_CALLBACK_INDEX] = old_r_lld_scan_process_rx_pkt;
    }
    if (old_r_lld_scan_evt_start_cbk != NULL) {
      (*r_ip_funcs_p)[RX_SCAN_CONFIG_INDEX] = old_r_lld_scan_evt_start_cbk;
    }
    if (old_r_lld_test_freq2chnl != NULL) {
      (*r_ip_funcs_p)[LLD_FREQ_TO_CHANNEL_CALLBACK_INDEX] = old_r_lld_test_freq2chnl;
    }
    #endif
}

#ifdef ESP32
void setup_phy_hook() {
    void **phy_romfuncs = (void**)phy_get_romfuncs();
    old_phy_rom_loopback_mode_en = phy_romfuncs[ROM_LOOPBACK_MODE_ENABLE_INDEX];
    phy_romfuncs[ROM_LOOPBACK_MODE_ENABLE_INDEX] = (void*)jamming_callback;
}
#endif

void init_radio(rx_callback_t rx_callback) {
    radio_config.enabled = false;
    reception_queue = xQueueCreate(15, sizeof(rx_packet_t));
    transmission_queue = xQueueCreate(15, sizeof(tx_packet_t));

    if (reception_queue == NULL) {
      ESP_LOGE(TAG, "RX Queue creation failed\n");
      return;
    }
    if (transmission_queue == NULL) {
      ESP_LOGE(TAG, "TX Queue creation failed\n");
      return;
    }
    xTaskCreatePinnedToCore(&reception_packet_process, "reception_packet_process", 2048, NULL, 6, &rxProcessHandle, 0);
    xTaskCreatePinnedToCore(&transmission_packet_process, "transmission_packet_process", 2048, NULL, 6, &txProcessHandle, 0);

    hci_init();
    radio_config.rx_callback = rx_callback;
    setup_esperanto_hooks();

    #ifdef ESP32
    setup_phy_hook();
    #endif
}


void close_radio() {

  stop_radio();

  // Restore EM frequency table to legit value
  configure_frequency(2480);

  restore_esperanto_hooks();

  if (rxProcessHandle != NULL) {
    vTaskDelete(rxProcessHandle);
  }
  if (txProcessHandle != NULL) {
    vTaskDelete(txProcessHandle);
  }

}

void stop_radio() {
    if (radio_config.enabled) {
      if (radio_config.mode == MODE_RX) {
          hci_stop_scan();
      }
      else if (radio_config.mode == MODE_TX) {
          hci_stop_tx_test_mode();
      }
      else if (radio_config.mode == MODE_JAMMER) {
      }
      radio_config.enabled = false;
  }
}


void start_radio(radio_mode_t mode) {
    radio_config.mode = mode;
    if (mode == MODE_RX) {
        hci_start_scan();
    }
    else if (mode == MODE_TX) {
        #ifdef ESP32
        hci_start_tx_test_mode_v1();
        #endif

        #if defined(ESP32S3) || defined(ESP32C3)
        if (radio_config.data_rate == DATARATE_1M) {
          hci_start_tx_test_mode_v2(1);
        }
        else {
          hci_start_tx_test_mode_v2(2);
        }
        #endif
  }
  else if (mode == MODE_JAMMER) {
    #ifdef ESP32
    radio_config.enabled = true;
    chip7_sleep_params = 0;
    esp_bt_controller_shutdown();

    esp_phy_erase_cal_data_in_nvs();

    esp_phy_enable();
    i2c_master_reset();
    bb_init();
    #endif
    #if defined(ESP32S3) || defined(ESP32C3)
    ESP_LOGE(TAG, "Jammer mode not supported on ESP32S3/ESP32C3.");
    #endif
  }
}

void transmit_packet(uint8_t* packet, size_t size) {
    if (radio_config.enabled && radio_config.mode == MODE_TX) {
      tx_packet_t tx_packet;
      tx_packet.packet = (uint8_t*)malloc(sizeof(uint8_t)*size);
      memcpy(tx_packet.packet, packet, size);

      if (radio_config.swapping) {
        for (size_t i=0;i<size;i++) {
          tx_packet.packet[i] = swap(tx_packet.packet[i]);
        }
      }

      tx_packet.size = size;

      if (xQueueSend(transmission_queue, (void *)&tx_packet, ( TickType_t ) 0) != pdTRUE) {
          ESP_LOGD(TAG, "Failed to enqueue received packet. Queue full.");
          free(tx_packet.packet);
      }
    }
}
