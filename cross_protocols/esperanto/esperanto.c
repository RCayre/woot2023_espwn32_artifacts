#include "esperanto.h"

static const char* TAG = "ESPERANTO-HOOKS";

/* Structure storing radio configuration parameters */
radio_config_t radio_config;

/* Reception and transmission queues */
static QueueHandle_t reception_queue;
static QueueHandle_t transmission_queue;

/* Reception and transmission handles */
TaskHandle_t rxProcessHandle = NULL;
TaskHandle_t txProcessHandle = NULL;


/* Function pointers used to store hooked functions */
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
    /* This function accesses a memory zone in exchange memory. */
    return (void*)(0x3ffb0000+offset);
}
#endif

#ifdef ESP32
/**
 * rx_custom_callback_esp32()
 *
 * @brief: This hook is called when a reception packet is processed in scan mode (ESP32).
 **/
int rx_custom_callback_esp32(void* evt, uint32_t nb_rx_desc) {
  // Extract header from memory offset
  pkt_header_t *p_pkt = (pkt_header_t *)(r_emi_get_mem_addr_by_offset(HEADER_IN_EM_OFFSET));
  uint16_t pkt_status = *(uint16_t *)r_emi_get_mem_addr_by_offset(PACKET_STATUS_IN_EM_OFFSET);

  // Redirect to the old function pointer if no packet available.
  if (nb_rx_desc == 0 || ((pkt_status & 0x13f) != 0)) {
      return old_r_lld_evt_rx(evt,nb_rx_desc);
  }
  int j;

  // Redirect to the old function pointer if needed
  if ((*((uint8_t *)evt + 0x72) & 0x10) == 0) {
    // Retrieve the fifo index
    uint8_t fifo_index = ((uint8_t *)CURRENT_RX_FIFO_ADDRESS)[FIFO_INDEX_OFFSET];

    // Iterate over the available descriptors
    for (int k=0; k<nb_rx_desc; k++) {

      j = (fifo_index + k) % 8;

      // Extract the header of the current packet
      uint32_t pkt_header = p_pkt[j].header;

      int8_t rssi = (pkt_header>>16) & 0xff;
      uint8_t pkt_size = (pkt_header >> 8) & 0xff;
      if (pkt_size > 0 && pkt_size < 255) {
        uint8_t opcode = pkt_header & 0xFF;

        // Access the buffer offset
        uint16_t *buffer_offset = (uint16_t*)(r_emi_get_mem_addr_by_offset(RX_DESCRIPTOR_IN_EM_OFFSET) + 12*j);
        uint8_t *p_pdu = (uint8_t *)(r_emi_get_mem_addr_by_offset(*buffer_offset));

        // Build a rx_packet
        rx_packet_t packet;

        // Fill metadata information and copy the packet
        packet.total_size = 4+2+pkt_size;
        packet.rssi = rssi;
        packet.frequency = radio_config.frequency;
        packet.packet = (uint8_t*)malloc(sizeof(uint8_t)*packet.total_size);
        if (packet.packet != NULL) {

          // Copy the synchronization word, the header and and the packet content into our buffer
          memcpy(packet.packet,&radio_config.sync_word,4);
          packet.packet[4] = opcode;
          packet.packet[5] = pkt_size;
          memcpy(&packet.packet[6],p_pdu, pkt_size);

          // Swap endianness if needed
          if (radio_config.swapping) {
            for (int i=0;i<pkt_size;i++) packet.packet[i] = swap(packet.packet[i]);
          }
          // Correct a bitflip (occurs only with ESP32)
          packet.packet[4] = packet.packet[4] ^ (1 << 3);

          // Add the packet to the reception queue
          if (xQueueSend(reception_queue, (void *)&packet, ( TickType_t ) 0) != pdTRUE) {
              ESP_LOGD(TAG, "Failed to enqueue received packet. Queue full.");
              free(packet.packet);
          }
        }

      }
      // Alter the header of packet to prevent processing by the BLE stack
      p_pkt[j].header = 0;
    }
  }
  return old_r_lld_evt_rx(evt,nb_rx_desc);
}

/**
 * rx_custom_callback_esp32()
 *
 * @brief: This hook is is called when the scan mode is enabled (ESP32).
 **/
int rx_custom_config_esp32(uint8_t *scan_param) {

  // Call the old hooked pointer
  int ret = old_r_llm_start_scan_en(scan_param);

  // If the radio mode is enabled, alter the control structure
  if (radio_config.mode == MODE_RX && radio_config.enabled) {

    // Force a more flexible format (TEST_MODE_RX instead of ADV_FORMAT)
    configure_format(LLD_TEST_MODE_RX);

    // Configure an specific sync word
    configure_sync_word(radio_config.sync_word);

    // Configure a specific frequency
    configure_frequency(radio_config.frequency);

    // Disable frequency hopping
    disable_frequency_hopping();

    // Allow packets up to 255 bytes
    configure_max_rx_buffer_size(0xFF);

    // Disable CRC checking and disable whitening
    disable_crc_checking();
    disable_whitening();
    radio_config.enabled = true;
  }
  return ret;
}
#endif

#if defined(ESP32S3) || defined(ESP32C3)
/**
 * rx_custom_callback_esp32c3()
 *
 * @brief: This hook is called when a reception packet is processed in scan mode (ESP32S3/ESP32C3).
 **/
void rx_custom_callback_esp32c3(void* evt) {
    if (radio_config.mode == MODE_RX && radio_config.enabled) {

      // Extract header from memory offset
      pkt_header_t *p_pkt = (pkt_header_t *)(r_emi_get_mem_addr_by_offset(HEADER_IN_EM_OFFSET));
      uint8_t fifo_index = ((uint8_t *)CURRENT_RX_FIFO_ADDRESS)[FIFO_INDEX_OFFSET];

      uint32_t pkt_header = p_pkt[fifo_index].header;

      if (pkt_header == 0) {
          return;
      }

      // Retrieve metadata about the packet (packet size, rssi, etc)
      int8_t rssi = (pkt_header>>16) & 0xff;
      uint8_t pkt_size = (pkt_header >> 8) & 0xff;
      uint8_t opcode = pkt_header & 0xFF;

      // Access the buffer in exchange memory
      uint16_t buffer_offset = p_pkt[fifo_index].buffer_offset;
      uint8_t *p_pdu = (uint8_t *)(r_emi_get_mem_addr_by_offset(buffer_offset));

      // Build a rx_packet structure
      rx_packet_t packet;

      // Fill our rx_packet structure with metadata information
      packet.total_size = 4+2+pkt_size;
      packet.rssi = rssi;
      packet.frequency = radio_config.frequency;
      // Allocate memory for our packet
      packet.packet = (uint8_t*)malloc(sizeof(uint8_t)*packet.total_size);
      if (packet.packet != NULL) {

        // Copy sync word, header and packet content
        memcpy(packet.packet,&radio_config.sync_word,4);
        packet.packet[4] = opcode;
        packet.packet[5] = pkt_size;
        memcpy(&packet.packet[6],p_pdu, pkt_size);
        // For some reason, hardware no whitening mode doesn't work on ESP32C3, let's perform the dewhitening manually
        dewhiten_ble(&packet.packet[4], pkt_size+2,39);

        // Swap endianness if needed
        if (radio_config.swapping) {
          for (int i=0;i<pkt_size;i++) packet.packet[i] = swap(packet.packet[i]);
        }

        // Add the packet to the reception queue
        if (xQueueSend(reception_queue, (void *)&packet, ( TickType_t ) 0) != pdTRUE) {
            ESP_LOGD(TAG, "Failed to enqueue received packet. Queue full.");
            free(packet.packet);
        }
      }
      // Aklter the packet header to prevent processing by the BLE stack.
      p_pkt[fifo_index].header = 0;

    }
    // Return to the scan process
    old_r_lld_scan_process_rx_pkt(evt);
}

/**
 * rx_custom_config_esp32c3()
 *
 * @brief: This hook is called when the scan mode is enabled. (ESP32C3/ESP32S3).
 **/
void rx_custom_config_esp32c3(int param_1,int param_2,int param_3) {
    // Enable scan
    old_r_lld_scan_evt_start_cbk(param_1,param_2,param_3);
    if (radio_config.mode == MODE_RX) {
      // Configure a more flexible format (TEST_MODE_RX instead of ADV)
      configure_format(LLD_TEST_MODE_RX);

      // Configure a specific datarate by altering the control structure
      configure_data_rate(radio_config.data_rate);

      // Configure a specific sync word by altering the control structure
      configure_sync_word(radio_config.sync_word);

      // Configure a specific frequency by altering the control structure and frequency table
      configure_frequency(radio_config.frequency);

      // Disable the frequency hopping to stay on a static channel
      disable_frequency_hopping();

      // Extend packet size up to 255 bytes
      configure_max_rx_buffer_size(0xFF);

      // Disable CRC checking and whitening
      disable_crc_checking();
      disable_whitening();

      radio_config.enabled = true;
    }
}
#endif

/**
 * set_data_rate()
 *
 * @brief: This function allows the user to select a specific datarate.
 * @param data_rate: darate to use (DATARATE_1M or DATARATE_2M).
 **/
void set_data_rate(datarate_t data_rate) {
    radio_config.data_rate = data_rate;
    #ifdef ESP32
    if (data_rate == DATARATE_2M) {
      ESP_LOGE(TAG, "ESP32 does not support LE 2M, use ESP32C3 or ESP32S3 instead.");
    }
    #endif
}

/**
 * set_frequency()
 *
 * @brief: This function allows the user to select a specific frequency.
 * @param frequency: frequency to use (in MHz).
 **/
void set_frequency(uint32_t frequency) {
    radio_config.frequency = frequency;
}


/**
 * set_sync_word()
 *
 * @brief: This function allows the user to select a specific sync word.
 * @param sync_word: synchronization word to use.
 **/
void set_sync_word(uint32_t sync_word) {
    radio_config.sync_word = sync_word;
}

/**
 * set_swapping()
 *
 * @brief: This function allows the user to switch endianness from little endian to big endian.
 * @param swapping: boolean indicating if Big Endian is enabled.
 **/
 void set_swapping(bool swapping) {
    radio_config.swapping = swapping;
}

#if defined(ESP32S3) || defined(ESP32C3)

/**
 * configure_data_rate()
 *
 * @brief: This function alters the control structure to configure a specific datarate.
 * @param data_rate: data_rate to configure.
 **/
void configure_data_rate(datarate_t data_rate) {
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);
    int rate = (data_rate == DATARATE_1M ? 0 : 1);
    control_structure[THRCNTL_RATECNTL_REG_IN_CS_OFFSET] = (rate << 2) | rate;
    control_structure[THRCNTL_RATECNTL_REG_IN_CS_OFFSET+1] = 0x10;
}
#endif

/**
 * configure_sync_word()
 *
 * @brief:This function alters the control structure to configure a specific synchronization word.
 * @param sync: sync word to configure.
 **/
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

/**
 * configure_max_rx_buffer_size()
 *
 * @brief:This  function alters the control structure to configure a specific max packet size.
 * @param format: format to configure.
 **/
void configure_format(uint8_t format) {
    /* This function alters the control structure to configure a specific format */
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);
    control_structure[CNTL_REG_IN_CS_OFFSET] = format;
}

/**
 * configure_max_rx_buffer_size()
 *
 * @brief:This function alters the control structure to configure a specific max packet size.
 **/
void configure_max_rx_buffer_size(uint8_t max_size) {
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);
    control_structure[RXMAXBUF_REG_IN_CS_OFFSET] = max_size;
}
/**
 * disable_frequency_hopping()
 *
 * @brief: This function alters the control structure to force the channel to 39 and disable frequency hopping.
 **/
void disable_frequency_hopping() {
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);
    control_structure[HOPCTRL_REG_IN_CS_OFFSET+1] = 0;
    control_structure[HOPCTRL_REG_IN_CS_OFFSET] = 39;
}
/**
 * configure_frequency()
 *
 * @brief: This function alters the frequency table in exchange memory to select an arbitrary frequency.
 * @param frequency: frequency to configure.
 **/
void configure_frequency(uint32_t frequency) {
    uint8_t offset = frequency - 2402;
    uint8_t *frequency_table = r_emi_get_mem_addr_by_offset(FREQUENCY_TABLE_IN_EM_OFFSET);
    frequency_table[39] = offset;
    radio_config.frequency = frequency;
}
/**
 * disable_crc_checking()
 *
 * @brief: This function alters RWBLECNTL register to disable CRC checking.
 **/
void disable_crc_checking() {
    uint32_t *rwblecntl_reg = (uint32_t*)RWBLECNTL_REG_ADDRESS;
    *rwblecntl_reg = *rwblecntl_reg | (1 << 16)| (1 << 17);
}


/**
 * disable_whitening()
 *
 * @brief: This function alters RWBLECNTL register to disable whitening.
 **/
void disable_whitening() {
    uint32_t *rwblecntl_reg = (uint32_t*)RWBLECNTL_REG_ADDRESS;
    *rwblecntl_reg = *rwblecntl_reg | (1 << 18);
}


/**
 * get_em_tx_desc()
 *
 * @brief: This function returns the TX descriptor currently linked to control structure.
 * @return: TX descriptor currently linked to control structure.
 **/
struct em_buf_tx_desc * get_em_tx_desc() {
    /* This function returns the TX descriptor currently linked to control structure. */
    uint8_t *control_structure = r_emi_get_mem_addr_by_offset(CONTROL_STRUCTURE_IN_EM_OFFSET);

    uint16_t txdescptr = control_structure[TXDESCPTR_REG_IN_CS_OFFSET] | (control_structure[TXDESCPTR_REG_IN_CS_OFFSET+1] << 8);
    return (struct em_buf_tx_desc *)r_emi_get_mem_addr_by_offset(txdescptr);
}


/**
 * reception_packet_process()
 *
 * @brief: This task processes the received packets and calls the user callback.
 **/
void reception_packet_process(void *pvParameters) {
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
      if (radio_config.rx_callback != NULL) {
        radio_config.rx_callback(rcv_packet->packet,rcv_packet->total_size, rcv_packet->rssi, rcv_packet->frequency);
      }
      memset(rcv_packet, 0, sizeof(rx_packet_t));
    }
  }
}
/**
 * transmission_packet_process()
 *
 * @brief: This task processes the transmitted packets and write them to exchange memory.
 **/
void transmission_packet_process(void *pvParameters) {
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
      // Retrieve the current TX descriptor
      struct em_buf_tx_desc *txdesc = get_em_tx_desc();
      // Get a pointer to the data buffer
      uint8_t *dataptr = r_emi_get_mem_addr_by_offset(txdesc->txdataptr);

      // Flush the data buffer
      memset(dataptr, 0, 0xFF);

      // Write our packet to the data buffer
      memcpy(dataptr, tx_packet->packet, tx_packet->size);
      vTaskDelay(10 / portTICK_PERIOD_MS);

      // Flush the data buffer
      memset(dataptr, 0, 0xFF);

      // Reset structure to wait for the next packet
      memset(tx_packet, 0, sizeof(tx_packet_t));

    }
  }
}


#ifdef ESP32
/**
 * tx_custom_config_esp32()
 *
 * @brief: Hook when the TX configuration mode is triggered (ESP32).
 **/
void tx_custom_config_esp32(int param_1,uint8_t* param_2) {
  /* This hook is called when the TX configuration mode is triggered (ESP32). */
  configure_frequency(radio_config.frequency);
  configure_format(LLD_TEST_MODE_TX);
  old_r_lld_pdu_tx_push(param_1,param_2);
  radio_config.enabled = true;
}

#endif


#if defined(ESP32S3) || defined(ESP32C3)
/**
 * tx_custom_config_esp32c3()
 *
 * @brief: Hook called when the loopback mode is enabled during full calibration.
 **/
int tx_custom_config_esp32c3(int freq) {
    /* This hook is called when the TX configuration mode is triggered (ESP32C3/ESP32S3). */
    configure_frequency(radio_config.frequency);
    configure_format(LLD_TEST_MODE_TX);

    // Get the TX buffer and flush it
    struct em_buf_tx_desc *txdesc = get_em_tx_desc();
    uint8_t *dataptr = r_emi_get_mem_addr_by_offset(txdesc->txdataptr);
    memset(dataptr, 0, 0xFF);

    // Call the hooked old function
    int chnl = old_r_lld_test_freq2chnl(freq);
    radio_config.enabled = true;
    return chnl;
}
#endif

#ifdef ESP32
/**
 * jamming_callback()
 *
 * @brief: Hook called when the loopback mode is enabled during full calibration.
 * @param en: boolean enabling or disabling the loopback mode.
 **/
uint32_t jamming_callback(uint8_t en) {
  esp_task_wdt_reset();
   if (radio_config.mode == MODE_JAMMER && radio_config.enabled) {
      // Disable hardware frequency control
      phy_dis_hw_set_freq();

      int i=0;
      // Loop until we exit jammer mode, and transmit glitched signal on the three BLE advertising channels
      while (radio_config.mode == MODE_JAMMER && radio_config.enabled) {
          set_chan_freq_sw_start(2,0,0);
          ram_start_tx_tone(1,0,20,0,0,0);
          set_chan_freq_sw_start(80,0,0);
          ram_start_tx_tone(1,0,20,0,0,0);
          set_chan_freq_sw_start(26,0,0);
          ram_start_tx_tone(1,0,20,0,0,0);
          i++;
          // Wait a little bit sometimes to prevent watchdog to be triggered
          if ((i % 10000) == 0) {
            vTaskDelay(10 / portTICK_RATE_MS);
          }
      }
  }
   return old_phy_rom_loopback_mode_en(en);
}
#endif

/**
 * restore_lld_hooks()
 *
 * @brief: configure the LLD hooks to alter on the fly scan and TX Test mode.
 **/
void setup_lld_hooks() {
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

/**
 * restore_lld_hooks()
 *
 * @brief: Restores the initial LLD hooks.
 **/
void restore_lld_hooks() {

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
/**
 * setup_phy_hook()
 *
 * @brief: Configure the physical layer hook.
 **/
void setup_phy_hook() {
    void **phy_romfuncs = (void**)phy_get_romfuncs();
    old_phy_rom_loopback_mode_en = phy_romfuncs[ROM_LOOPBACK_MODE_ENABLE_INDEX];
    phy_romfuncs[ROM_LOOPBACK_MODE_ENABLE_INDEX] = (void*)jamming_callback;
}
#endif


/**
 * initialize_radio()
 *
 * @brief: Initialize the radio.
 * @param rx_callback: function pointer to a reception callback.
 **/
void init_radio(rx_callback_t rx_callback) {
    /* This function initialize the radio. */
    radio_config.enabled = false;

    /* Initialize reception and transmission queues */
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
    /* Initialize reception and transmission tasks */
    xTaskCreatePinnedToCore(&reception_packet_process, "reception_packet_process", 2048, NULL, 6, &rxProcessHandle, 0);
    xTaskCreatePinnedToCore(&transmission_packet_process, "transmission_packet_process", 2048, NULL, 6, &txProcessHandle, 0);

    /* Init BLE core and HCI */
    hci_init();
    radio_config.rx_callback = rx_callback;
    /* Configure lld hooks */
    setup_lld_hooks();

    #ifdef ESP32
    /* Configure phy hooks */
    setup_phy_hook();
    #endif
}


/**
 * close_radio()
 *
 * @brief: Close the radio module.
 **/
void close_radio() {
  stop_radio();

  // Restore EM frequency table to legit value
  configure_frequency(2480);

  /* Restore lld hooks */
  restore_lld_hooks();

  /* If needed, delete reception and transmission tasks. */
  if (rxProcessHandle != NULL) {
    vTaskDelete(rxProcessHandle);
  }
  if (txProcessHandle != NULL) {
    vTaskDelete(txProcessHandle);
  }

}


/**
 * stop_radio()
 *
 * @brief: Stop the current radio operation.
 **/
void stop_radio() {
    if (radio_config.enabled) {
      if (radio_config.mode == MODE_RX) {
          // Stop TX test mode to stop arbitrary reception
          hci_stop_scan();
      }
      else if (radio_config.mode == MODE_TX) {
          // Stop TX test mode to stop arbitrary transmission
          hci_stop_tx_test_mode();
      }
      radio_config.enabled = false;
  }
}

/**
 * start_radio()
 *
 * @brief: Start the radio in a specific mode (MODE_RX, MODE_TX or MODE_JAMMER).
 * @param mode: radio mode to configure.
 **/
void start_radio(radio_mode_t mode) {
    /* Start the radio in a specific mode: TX, RX or JAMMER*/
    radio_config.mode = mode;
    if (mode == MODE_RX) {
        /* Starts Scan mode to trigger arbitrary reception */
        hci_start_scan();
    }
    else if (mode == MODE_TX) {
        /* Starts TX Test mode to trigger arbitrary transmission */
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
    /* Perform a sequence of operations allowing to trigger a full calibration of radio and trigger jamming */
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

/**
 * transmit_packet()
 *
 * @brief: Transmit a specific packet (fill the transmission packet queue).
 * @param packet: pointer to a buffer containing the packet to transmit.
 * @param size: size of buffer.
 **/
void transmit_packet(uint8_t* packet, size_t size) {
    if (radio_config.enabled && radio_config.mode == MODE_TX) {
      tx_packet_t tx_packet;
      tx_packet.packet = (uint8_t*)malloc(sizeof(uint8_t)*size);
      memcpy(tx_packet.packet, packet, size);

      // Switch endianness if needed
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
