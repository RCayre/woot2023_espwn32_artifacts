#ifndef ESPERANTO_H
#define ESPERANTO_H

#include "hci.h"
#include "helpers.h"

#define LLD_TEST_MODE_RX                    0x1D
#define LLD_TEST_MODE_TX                    0x1E

#ifdef ESP32
#define TX_LOOP_CALLBACK_INDEX              595
#define TX_PUSH_CALLBACK_INDEX              598

#define RX_SCAN_CALLBACK_INDEX              603
#define LLD_PDU_TX_PUSH_CALLBACK_INDEX      598
#define RX_SCAN_CONFIG_INDEX                641
#define CONTROL_STRUCTURE_IN_EM_OFFSET      0x43A
#define FREQUENCY_TABLE_IN_EM_OFFSET        0x40

#define CNTL_REG_IN_CS_OFFSET               0
#define SYNC_REG_IN_CS_OFFSET               6
#define HOPCTRL_REG_IN_CS_OFFSET            16
#define RXMAXBUF_REG_IN_CS_OFFSET           34
#define TXDESCPTR_REG_IN_CS_OFFSET          22
#define RX_DESCRIPTOR_IN_EM_OFFSET 0x950


#define RWBLECNTL_REG_ADDRESS               0x3ff71200
#define HEADER_IN_EM_OFFSET                 0x94c
#define PACKET_STATUS_IN_EM_OFFSET          0x94a
#define CURRENT_RX_FIFO_ADDRESS             0x3ffb8d74
#define FIFO_INDEX_OFFSET                   0x5c8

#define ROM_LOOPBACK_MODE_ENABLE_INDEX      (10)
#endif

#if defined(ESP32S3) || defined(ESP32C3)
#define RX_SCAN_CALLBACK_INDEX              258
#define RX_SCAN_CONFIG_INDEX                268
#define CONTROL_STRUCTURE_IN_EM_OFFSET      0x400
#define FREQUENCY_TABLE_IN_EM_OFFSET        0x100

#define LLD_FREQ_TO_CHANNEL_CALLBACK_INDEX  128


#define CNTL_REG_IN_CS_OFFSET               0
#define THRCNTL_RATECNTL_REG_IN_CS_OFFSET   4
#define SYNC_REG_IN_CS_OFFSET               12
#define TXDESCPTR_REG_IN_CS_OFFSET          28
#define HOPCTRL_REG_IN_CS_OFFSET            22
#define RXMAXBUF_REG_IN_CS_OFFSET           40
#define RWBLECNTL_REG_ADDRESS               0x60031000
#define HEADER_IN_EM_OFFSET                 0x1000
extern void *p_lld_env;
#define CURRENT_RX_FIFO_ADDRESS             p_lld_env
#define FIFO_INDEX_OFFSET                   0xd8
#endif


#ifdef ESP32
extern void set_chan_freq_sw_start(uint8_t chan_freq,int16_t freq_offset,uint8_t crystal_select);
extern uint32_t ram_start_tx_tone (uint32_t tone1, int freq1, int tone1_atten, uint32_t tone2, int freq2, int tone2_atten);
extern void phy_dis_hw_set_freq();
extern void **phy_get_romfuncs();

extern void bb_init();
extern void i2c_master_reset();
extern void esp_bt_controller_shutdown();
#endif

/**
 * EM TX node descriptor.
 *
 * This structure stores information about a TX PDU.
 **/
struct em_buf_tx_desc
{
    /// tx pointer
    uint16_t txptr;
    /// tx header
    uint16_t txheader;
    /// tx data pointer
    uint16_t txdataptr;
    /// tx data length extension info
    uint16_t txdle;
};


extern void **r_ip_funcs_p[738];

#ifdef ESP32
uint16_t r_em_buf_rx_free(uint32_t desc);
#define BUF_RX_FREE r_em_buf_rx_free
#endif
#if defined(ESP32S3) || defined(ESP32C3)
uint16_t r_ble_util_buf_rx_free(uint32_t desc);
#define BUF_RX_FREE r_ble_util_buf_rx_free
#endif

typedef void (*rx_callback_t)(uint8_t *packet,size_t size, int8_t rssi, int frequency);

#ifdef ESP32
typedef int (*F_r_lld_evt_rx)(void* evt, uint32_t nb_rx_desc);
typedef int (*F_r_llm_set_scan_en)(uint8_t *scan_param);

typedef int (*F_r_lld_pdu_tx_push)(int param_1,uint8_t* param_2);
#endif

#if defined(ESP32S3) || defined(ESP32C3)
typedef void (*F_r_lld_scan_process_rx_pkt)(void* evt);
typedef void (*F_r_lld_scan_evt_start_cbk)(int param1,int param2,int param3);

typedef int (*F_r_lld_test_freq2chnl)(int freq);
#endif

typedef uint32_t (*F_phy_rom_loopback_mode_en)(uint8_t en);

// Helper structure to recover packet informations
typedef struct {
    #if defined(ESP32S3) || defined(ESP32C3)
    uint32_t unknown_1;
    #endif
    uint32_t header;
    uint32_t unknown_2;
    uint32_t unknown_3;
    #if defined(ESP32S3) || defined(ESP32C3)
    uint16_t unknown_4;
    uint16_t buffer_offset;
    #endif
} pkt_header_t;

typedef enum {
    DATARATE_1M = 0,
    DATARATE_2M = 1
} datarate_t;

typedef enum {
    MODE_RX = 0,
    MODE_TX = 1,
    MODE_JAMMER = 2
} radio_mode_t;

typedef struct {
    rx_callback_t rx_callback;
    uint32_t sync_word;
    uint32_t frequency;
    bool swapping;
    datarate_t data_rate;
    mode_t mode;
    bool enabled;
} radio_config_t;

typedef struct {
  uint8_t *packet;
  size_t total_size;
  int8_t rssi;
  int frequency;
} rx_packet_t;

typedef struct {
  uint8_t *packet;
  size_t size;
} tx_packet_t;
extern uint8_t *chip7_sleep_params;
extern void *r_emi_get_mem_addr_by_offset(uint16_t offset);

void set_frequency(uint32_t frequency);
void set_sync_word(uint32_t sync_word);
void set_swapping(bool swapping);
void set_data_rate(datarate_t datarate);

#if defined(ESP32S3) || defined(ESP32C3)
void configure_data_rate(datarate_t datarate);
#endif
void configure_max_rx_buffer_size(uint8_t max_size);
void configure_frequency(uint32_t frequency);
void disable_whitening();
void disable_frequency_hopping();
void configure_sync_word(uint32_t sync);
void configure_format(uint8_t format);
void disable_crc_checking();

void init_radio(rx_callback_t rx_callback);

void start_radio(radio_mode_t mode);
void stop_radio();

void close_radio();

void transmit_packet(uint8_t* packet, size_t size);
#endif
