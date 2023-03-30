#ifndef HCI_H
#define HCI_H
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "common.h"

/*
Minimal HCI implementation used by Esperanto component to trigger
reception (scan mode) and transmission (TX test mode).
*/

/* HCI Event opcodes and status codes */
#define HCI_COMPLETE_COMMAND_OPCODE           (0x0e)
#define HCI_COMMAND_SUCCESSFUL                (0x00)

/* HCI Command types */
#define H4_TYPE_COMMAND                       (0x01)

/*  HCI Command Opcode group Field (OGF) */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS       (0x03 << 10)
#define HCI_GRP_BLE_CMDS                      (0x08 << 10)

/*  HCI Command Opcode Command Field (OCF) */
#define HCI_RESET                             (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_SET_EVT_MASK                      (0x0001 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)

/* Advertising Commands. */
#define HCI_BLE_WRITE_ADV_ENABLE              (0x000A | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_DATA                (0x0008 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_PARAMS              (0x0006 | HCI_GRP_BLE_CMDS)

/* Scan commands */
#define HCI_BLE_WRITE_SCAN_PARAM              (0x000B | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_SCAN_ENABLE             (0x000C | HCI_GRP_BLE_CMDS)


/* Test commands */
#define HCI_BLE_TX_TEST_MODE_V1                (0x001E | HCI_GRP_BLE_CMDS)
#define HCIC_PARAM_SIZE_TX_TEST_MODE_V1        (3)

#define HCI_BLE_TX_TEST_MODE_V2                (0x0034 | HCI_GRP_BLE_CMDS)
#define HCIC_PARAM_SIZE_TX_TEST_MODE_V2        (4)

#define HCI_BLE_TEST_MODE_END                  (0x001F | HCI_GRP_BLE_CMDS)


/* HCI Command length. */
#define HCIC_PARAM_SIZE_WRITE_ADV_ENABLE        (1)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS    (15)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA      (31)
#define HCIC_PARAM_SIZE_SET_EVENT_MASK          (8)
#define HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAM    (7)
#define HCIC_PARAM_SIZE_BLE_WRITE_SCAN_ENABLE   (2)
#define HCI_H4_CMD_PREAMBLE_SIZE                (4)

// Initialize HCI module.
void hci_init();

// Reset HCI module.
void hci_reset();

// Start and stop Transmission using TX Test mode
void hci_start_tx_test_mode_v1();
void hci_start_tx_test_mode_v2(uint8_t phy);
void hci_stop_tx_test_mode();

// Start and stop Reception using scan mode
void hci_start_scan();
void hci_stop_scan();
#endif
