#include "hci.h"

static const char* TAG = "ESPERANTO-HCI";

void on_controller_ready() {
    /* Callback called when the controller is triggered */
    ESP_LOGI(TAG, "HCI - Controller is ready.");
}

int on_hci_event(uint8_t *data, uint16_t len) {
    /* Callback called when an HCI event is received. */
    if (data[1] == HCI_COMPLETE_COMMAND_OPCODE) {
        if (data[6] == HCI_COMMAND_SUCCESSFUL) {
            ESP_LOGI(TAG, "HCI - command successful ! [opcode = 0x%02x]", data[4]);
        }
        else {
            ESP_LOGE(TAG, "HCI - command failure. [opcode = 0x%02x | reason: 0x%02x]", data[4], data[6]);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

esp_vhci_host_callback_t vhci_host_cb = {
    on_controller_ready,
    on_hci_event
};

static void send_hci_command_reset() {
    /* Send HCI command reset. */

    uint8_t hci_command[HCI_H4_CMD_PREAMBLE_SIZE+0] = {
      H4_TYPE_COMMAND,
      (uint8_t)HCI_RESET,(uint8_t)(HCI_RESET >> 8),0x00
    };

    while (!esp_vhci_host_check_send_available());
    esp_vhci_host_send_packet(hci_command, HCI_H4_CMD_PREAMBLE_SIZE+0);
}

static void send_hci_command_set_event_mask() {
    /* Send HCI command set event mask. */

    uint8_t hci_command[HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_SET_EVENT_MASK] = {
        H4_TYPE_COMMAND,
        (uint8_t)HCI_SET_EVT_MASK,(uint8_t)(HCI_SET_EVT_MASK >> 8),
        HCIC_PARAM_SIZE_SET_EVENT_MASK,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20
    };

    while (!esp_vhci_host_check_send_available());
    esp_vhci_host_send_packet(hci_command, HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_SET_EVENT_MASK);
}

static void send_hci_command_set_scan_parameters() {
    /* Configure basic scan parameters. */
    uint8_t scan_type = 0x01; // Passive scanning
    uint16_t scan_interval = 0x50; // 50 ms
    uint16_t scan_window = 0x30; // 30 ms

    uint8_t own_addr_type = 0x00; // Public Device Address (default).
    uint8_t filter_policy = 0x00; // Accept all packets excpet directed advertising packets (default).

    uint8_t hci_command[HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAM] = {
        H4_TYPE_COMMAND,
        (uint8_t)HCI_BLE_WRITE_SCAN_PARAM,(uint8_t)(HCI_BLE_WRITE_SCAN_PARAM >> 8),
        HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAM,
        scan_type,
        (uint8_t)scan_interval,(uint8_t)(scan_interval >> 8),
        (uint8_t)scan_window,(uint8_t)(scan_window >> 8),
        own_addr_type,
        filter_policy
    };

    while (!esp_vhci_host_check_send_available());
    esp_vhci_host_send_packet(hci_command, HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAM);
}

void send_hci_command_set_scan_enable(bool enable) {
    /* Enable or disable scan mode. */

    uint8_t enable_scan = (enable ? 0x01 : 0x00);
    uint8_t filter_duplicates = 0x00; // Don't filter duplicates

    uint8_t hci_command[HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_BLE_WRITE_SCAN_ENABLE] = {
         H4_TYPE_COMMAND,
        (uint8_t)HCI_BLE_WRITE_SCAN_ENABLE,(uint8_t)(HCI_BLE_WRITE_SCAN_ENABLE >> 8),
        HCIC_PARAM_SIZE_BLE_WRITE_SCAN_ENABLE,
        enable_scan,
        filter_duplicates
    };

    while (!esp_vhci_host_check_send_available());
    esp_vhci_host_send_packet(hci_command, HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_BLE_WRITE_SCAN_ENABLE);
}

void send_hci_command_set_advertising_enable(bool enable) {
    /* Enable or disable advertising mode. */
    uint8_t enable_advertising = (enable ? 0x01 : 0x00);
    uint8_t hci_command[HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_WRITE_ADV_ENABLE] = {
         H4_TYPE_COMMAND,
        (uint8_t)HCI_BLE_WRITE_ADV_ENABLE,(uint8_t)(HCI_BLE_WRITE_ADV_ENABLE >> 8),
        HCIC_PARAM_SIZE_WRITE_ADV_ENABLE,
        enable_advertising
    };
    while (!esp_vhci_host_check_send_available());
    esp_vhci_host_send_packet(hci_command, HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_WRITE_ADV_ENABLE);
}

void send_hci_command_set_advertising_parameters() {
    /* Minimum and maximum Advertising interval are set in terms of slots. Each slot is of 625 microseconds. */
    uint16_t adv_int_min = 0x30;
    uint16_t adv_int_max = 0x30;

    uint8_t adv_type = 0; // Connectable undirected advertising (ADV_IND).

    uint8_t own_addr_type = 0; // Public Address (0x00)

    uint8_t peer_addr_type = 0; // Public device address
    uint8_t peer_addr[6] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85};

    uint8_t adv_chn_map = 0x04; // only channel 39

    uint8_t adv_filter_policy = 0; // Disable whitelist

    uint8_t hci_command[HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS] = {
         H4_TYPE_COMMAND,
        (uint8_t)HCI_BLE_WRITE_ADV_PARAMS,(uint8_t)(HCI_BLE_WRITE_ADV_PARAMS >> 8),
        HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS,
        (uint8_t)adv_int_min,(uint8_t)(adv_int_min >> 8),
        (uint8_t)adv_int_max,(uint8_t)(adv_int_max >> 8),
        adv_type,
        own_addr_type,
        peer_addr_type,
        peer_addr[5], peer_addr[4], peer_addr[3], peer_addr[2], peer_addr[1], peer_addr[0],
        adv_chn_map,
        adv_filter_policy
    };

    while (!esp_vhci_host_check_send_available());
    esp_vhci_host_send_packet(hci_command, HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS);
}

void send_hci_command_set_advertising_data() {
    /* Configure basic advertising data. */

    // Configure arbitrary advertising data.
    uint8_t adv_data[31];
    for (int i=0;i<31;i++) {
      adv_data[i] = 0xA0 + i;
    }
    uint8_t adv_data_len = 31; // Use maximum length.

    uint8_t hci_command[HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1] = {
         H4_TYPE_COMMAND,
        (uint8_t)HCI_BLE_WRITE_ADV_DATA,(uint8_t)(HCI_BLE_WRITE_ADV_DATA >> 8),
        HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1,
        adv_data_len,
    };
    memcpy(&hci_command[5], adv_data, 31);
    while (!esp_vhci_host_check_send_available());
    esp_vhci_host_send_packet(hci_command, HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1);
}

void send_hci_command_tx_test_mode_v1() {
    /* Configure TX test mode on channel 39, with maximal packet length (version 1). */

    uint8_t channel = 39;
    uint8_t length = 0xFF;
    uint8_t packet_payload = 0x06;

    uint8_t hci_command[HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_TX_TEST_MODE_V1] = {
         H4_TYPE_COMMAND,
        (uint8_t)HCI_BLE_TX_TEST_MODE_V1,(uint8_t)(HCI_BLE_TX_TEST_MODE_V1 >> 8),
        HCIC_PARAM_SIZE_TX_TEST_MODE_V1,
        channel,
        length,
        packet_payload
    };
    while (!esp_vhci_host_check_send_available());
    esp_vhci_host_send_packet(hci_command, HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_TX_TEST_MODE_V1);
}

void send_hci_command_tx_test_mode_v2(uint8_t phy) {
    /* Configure TX test mode on channel 39, with maximal packet length and a specific phy (version 2). */

    uint8_t channel = 39;
    uint8_t length = 0xFF;
    uint8_t packet_payload = 0x06;

    uint8_t hci_command[HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_TX_TEST_MODE_V2] = {
         H4_TYPE_COMMAND,
        (uint8_t)HCI_BLE_TX_TEST_MODE_V2,(uint8_t)(HCI_BLE_TX_TEST_MODE_V2 >> 8),
        HCIC_PARAM_SIZE_TX_TEST_MODE_V2,
        channel,
        length,
        packet_payload,
        phy
    };
    while (!esp_vhci_host_check_send_available());
    esp_vhci_host_send_packet(hci_command, HCI_H4_CMD_PREAMBLE_SIZE+HCIC_PARAM_SIZE_TX_TEST_MODE_V2);
}


void send_hci_command_tx_test_stop() {
    /* Stop TX test mode. */

    uint8_t hci_command[HCI_H4_CMD_PREAMBLE_SIZE] = {
         H4_TYPE_COMMAND,
        (uint8_t)HCI_BLE_TEST_MODE_END,(uint8_t)(HCI_BLE_TEST_MODE_END >> 8)
    };
    while (!esp_vhci_host_check_send_available());
    esp_vhci_host_send_packet(hci_command, HCI_H4_CMD_PREAMBLE_SIZE);
}

void hci_init() {
    // Init the HCI interface.
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    int ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(TAG,"Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG,"Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGE(TAG,"Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
    esp_vhci_host_register_callback(&vhci_host_cb);
    send_hci_command_reset();
}

void hci_reset() {
  /* Reset HCI interface. */
  send_hci_command_reset();
}

void hci_start_adv() {
  /* Start advertising mode. */

    send_hci_command_set_advertising_parameters();
    send_hci_command_set_advertising_data();
    send_hci_command_set_advertising_enable(true);
}

void hci_stop_adv() {
    /* Stop advertising mode. */

    send_hci_command_set_advertising_enable(false);
}

void hci_start_scan() {
    /* Start scan mode. */
    send_hci_command_set_event_mask();
    send_hci_command_set_scan_parameters();
    send_hci_command_set_scan_enable(true);
}

void hci_stop_scan() {
    /* Stop scan mode. */
    send_hci_command_set_scan_enable(false);
}


void hci_start_tx_test_mode_v1() {
    /* Start TX test in mode 1. */

  send_hci_command_tx_test_mode_v1();
}

void hci_start_tx_test_mode_v2(uint8_t phy) {
    /* Start TX test in mode 2. */

  send_hci_command_tx_test_mode_v2(phy);
}

void hci_stop_tx_test_mode() {
    /* Stop TX test mode. */

  send_hci_command_tx_test_stop();
}
