# Cross protocols artifacts

This artifact includes a minimal cross-protocol library (ESPeranto) as an IDF component, and a set of examples demonstrating its use.
It allows to alter the behaviour of the BLE controller by modifying some structures in exchange memory, to communicate with non natively supported protocol.

The example should be compatible with ESP32, ESP32C3 and ESP32S3.


The examples provided are the following one:

  * **ant_rx**: example of ANT+ packet reception.
  * **ant_tx**: example of ANT+ Heart Rate Monitor device simulation.
  * **ant_scan**: example of ANT+ scan mode.
  * **dot15d4_rx**: example of 802.15.4 packet reception. (ESP32-C3/ESP32-S3 only)
  * **dot15d4_tx**: example of 802.15.4 packet transmission. (ESP32-C3/ESP32-S3 only)
  * **riitek_rx**: example of Riitek packet reception.
  * **riitek_tx**: example of Riitek packet transmission (keystroke injection).
  * **jam_ble**: example of jamming attack targeting the three advertising channels. (ESP32 only)

## Requirements

You need to install Espressif SDK **in version 4.4**. Follow the guide here:
[link](https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/get-started/index.html)

## Build a specific example

* Go to the example directory.
`
$ cd ant_tx
`

Clean the environment:
`
idf.py fullclean
`

Pick your target (esp32, esp32s3 or esp32c3):
`
idf.py set-target esp32
`

Build the example:
`
idf.py build
`

Flash it on the board and monitor the output over serial port:
`
idf.py flash && idf.py monitor
`