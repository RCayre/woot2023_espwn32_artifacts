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

You need to install Espressif SDK **in version 4.4**. A full complete guide is available [here](https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/get-started/index.html). 

Once the SDK is configured, clone the repository and go to the cross_protocols folder:

```
$ git clone https://github.com/RCayre/woot2023_espwn32_artifacts

$ cd woot2023_espwn32_artifacts/cross_protocols
```

You can then build one of the provided example.

## Build a specific example

1. Go to the example directory. For example, if you want to build the **ant\_tx** example:

`
$ cd ant_tx
`

2. Clean the environment:

`
idf.py fullclean
`

3. Pick your target (esp32, esp32s3 or esp32c3):

`
idf.py set-target esp32
`

4. Build the example:

`
idf.py build
`

5. Flash it on the board and monitor the output over serial port:

`
idf.py flash && idf.py monitor
`

## Using the library

First, you need to add *esperanto* component to your project. 
You can do it easily by modifying your CMakeLists.txt file and add the following line:

```
set(EXTRA_COMPONENT_DIRS <path_to_library>/esperanto)
```


You can then import the library by including the header file:

```c
#include "esperanto.h"
```

The library exposes several functions, allowing you to alter the BLE core behaviour to interact with non-native protocols:

```c
void set_frequency(uint32_t frequency);
void set_sync_word(uint32_t sync_word);
void set_swapping(bool swapping);
void set_data_rate(datarate_t datarate);
void init_radio(rx_callback_t rx_callback);
void start_radio(radio_mode_t mode);
void stop_radio();
void close_radio();
void transmit_packet(uint8_t* packet, size_t size);
```

Three modes are currently supported: reception (MODE_RX), transmission (MODE_TX) and jammer (MODE_JAMMER).
First, you need to initialize the radio and enable BLE core using *init_radio* function. If you want to receive non-native packets, you can provide a reception callback matching the following signature:

```c
void my_reception_callback(uint8_t *packet,size_t size, int8_t rssi, int frequency) {
 // Process the received packets
}
```
This callback will be triggered every time a packet is received by the core.

You can then initialize the radio using:

```c
init_radio(my_reception_callback);
```

Then, you can configure the radio parameters according to the protocol to match. 

You can configure the frequency by providing a frequency included between 2402 and 2480 MHz:

```c
set_frequency(2425); // configure 2425 MHz as central frequency
```

You can configure the endianness using:

```c
set_swapping(true); // use big endian format
```

or :

```c
set_swapping(false); // use little endian format
```

You can provide an arbitrary synchronization word, which can be used to match a specific radio pattern:

```c
set_sync_word(0x11223344); // configure 0x11223344 as synchronization word
```

On ESP32-C3 and ESP32-S3, you can either configure 1Mbps or 2Mbps datarate using:

```c
set_data_rate(DATARATE_1M); // configure 1Mbps datarate
```

or:

```c
set_data_rate(DATARATE_2M); // configure 2Mbps datarate
```

Then, you can start the radio in the mode you want to use.

For example, if you want to start the reception mode:

```c
// Start the reception mode and starts receiving packets
start_radio(MODE_RX);
```

You can also start the transmission mode:

```c
// Start the reception mode and starts receiving packets
start_radio(MODE_RX);
```

and transmit packets using:
```c
uint8_t mypacket[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a};
transmit_packet(mypacket, 10);
```

The jammer mode toggles the radio calibration mode, hooks the loobpack function and iterates over the three BLE advertising channels (37, 38 and 39) while transmitting a glitched signal:

```c
start_radio(MODE_JAMMER);
```

You can stop the radio using:

```c
stop_radio();
```

And close it using:

```c
close_radio();
```
