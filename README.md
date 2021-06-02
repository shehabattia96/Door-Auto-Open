# Door Auto-open

Automatic door opening system that doesn't involve replacing any components in your existing door.

## Getting started

### Materials

- [ESP32 dev board](https://amzn.com/B0718T232Z)
- [USB to TTL programmer](https://amzn.com/B00IJXZQ7C) - For example, HiLetgo FT232RL. This is needed to flash the ESP32 board.
  - Wiring:
    - GND (programmer) -> GND (ESP)
    - CTS (programmer) -> not used
    - VCC (programmer) -> 5V (ESP)
    - TX (programmer) -> RX/U0R (ESP)
    - RX (programmer) -> TX/U0T (ESP)
  - When uploading, connect GND (ESP) -> IO0 (ESP). Press the reset button for good measure.
- Optional: Batteries/Battery holder or Power Supply (PS). Connect battery/PS to 5V and GND of ESP32. The power supply must be able to output at least 400mA.

### Pre-requisite extensions

This was developed on VSCode1.55 with the help of the extensions below.

- [PlatformIO](https://platformio.org/platformio-ide)

### Setting up MQTT server (mosquito)

1. Grab Mosquitto v2 from [https://mosquitto.org/download/](https://mosquitto.org/download/).
2. To allow the mqtt server to connect to/from the ESP32, you should create a config file with just this line:

    ``` listener 1883 0.0.0.0 ```

    Also whitelist port 1883 in your firewall. Please practice caution as this opens port 1883 on your machine to the network.
3. Run the `mosquito` executable using `./mosquito -v -c /path/to/conf/file`

> Note: Mosquitto v2 supports mqtt5. v1 does not. Make sure you grab v2 of mosquitto.

### Setting up your environment

1. Run `sh ./install.sh` to install the required lib dependencies
2. Touch a `.environment_variables.h` file in [./src/.environment_variables.h](./src/.environment_variables.h). Fill it out:

    ```C++
        #ifndef environmentvars_H
        #define environmentvars_H

        // debug options
        #define DEBUG_MODE true // if enabled, will print log messages to console.

        // mqtt
        #define mqttClientId "doorAutoOpen"
        #define mqttHost "192.168.1.202"
        #define mqttPort 1883
        // wifi
        #define wifiSsid "wifiAPtoConnectTo"
        #define wifiPassword "hunter2"

        #endif
    ```

3. Build and upload to the ESP32 using PlatformIO controls

    > Important: When uploading code to the AIThinker ESP32, make sure to ground the IO0 pin. If it's not grounded, you will not be able to upload code.

## References

The following sources were used to design and build this program.

- ESP32 Wifi events example code - author:espressif - link:[github](https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/station/main/station_example_main.c)
- ESP32 events - author:espressif - link:[github](https://github.com/espressif/esp-idf/blob/master/docs/en/api-reference/system/esp_event.rst)
- ESP32 SDCard example code - author:espressif - link:[github](https://raw.githubusercontent.com/espressif/esp-idf/master/examples/storage/sd_card/main/sd_card_example_main.c)
