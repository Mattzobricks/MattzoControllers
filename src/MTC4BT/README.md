# Mattzo Train Controller for Bluetooth devices (MTC4BT)


The MTC4BT controller acts as a bridge between Bluetooth (Low Energy) devices and Rocrail.  
It currently supports the following type of BLE devices:
- [Lego Powered Up](https://www.lego.com/nl-nl/product/hub-88009)
- [SBrick](https://sbrick.com/)

> Please note the firmware only runs on [ESP32](https://www.espressif.com/en/products/socs/esp32) MCU's. 

It has been tested succesfully with these boards:
- AZDelivery ESP32 NodeMCU Module (verified by [MattzoBricks](https://mattzobricks.com/forums/users/rbrink))
- DOIT DEVKIT V1 ESP32-WROOM-32 Development Board (verified by [Steven Elston](https://mattzobricks.com/forums/users/steve1814))

If you have tested the firmware succesfully with a different board, please let us know through [our forum](https://mattzobricks.com/forums/forum/mattzobricks-forum).

---
Getting Started
---
For this project we have moved away from Arduino IDE in favour of Visual Studio Code and PlatformIO. We wanted to try this modern development environment for this project and have no regrets whatsoever. 

You can run VSCode on Windows, Linux or Mac and there are loads of handy plug-ins for it. The mandatory extension is 'PlatformIO IDE', this can be installed by pressing on the gear on the bottom left of the screen en press "Extensions", the search the "Extension in Marketplace" for "PlatformIO", click "intall" and you are ready to go.  It made our lives so much easier and we believe it will do the same for you. Chances are we move other controller projects to Visual Studio Code as well in the future.

An easy way to start is to open in VS-Code the `MattzoControllers.code-workspace` in the root of this repository. You can open it in VS-Code under 'File->Open Workspace from file...', and then you can start edditing.

But how do I setup this new environment, you might ask?
For this purpose we have setup a [Getting Started guide](docs/README.md).

---
General Configuration
---
In the root of the workspace you'll find a file [my_platformio.ini.example](my_platformio.ini.example). You should copy it and rename it to `my_platformio.ini`. 

Please do not edit the platformio.ini, this one comes with the project and has the correct library references and versions. It also makes shure it can find the used libraries.

This file is **NOT** touched when you update the code, so your local configuratians stay and all should compile.

---
Network Configuration
---
[data_example/network_config.json](data_example/network_config.json)

---
Controller Configuration
---
[data_example/controller_config.json](data_example/controller_config.json)

---
Using the W5500 ethernet module
---
It is now possible to make a wired network connection using the W5500 module. To enable it in the code, add -DWIRED in `my_platformio.ini`.

For now the pins should be connected as folows:

| W5500 pin  | ESP32 pin |
|---|---|
| RESET | GPIO 26 |
| CS/SS  | GPIO 5  |
| MOSI  | GPIO 23 |
| MISO  | GPIO 19 |
| SCK   | GPIO 18 |

GND and 3V3 are also connected, the other pins are not connected.

If the code is compiled with the -DWIRED, it will try and connect to the network using the W5500 module. If there is no networkcable connected, it will fall back to wifi. This is only done at the **STARTUP** of the module, it will **NOT FALLBACK to WiFI** if you disconnect the cable during normal operation, you have to restart the ESP!

OTA will not work when the network connction is over the cable, disconnect the cable and restart the module so it starts in WiFi mode. This limitation is caused by the ArduinoOTA library being hard wired (pun intended) to Wifi.h (at the time of writing). 

The IP address is different for Wifi and cable because an other MAC address is used.

---
For more information visit https://mattzobricks.com/controllers/mtc4bt
