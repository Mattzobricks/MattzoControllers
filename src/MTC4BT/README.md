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

You can run VSCode on Windows, Linux or Mac and there are loads of handy plug-ins for it. It made our lives so much easier and we believe it will do the same for you. Chances are we move other controller projects to Visual Studio Code as well in the future.

But how do I setup this new environment, you might ask?
For this purpose we have setup a [Getting Started guide](docs/README.md).

---
General Configuration
---
In the root of the workspace you'll find a file [platformio.example.ini](platformio.example.ini). You should copy it and rename it to `platformio.ini`. 

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
It is now possible to make a wired network connection using the W5500 module.

TODO: more documentation here.

---
For more information visit https://mattzobricks.com/controllers/mtc4bt