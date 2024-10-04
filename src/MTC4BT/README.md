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
For more information visit https://mattzobricks.com/controllers/mtc4bt
