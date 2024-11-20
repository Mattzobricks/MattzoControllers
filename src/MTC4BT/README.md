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
## Getting Started

For this project we have moved away from Arduino IDE in favour of Visual Studio Code and PlatformIO. We wanted to try this modern development environment for this project and have no regrets whatsoever. 

You can run VSCode on Windows, Linux or Mac and there are loads of handy plug-ins for it. The mandatory extension is 'PlatformIO IDE', this can be installed by pressing on the gear on the bottom left of the screen en press "Extensions", the search the "Extension in Marketplace" for "PlatformIO", click "intall" and you are ready to go.  It made our lives so much easier and we believe it will do the same for you. Chances are we move other controller projects to Visual Studio Code as well in the future.

An easy way to start is to open in VS-Code the `MattzoControllers.code-workspace` in the root of this repository. You can open it in VS-Code under 'File->Open Workspace from file...', and then you can start edditing.

But how do I setup this new environment, you might ask?
For this purpose we have setup a [Getting Started guide](docs/README.md).

---

## General Configuration

In the root of the workspace you'll find a file [my_platformio.ini.example](my_platformio.ini.example). You should copy it and rename it to `my_platformio.ini`. 

Please do not edit the platformio.ini, this one comes with the project and has the correct library references and versions. It also makes shure it can find the used libraries.

This file is **NOT** touched when you update the code, so your local configuratians stay and all should compile.


### Network Configuration

[data_example/network_config.json](data_example/network_config.json)

### Controller Configuration

[data_example/controller_config.json](data_example/controller_config.json)

Supported hub types
 - `PU` Lego powered up
 - `SBrick` SBrick
 - `BuWizz2` BuWizz (version 2.0 only)

 The BuWizz 2.0 hub has an additional configuration item `powerlevel` which can hold de following values (case sensitive!):
  - `disabled` Power is disabled
  -  `slow`
  -  `normal` Default if not specified
  -  `fast`
  -  `ldcrs`
  

### Using the W5500 ethernet module

It is now possible to make a wired network connection using the W5500 module. To enable it in the code, add -DWIRED in `my_platformio.ini`.

For now the pins should be connected as follows:

| W5500 pin  | ESP32 pin |
|---|---|
| RESET | GPIO 26 |
| CS/SS  | GPIO 5  |
| MOSI  | GPIO 23 |
| MISO  | GPIO 19 |
| SCK   | GPIO 18 |

GND and 3V3 are also connected, the other pins are not connected.

If the code is compiled with the -DWIRED, it will try and connect to the network using the W5500 module. If there is no network cable connected, it will fall back to wifi. This is only done at the **STARTUP** of the module, it will **NOT FALLBACK to WiFI** if you disconnect the cable during normal operation, you have to restart the ESP!

OTA will not work when the network connection is over the cable, disconnect the cable and restart the module so it starts in WiFi mode. This limitation is caused by the ArduinoOTA library being hard wired (pun intended) to Wifi.h (at the time of writing). 

The IP address is different for Wifi and cable because an other MAC address is used.

### Configuring the PURemote

#### Configuring the `controller_config.json`

The configuration has two modes, a range mode and a fixed mode.

##### Range mode

In the range mode, the configuration looks something like this:

```
   {
      "address": 99999,
      "name": "Lego",
      "bleHubs": [
        {
          "type": "PUController",
          "address": "e4:.....",
          "range": {
            "min": 1,
            "max": 10
          }
        }
      ]
    },
```

The fields `address` and `name` are not used, but there should be a unique value in it to prevent strange behaviour and it should not be in the addresses and names that Rocrail uses!

`min` and `max` indicate the locomotive address range in which you can select a locomotive.

##### Fixed mode

In the fixed mode, the configuration looks something like this:

```
    {
      "address": 99999,
      "name": "Lego",
      "bleHubs": [
        {
          "type": "PUController",
          "address": "e4:......",
          "range": {
            "portA": 1,
            "portB": 2
          }
        }
      ]
    },
```    
he fields `address` and `name` are not used, but there should be a unique value in it to prevent strange behaviour and it should not be in the addresses and names that Rocrail uses!

`portA` and `portB` holds the address of the locomotive you want to control.

#### Working with a PURemote

Both modes use the keys in different ways, the PU remote has two ports, port A on the left and port B on the right. In both mode the PU remote should be bound to the MTC4BT. The led on the remote indicates it state: 
 - short flashes: the remote is in 'Discovery' mode, the remote is searching for the MTC4BT;
 - 50% on and 50% off: the remote is discoverd and bound the the MTC4BT, but the layout is switched off, or in e-brake state;
 - 100% on: the remote is discoverd and bound to the MTC4BT and the layout is switched on.
 **All button actions work only in the latter two modes!**

##### Range mode

In the range mode, port B lets you select a locomotive in the configured range. At first, there are no locomotives loaded, pressing the 'green' button will turn on the layout, but also force a load of the locomotive list to MTC4BT. Pressing the '-' or '+' button on port B will also load the locomotives, no locomotive is selected, you need to press the '+' or '-' button a second time to select a locomotive.
Buttons on port B:
 - '+' button, get the next locomotive in the range, the controller led will change colour.
 - '-' button, get the previous locomotive in the range, the controller led will change colour.
 - 'red' button, the e-brake or emergency brake is pressed and send to Rocrail, the led wil be flashing on and off.
When the last locomotive in the range is selected, it will circle back to the first or last in the list depending on the button.
The colours the led can have are:
 - off: not used, the controller is not discoverd and off;
 - pink: number 1;
 - purple: number 2;
 - blue: number 3,
 - lighblue: number 4,
 - clyan: number 5,
 - green: number 6,
 - yellow: number 7,
 - orange: number 8,
 - red: number 9,
 - white: no locomotive selected.

Buttons on port A:
 - '+' button, increment the speed with 10, this could be '%' or 'kmh' until the max speed is reached;
 - '-' button, decrement the speed with 10, this could be  '%' or 'kmh' until the min speed is reached;
 - 'red' button, set the speed to 0.


##### Fixed mode

In the fixed mode, at first, there are no locomotives loaded, pressing the 'green' button will turn on the layout, but also force a load of the locomotive list to MTC4BT.
The address of the locomotives for port A and port B are the ones in the configuration.

Buttons on port A:
 - '+' button, increment the speed with 10, this could be '%' or 'kmh' until the max speed is reached;
 - '-' button, decrement the speed with 10, this could be  '%' or 'kmh' until the min speed is reached;
 - 'red' button, set the speed to 0.
 
Buttons on port B:
 - '+' button, increment the speed with 10, this could be '%' or 'kmh' until the max speed is reached;
 - '-' button, decrement the speed with 10, this could be  '%' or 'kmh' until the min speed is reached;
 - 'red' button, set the speed to 0.

---
For more information visit https://mattzobricks.com/controllers/mtc4bt
