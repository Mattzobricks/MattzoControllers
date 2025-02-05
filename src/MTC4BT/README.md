# Mattzo Train Controller for Bluetooth devices (MTC4BT)


The MTC4BT controller acts as a bridge between Bluetooth (Low Energy) devices and Rocrail.  
It currently supports the following type of BLE devices:
- [LEGO Powered Up](https://amzn.to/3VUsPAS)
- [SBrick](https://sbrick.com/?ref=7992)
- [BuWizz2](https://buwizz.com/?ref=240)
- [LEGO Remote Control](https://amzn.to/406qFjY)

Please note the firmware only runs on [ESP32](https://www.espressif.com/en/products/socs/esp32) MCU's.

The firmware has been tested successfully with these boards:
- AZDelivery ESP32 NodeMCU Module (verified by [MattzoBricks](https://mattzobricks.com/forums/users/rbrink))
- DOIT DEVKIT V1 ESP32-WROOM-32 Development Board (verified by [Steven Elston](https://mattzobricks.com/forums/users/steve1814))

If you have tested the firmware successfully with a different board, please let us know through [our forum](https://mattzobricks.com/forums/forum/mattzobricks-forum).

---
## Getting Started

Compiling and uploading the firmware requires Microsoft Visual Studio Code (VSCode) and PlatformIO.

You can run VSCode on Windows, Linux or Mac and there are loads of handy plug-ins for it. The mandatory extension is 'PlatformIO IDE'. This extension can be installed by pressing on the gear on the bottom left of the screen, then press "Extensions". On the "Extension in Marketplace", search for "PlatformIO". Click "install" and you are ready to go.

An easy way to start is to open in VS-Code the `MattzoControllers.code-workspace` in the root of this repository. You can open it in VS-Code under 'File->Open Workspace from file...', and then you can start editing.

To setup the environment, we have setup a [Getting Started guide](docs/README.md).

To get more information about the PU-Remote [PU remote implementation and config](docs/Remote.md)

---
## General Configuration

In the root of the workspace you'll find a file [my_platformio.ini.example](my_platformio.ini.example). You need to copy it and rename it to "my_platformio.ini". 

It is discouraged to edit the platformio.ini. This file is under source code control, comes with the project and may change when you update to a new version.

As the "my_platformio.ini" file is not under source code control, it will not be changed when you update the code. So your local configurations stay and all should compile.


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
Please be careful with powerlevels beyond "normal", as your appliances may be damaged due to overvoltage.

The channel supports the `power` field, this must be between 1 and 100, with this field it is possible to mix different motor types to achieve the same wheel speed.
```
"channels": [
            {
              "channel": "A",
              "attachedDevice": "motor",
              "power" : 75
            }
          ]
```

### Wired Ethernet

We support only the W5500 module, one a separate module, **any other module or wiring is not supported.**

To select wired or wireless mode a new section in `network_config.json` is introduced and some fields in the `wifi` section are deprecated.

Added is `network`:
```
"network" : {
		"hostname": "<some fancy hostname>",
		"otaPassword": "<some fancy password>",
		"type" : "wired"
}
```
The field `type` may have the following two values:
- `wireless` go in wireless mode (default if the `type` field is not specified;
- `wired` fo in wired mode.

If there is no wire connected or there is no wireless network, with the configure `wifi` settings, then there will be **NO** network connection!

The deprecated field is the `wifi` section are:
 - `"hostname"`
 - `"otaPassword"`
At the moment, if they are present, these values take precedence over the `network` section values, for backwards compatibility. There will be warnings in the log that you are using deprecated fields!

#### Using the W5500 ethernet module

The pins should be connected as follows:

| W5500 pin  | ESP32 pin |
|---|---|
| INT   | GPIO 25 |
| RESET | GPIO 26 |
| CS/SS | GPIO 5  |
| MOSI  | GPIO 23 |
| MISO  | GPIO 19 |
| SCK   | GPIO 18 |

GND and 3V3 are also connected, the other pins are not connected.


If a status LED is configured, it will turn on solid in network discovery. This will get the other statuses when out of setup and in the regular operation mode.

### Configuring the PURemote

#### Configuring the `controller_config.json`

The configuration has two modes, a list mode and a free mode.

##### List mode

In the list mode, the configuration looks something like this:

``
    "remotes": [
        {
            "name": "Lego remote",
            "type": "PUController",
            "address": "e4:e1:12:9b:42:14",
            "mode": "list",
            "list": [
                {
                    "id": "V100",
                    "type": "loco",
                    "color": "yellow"
                },
                {
                    "addr": 2,
                    "type": "loco",
                    "color": "red"
                },
                {
                    "id": "sw21",
                    "type": "switch",
                    "color": "pink"
                },
                {
                    "id": "sg24",
                    "type": "signal",
                    "color": "lightblue"
                },
                {
                    "id": "sw26",
                    "type": "switch",
                    "color": "blue"
                }
            ]
        }
    ]
```

When allowed types are:
 - "loco"
 - "switch"
 - "switch3"
 - "signal"
 - "output"
 - "system"

**Only** with the "loco" type you may use "id" or "addr", all other types **only** allow for "id".

##### Free mode

In the free mode, the configuration looks something like this:

```
    "remotes" : [
        {
            "name": "Lego remote",
            "type": "PUController",
            "address": "e4:e1:12:9b:42:14",
            "mode": "free",
            "buttons": [
                {
                    "button": "A+",
                    "type": "signal",
                    "id": "vs23-",
                    "action": "green"
                },
                {
                    "button": "A+",
                    "type": "switch",
                    "id": "sw22",
                    "action": "straight"
                },
                {
                    "button": "A+",
                    "type": "switch",
                    "id": "sg02-",
                    "action": "green"
                },
                {
                    "button": "A-",
                    "type": "switch",
                    "id": "sw21",
                    "action": "flip"
                },
                {
                    "button": "Ared",
                    "type": "switch",
                    "id": "sw26",
                    "action": "flip"
                },
                {
                    "button": "B+",
                    "type": "loco",
                    "id": "V100",
                    "action": "inc"
                },
                {
                    "button": "Bred",
                    "type": "loco",
                    "id": "V100",
                    "action": "stop"
                },
                {
                    "button": "B-",
                    "type": "loco",
                    "id": "V100",
                    "action": "dec"
                },
                {
                    "button": "Green",
                    "type": "loco",
                    "id": "V100",
                    "action": "fn4",
		                "fn-action" : "on"
                }
            ]
        }
    ]
```
    
Same types as the list mode, the allowed values for buttons are:
 - "A-"
 - "A+"
 - "Ared"
 - "B-"
 - "B+"
 - "Bred"
 - "Green"

For more information on how to used it see [mattzobricks](https://mattzobricks.com/automation/train-automation/mtc4bt/mtc4bt-with-remote-controls)

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
 - off, not used, the controller is not discoverd and off;
 - pink, number 1;
 - purple, number 2;
 - blue, number 3,
 - lighblue, number 4,
 - cyan, number 5,
 - green, number 6,
 - yellow, number 7,
 - orange, number 8,
 - red, number 9,
 - white no locomotive selected.

Buttons on port A:
 - '+' button, increment the speed with 10, this could be '%' or 'kmh' until the max speed is reached;
 - '-' button, decrement the speed with 10, this could be  '%' or 'kmh' until the min speed is reached;
 - 'red' button, set the speed to 0.


##### Fixed mode

In the fixed mode, at first, there are no locomotives loaded, pressing the 'green' button will turn on the layout, but also force a load of the locomotive list to MTC4BT.
The controller LED will turn green when a valid locomotive list is received from RocRail.
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
