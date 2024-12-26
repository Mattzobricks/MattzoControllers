# List mode

Configuration and implementation description/hints

## Hardwired buttons

 - B+: navigate up
 - B-: navigate down
 - Bred: Emergency brake (switches layout power off and stops all trains)
 - Green: Layout power on (releases emergency brake)

## Other buttons

### Locomotive

For the locomotives (hard wired):

 - A+: increase speed (decrease speed if going backwards)
 - Ared: stop loco (set speed to 0)
 - A-: decrease speed (increase speed if going backwards)

### Accessories

The default actions for the buttons for accessories:

For switches:
 - A+: straight (triple switch: left)
 - Ared: flip (triple switch: straight)
 - A-: turnout (triple switch: right)

For outputs:
 - A+: on
 - Ared: flip
 - A-: off

For signals:
 - A+: green
 - Ared: flip
 - A-: red

All of these default assignments can be overridden by the configuration.

Allowed values:

For switches:
 - flip
 - straight
 - turnout
 - left
 - right
 - noop

For outputs:
 - flip
 - on
 - off
 - noop

For signals:
 - flip 
 - green
 - red
 - yellow
 - white
 - noop

## Implementation

Some kind of structure is needed to hold the configuration of the list buttons.

Remote switch structure, this list is leading, so from the item type in the list, the button and action is determined. 
```
  array of types (loco, switch, signal, output)
    array of buttons ( A+, Ared, A-, B+ Bred, B-, Green )
      action:     inc, dec, stop, flip, on, off, green, red, yellow, white, left, right,  straight, turnout, go, ebrake, noop
      reserved actions: navUp, navDown
```

The json would look like:
```
"buttons" : [
  {
    "button" : "Ared",
    "type"   : "signal",
    "action" : "red"
  },
  {
    "button" : "A-",
    "type"   : "signal",
    "action" : "flip"
  },
  {
    "button" : "Ared",
    "type"   : "switch",
    "action" : "turnout"
  },
  {
    "button" : "A-",
    "type"   : "switch",
    "action" : "flip"
  } 
]
```

List items
```
  "list"
    id/addr: address or identifier
    type:    loco, switch, signal, output
    ledcolour: (names from enum.h)
```
In list mode, the first item is default selected (after the data is collected from Rocrail)

In `Loop()` the ESP tries to get a list of locomotives of the current plan, if not it tries again every 10 seconds, until it gets one.

In the `init` of the remote it sets the colour of the remote, and if it is a locomotive also `currentLC` and sets `initiated` to false, meaning this item has no locomotive info.
The field `index` is set to -1 to indicate that the items are not  synchronised with Rocrail. The colour of the remote is still white and does not reflect the selection of the first colour in the list.

## Example configuration

**This is a part of the example**

```
    "remotes" : [
        {
            "address": 9999,
            "name": "Lego remote",
            "bleHubs": [
                {
                    "type": "PUController",
                    "address": "e4:e1:*",
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
                    ],
                    "buttons": [
                        {
                            "button": "Ared",
                            "type": "signal",
                            "action": "red"
                        },
                        {
                            "button": "A-",
                            "type": "signal",
                            "action": "flip"
                        },
                        {
                            "button": "Ared",
                            "type": "switch",
                            "action": "turnout"
                        },
                        {
                            "button": "A-",
                            "type": "switch",
                            "action": "flip"
                        }
                    ]
                }
            ]
        }
    ]

```

### Implementation hints for parsePortValueSingleMessage() port A and port B

Input values of this function are the port number and value of the button.

port 0 is port A

port 1 is port B

In combination with the values from the PURemote 0x01, 0x7f and 0xff it will combine to the the button pressed on the remote.

### Implementation hints for parseHWNetworkCommandMessage() Green button

`case 0x2` and `value == 1` means Green buttons in pressed.

# Free mode

## Implementation

Some kind of structure is needed to hold the configuration of the list buttons.

Type definition
```
   array of buttons
     buttonName: A+, Ared, A-, B+ Bred, B-, Green
       list of actions
         type:       loco, switch, signal, output
         id/addr:    address or identifier
         action:     inc, dec, stop, flip, on, off, green, red, yellow, white, left, right, straight, turnout, go, ebrake
```

Free items
```
  "free"
    button: A+, Ared, A-, B+ Bred, B-, Green
    type:   loco, switch, signal, output
    id/addr:    address or identifier
    action:     inc, dec, stop, flip, on, off, green, red, yellow, white, left, right, straight, turnout, go, ebrake
```

# Common function

 - checkAction(type, action) returns true if the type and action are a valid combination
 - checkButton(button) returns true if is a valid button name