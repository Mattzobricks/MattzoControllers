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
 - (nop)

For outputs:
 - flip
 - on
 - off
 - (nop)

For signals:
 - flip 
 - green
 - red
 - yellow
 - white
 - (nop)

## Implementation

Some kind of structure is needed to hold the configuration of the list buttons.

Remote switch structure, this list is leading, so from the item type in the list, the button and action is determined. 
```
   type: loco, switch, signal, output
     array of buttons
       buttonName: A+, Ared, A-, B+ Bred, B-, Green
       action:     inc, dec, stop, flip, on, off, green, red, yellow, white, left, right, straight, turnout, go, ebrake
```

List items
```
  "list"
    id/addr: address or identifier
    type:    loco, switch, signal, output
    ledcolour: (names from enum.h)
```

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

checkAction(type, action) returns true if the type and action are a valid combination
checkButton(button) returns true if is a valid button name