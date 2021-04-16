# arduino-python-can-interface

Provides a Python-CAN interface using an Arduino and a MCP2515-based CAN module.

Without modifying Python-CAN, it will just use the 11-bit IDs, but the modifications listed below will allow Python-CAN to transmit and receive extended frames as well.

## TODO

- [ ] Fix up documentation
- [ ] Allow for Python-CAN to set the baud rate of the CAN bus
    - this would mean another ad hoc change to Python-CAN

## Notes
* being that this runs on an Arduino Uno, this is pretty slow and may not accurately represent a bus with tight timing requirements.
* this requires modifying the python-can code to accomodate using the 31st  bit of the arbitration ID field to denote whether the message should be an extended frame (29-bit identifier) of r astandard frame (11-bit identifier)

### Changing Python-CAN to be aware of extended frames

Edits to serial_can.py are as follows:

#### In `send()`

before `a_id = struct.pack('<I', msg.arbitration_id)`, add to be:

```c
[...]
if msg.is_extended_id:
                msg.arbitration_id += 0x80000000
            a_id = struct.pack('<I', msg.arbitration_id)
[...]
```

#### In `_recv_internal()`
```c
[...]
arb_id = (struct.unpack('<I', s))[0]
isExtended = False
        if arb_id & 0x80000000:
            isExtended = True
            arb_id = arb_id & 0x1FFFFFFF
[...]
```

and, further down:
```c
[...]
msg = Message(timestamp=timestamp/1000,
                              arbitration_id=arb_id,
                              dlc=dlc,
                              extended_id=isExtended,
                              data=data)
[...]
```


