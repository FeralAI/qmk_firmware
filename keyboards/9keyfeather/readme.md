# 9-key Feather Pad

This is a keyboard template based on the Adafruit Feather 32u4 BLE module. This firmware currently only uses pins from a MCP23017 GPIO expander for the keyboard matrix. It also uses an Adafruit FeatherWing 128x32 OLED for display using the built-in QMK OLED driver.

## Building

Make and install this keyboard (after setting up your build environment):

```sh
make 9keyfeather:default:avrdude
```

See the [build environment setup](https://docs.qmk.fm/#/getting_started_build_tools) and the [make instructions](https://docs.qmk.fm/#/getting_started_make_guide) for more information. Brand new to QMK? Start with our [Complete Newbs Guide](https://docs.qmk.fm/#/newbs).

## Credits

The MCP23017 implementation is a port of the [Adafruit MCP23017 Arduino library](https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library), using the QMK I2C driver instead of the Arduino Wire library. It also has some additional convenience methods.
