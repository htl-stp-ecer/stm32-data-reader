#!/bin/bash
BOOT0=17
RST=23

echo "Resetting co-processor..."
# Ensure BOOT0 stays low and RST is configured as output before toggling.
# Without `op`, pinctrl can't drive the pin if it's still in its default input
# mode after a Pi reboot ("Can't set pin value, not an output").
pinctrl set ${BOOT0} op dl
pinctrl set ${RST} op dh
sleep 0.1
pinctrl set ${RST} dl
sleep 0.1
pinctrl set ${RST} dh