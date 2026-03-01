#!/bin/bash
# GPIO numbers
BOOT0=17
RST=23

echo "Initializing GPIOs..."
# Initialize BOOT0 to low (output, drive low)
pinctrl set ${BOOT0} op dl
# Initialize RST to high (output, drive high)
pinctrl set ${RST} op dh