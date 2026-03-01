#!/bin/bash
BOOT0=17
RST=23

echo "Resetting co-processor..."
# Toggle RST: high -> low -> high
pinctrl set ${RST} dh
sleep 0.1
pinctrl set ${RST} dl
sleep 0.1
pinctrl set ${RST} dh