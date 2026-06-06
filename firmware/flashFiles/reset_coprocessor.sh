#!/bin/bash
BOOT0=17
RST=23

echo "Resetting co-processor..."
# Standalone normal-boot reset: BOOT0 low, then toggle RST.
# `op` ensures both pins are outputs even after a fresh Pi boot
# (otherwise pinctrl errors with "Can't set pin value, not an output").
pinctrl set ${BOOT0} op dl
pinctrl set ${RST} op dh
sleep 0.1
pinctrl set ${RST} dl
sleep 0.1
pinctrl set ${RST} dh