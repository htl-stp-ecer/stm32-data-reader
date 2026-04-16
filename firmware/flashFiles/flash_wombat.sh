#!/bin/bash
cd "$(dirname "${BASH_SOURCE[0]}")" || exit 1

BOOT0=17
RST=23
STM32FLASH='stm32flash'
BINFILE='wombat.bin'
DEV='/dev/ttyAMA0'

if [ $# -ne 0 ]; then
    BINFILE=$1
fi

# Initialize GPIOs
if ! bash ./init_gpio.sh; then
    echo "Failed to initialize GPIOs."
    exit 1
fi

# Set BOOT0 high so we stay in the bootloader on reboot
echo "Setting BOOT0 high..."
pinctrl set ${BOOT0} dh

# Reset co-processor
bash ./reset_coprocessor.sh

# Program the device
sleep 1
echo "Flashing firmware..."
CMD="${STM32FLASH} -v -S 0x08000000 -w ${BINFILE} ${DEV}"
echo $CMD
eval $CMD

# Set BOOT0 low to run the program after reset
echo "Setting BOOT0 low..."
pinctrl set ${BOOT0} dl

sleep 1

# Reset co-processor again
bash ./reset_coprocessor.sh

echo "Flashing process completed."