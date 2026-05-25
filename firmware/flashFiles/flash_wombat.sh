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

# Enter bootloader: BOOT0 high during reset.
# NOTE: We can't use reset_coprocessor.sh here — it's a standalone normal-boot
# reset and forces BOOT0 low. We inline the RST toggle while holding BOOT0 high.
echo "Setting BOOT0 high and resetting into bootloader..."
pinctrl set ${BOOT0} op dh
pinctrl set ${RST} op dh
sleep 0.1
pinctrl set ${RST} dl
sleep 0.1
pinctrl set ${RST} dh

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