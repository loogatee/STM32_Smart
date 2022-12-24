#!/usr/bin/bash

#arm-atollic-eabi-objcopy -O binary waveshare.elf waveshare.bin
#
#../Host/crc.exe xx
#
set -x
st-flash write F103shell.bin 0x08000000
