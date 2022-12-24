#!/bin/bash

set -x

arm-none-eabi-objcopy -O binary F103shell.elf F103shell.bin
