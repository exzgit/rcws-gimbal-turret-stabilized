#!/usr/bin/bash

# stop if error occurs
set -e

# Upload firmware in the STM32F411CEU via ST-Link
st-flash --reset write bin/firmware.bin 0x8000000
echo "Upload complete."
