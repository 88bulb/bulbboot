#!/bin/bash
esptool.py \
    --chip esp32c3 merge_bin \
    -o bulbboot_merged.bin \
    --flash_mode dio \
    --flash_size 4MB \
    0x00000 build/bootloader/bootloader.bin \
    0x08000 build/partition_table/partition-table.bin \
    0x0d000 build/ota_data_initial.bin \
    0x10000 build/bulbboot.bin
