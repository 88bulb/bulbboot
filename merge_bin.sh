#!/bin/bash
esptool.py \
    --chip esp32c3 merge_bin \
    -o bulbboot_merged.bin \
    --flash_mode dio \
    --flash_size 4MB \
    0x1000 build/bootloader/bootloader.bin \
    0x8000 build/partition_table/partition-table.bin \
    0x10000 build/bulbboot.bin
