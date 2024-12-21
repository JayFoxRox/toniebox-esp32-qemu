#!/usr/bin/env bash

set -e
set -u

# SD image was created using:
# ./build/qemu-img convert -f raw -O qcow2 sd-backup.img sd.qcow2

# Arch started some debug-info url bullshit now
unset DEBUGINFOD_URLS

ninja -C ./build
gdb --args ./build/qemu-system-xtensa -nographic \
    -machine esp32s3 \
    -drive file=../emulation/patched.bin,if=mtd,format=raw \
    -global driver=timer.esp32.timg,property=wdt_disable,value=true \
    -drive file=../emulation/sd.qcow2,if=sd,format=qcow2 \
    --trace led_set_intensity \
    -s -S

