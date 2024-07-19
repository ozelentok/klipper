#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)

OUTPUT_PATH="$SCRIPT_DIR/out/Robin_nano.bin"

cd "$SCRIPT_DIR"
make menuconfig
make -j
./scripts/update_mks_robin.py ./out/klipper.bin "$OUTPUT_PATH"
echo "Copy $OUTPUT_PATH to SD Card and flash printer"
