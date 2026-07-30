#!/bin/bash
# Flash moteus_g431 via ST-Link + OpenOCD (bundled with CubeIDE)
# Usage: bash flash.sh   (run from project root)
#
# Note: OpenOCD on Windows needs C:/... paths, not /c/...

set -e

PLUGIN_DIR="C:/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins"
OPENOCD="$PLUGIN_DIR/com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.200.202505051030/tools/bin/openocd.exe"
SCRIPTS="$PLUGIN_DIR/com.st.stm32cube.ide.mcu.debug.openocd_2.3.100.202501240831/resources/openocd/st_scripts"

# ELF path: convert MSYS /c/... to Windows C:/... for openocd
ELF="$(pwd)/build/moteus_g431.elf"
ELF_WIN="${ELF/\/c\//C:/}"

if [ ! -f "$ELF" ]; then
    echo "ERROR: $ELF not found. Run 'make' first."
    exit 1
fi

echo "Flashing via ST-Link..."
"$OPENOCD" \
    -s "$SCRIPTS" \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "program $ELF_WIN verify reset exit"

echo "Done. MCU reset and running."
