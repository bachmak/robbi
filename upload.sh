#!/bin/bash

# Path to the firmware relative to the script location
FIRMWARE="./.pio/build/teensy40/firmware.hex"

# Remote Pi information
PI_USER="pi"
PI_HOST="de-pi-228.local"
SSH_KEY_PATH="~/.ssh/de-pi-228"
PI_DEST="/home/pi/"

# Check if the firmware file exists
if [ ! -f "$FIRMWARE" ]; then
    echo "Error: Firmware file not found at $FIRMWARE"
    exit 1
fi

# Copy the firmware to the Pi
echo "Copying $FIRMWARE to $PI_USER@$PI_HOST:$PI_DEST ..."
scp -i "$SSH_KEY_PATH" "$FIRMWARE" "$PI_USER@$PI_HOST:$PI_DEST"
ssh -i "$SSH_KEY_PATH" "$PI_USER@$PI_HOST" "teensy_loader_cli -mmcu=TEENSY40 -w -v -s firmware.hex"

# Check if scp succeeded
if [ $? -eq 0 ]; then
    echo "Firmware successfully copied to $PI_HOST"
else
    echo "Error: Failed to copy firmware"
    exit 1
fi
