#!/bin/bash

# Path to the firmware relative to the script location
FIRMWARE="./.pio/build/teensy40/firmware.hex"
# Filename used on the remote Pi after copying
FIRMWARE_NAME="$(basename "$FIRMWARE")"

# Remote Pi information
PI_USER="pi"
PI_HOST="172.20.10.10"
PI_DEST="/home/pi/"
SSH_KEY_PATH=".ssh-keys/de-pi-228"
MICRO_ROS_WS="/home/pi/microros_ws"
MICRO_ROS_DEV="/dev/ttyACM0"

# Check if the firmware file exists
if [ ! -f "$FIRMWARE" ]; then
    echo "Error: Firmware file not found at $FIRMWARE"
    exit 1
fi

CMD_KILL_ROS_NODES='ps aux | grep ros | grep -v grep | awk '\''{print $2}'\'' | xargs -r kill -9'
CMD_LOAD_FIRMWARE="teensy_loader_cli -mmcu=TEENSY40 -w -v -s $FIRMWARE_NAME"
CMD_RUN_MICRO_ROS_AGENT="
    cd \"$MICRO_ROS_WS\" && 
    source \"install/setup.bash\" &&
    ros2 run micro_ros_agent micro_ros_agent serial --dev \"$MICRO_ROS_DEV\""

REMOTE_COMMANDS="
    $CMD_KILL_ROS_NODES &&
    cd $PI_DEST &&
    $CMD_LOAD_FIRMWARE &&
    $CMD_RUN_MICRO_ROS_AGENT
"

# Copy the firmware to the Pi
echo "Copying $FIRMWARE to $PI_USER@$PI_HOST:$PI_DEST ..."
scp -i "$SSH_KEY_PATH" "$FIRMWARE" "$PI_USER@$PI_HOST:$PI_DEST"
ssh -i "$SSH_KEY_PATH" "$PI_USER@$PI_HOST" "$REMOTE_COMMANDS"

# Check if scp succeeded
if [ $? -eq 0 ]; then
    echo "Firmware successfully copied to $PI_HOST"
else
    echo "Error: Failed to copy firmware"
    exit 1
fi
