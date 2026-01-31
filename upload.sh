#!/bin/bash

show_help() {
    echo "Usage: $0 [--python] [--teensy]"
    echo ""
    echo "  --python     Deploy python files via SCP"
    echo "  --teensy     Flash Teensy firmware"
    exit 1
}

DEPLOY_PYTHON=false
FLASH_TEENSY=false

# -------------------------
# Parse arguments
# -------------------------
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --python)
            DEPLOY_PYTHON=true
            ;;
        --teensy)
            FLASH_TEENSY=true
            ;;
        -h|--help)
            show_help
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            ;;
    esac
    shift
done

# -------------------------
# Perform actions
# -------------------------

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
MICRO_ROS_DEV="/dev/ttyACM1"
SERIAL_BAUD="115200"

if $FLASH_TEENSY; then
    echo "Flashing Teensy firmware..."
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
        ros2 run micro_ros_agent micro_ros_agent serial --dev \"$MICRO_ROS_DEV\" -b \"$SERIAL_BAUD\""

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
fi

if $DEPLOY_PYTHON; then
    echo "Uploading Python files..."
    declare -A PY_FILES=(
        ["ros_nodes/labyrinthi.py"]="/home/pi/lab_6/ros_ws/src/labyrinthi/labyrinthi/labyrinthi.py"
        ["ros_nodes/long_runny.py"]="/home/pi/lab_6/ros_ws/src/long_runny/long_runny/long_runny.py"
        ["ros_nodes/randi.py"]="/home/pi/lab_6/ros_ws/src/randi/randi/randi.py"
        ["ros_nodes/robbi.py"]="/home/pi/lab_6/ros_ws/src/robbi/robbi/robbi.py"
        ["ros_nodes/sensi.py"]="/home/pi/lab_6/ros_ws/src/sensi/sensi/sensi.py"
        ["ros_nodes/vissy.py"]="/home/pi/lab_6/ros_ws/src/vissy/vissy/vissy.py"
    )

    if $DEPLOY_PYTHON; then
        echo "Uploading Python files…"
        for LOCAL in "${!PY_FILES[@]}"; do
            REMOTE="${PY_FILES[$LOCAL]}"
            echo " → $LOCAL  →  $PI_USER@$PI_HOST:$REMOTE"
            scp -i "$SSH_KEY_PATH" "$LOCAL" "$PI_USER@$PI_HOST:$REMOTE"
        done
    fi
fi


if ! $DEPLOY_PYTHON && ! $FLASH_TEENSY; then
    echo "No action selected."
    show_help
fi



