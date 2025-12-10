#!/bin/bash

# Get directory of this script
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

while ! pgrep -f '/unitree/module/basic_service/basic_service'; do
    sleep 1
done

echo "Starting control interface..."
${SCRIPT_DIR}/../build/bin/typego_control_interface &
${SCRIPT_DIR}/../build/bin/typego_audio_interface &
