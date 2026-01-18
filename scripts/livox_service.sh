#!/bin/bash

# Get directory of this script
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "Starting lidar service..."
${SCRIPT_DIR}/../build/bin/typego_livox_full_service ${SCRIPT_DIR}/mid360_config.json &
