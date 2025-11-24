#!/bin/bash

# Get directory of this script
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "Starting high state service..."
${SCRIPT_DIR}/../build/bin/typego_high_state_service &
