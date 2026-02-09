#!/bin/bash

# Get directory of this script
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Load .env if present
if [[ -f "${SCRIPT_DIR}/.env" ]]; then
    set -a
    source "${SCRIPT_DIR}/.env"
    set +a
else
    echo "Warning: No .env file found in ${SCRIPT_DIR}"
fi

sleep 5
echo "Starting background services..."

"${SCRIPT_DIR}/video_forward.sh" &
"${SCRIPT_DIR}/livox_service.sh" &
"${SCRIPT_DIR}/control_interface.sh" &
# "${SCRIPT_DIR}/audio_forward.sh" &
"${SCRIPT_DIR}/high_state_service.sh" &

echo "All services launched. Waiting for them to exit..."

# sleep forever
sleep infinity