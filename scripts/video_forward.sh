#!/bin/bash

# Get directory of this script
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

ROBOT_ID=${ROBOT_ID:-1}
MULTICAST_IP="230.1.1.${ROBOT_ID}"
GSTREAMER_RGB_PORT=${GSTREAMER_RGB_PORT:-1722}
GSTREAMER_DEPTH_PORT=${GSTREAMER_DEPTH_PORT:-1723}

# while ! pgrep -f '/unitree/module/video_hub/videohub'; do
#     echo "Wait for video_hub to start..."
#     sleep 1
# done

#### Dual fisheye camera forwarding
# gst-launch-1.0 v4l2src device=/dev/video2 do-timestamp=true ! \
#    image/jpeg,width=1600,height=600,framerate=30/1 ! \
#    rtpjpegpay quality=80 timestamp-offset=0 ! \
#    udpsink host=${MULTICAST_IP} port=${GSTREAMER_RGB_PORT} auto-multicast=true multicast-iface=wlan0 sync=false

#### Go2 native camera forwarding
# echo "Forwarding Go2 camera stream to multicast IP ${MULTICAST_IP}:${GSTREAMER_RGB_PORT}"
# gst-launch-1.0 -v \
#   udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 \
#   ! udpsink host=${MULTICAST_IP} port=${GSTREAMER_RGB_PORT} auto-multicast=true multicast-iface=wlan0

# gst-launch-1.0 -v \
#   udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 \
#   ! application/x-rtp, media=video, encoding-name=H264 \
#   ! queue \
#   ! udpsink host=${MULTICAST_IP} port=${GSTREAMER_RGB_PORT} auto-multicast=true multicast-iface=wlan0

#### Depth Camera D435i Streaming
# Use -u flag for unbuffered output so print statements appear in logs immediately
python3 -u ${SCRIPT_DIR}/d435i.py