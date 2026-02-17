#!/bin/bash

# Get directory of this script
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
MULTICAST_IP="230.1.1.${ROS_DOMAIN_ID}"
GSTREAMER_RGB_PORT=${GSTREAMER_RGB_PORT:-1722}
GSTREAMER_DEPTH_PORT=${GSTREAMER_DEPTH_PORT:-1723}

echo "Forwarding video stream to multicast IP ${MULTICAST_IP}:${GSTREAMER_RGB_PORT}"

# while ! pgrep -f '/unitree/module/video_hub/videohub'; do
#     echo "Wait for video_hub to start..."
#     sleep 1
# done

#### Binocular camera forwarding
### RFC 2435 encodes dimensions as width/8 and height/8 in a single byte each, giving a maximum of 2040x2040.
### Forward both cameras, the image size cannot exceed 2040x2040.
# gst-launch-1.0 v4l2src device=/dev/video2 do-timestamp=true ! \
#     image/jpeg,width=1600,height=600,framerate=15/1 ! \
#     mppjpegdec ! \
#     mpph264enc rc-mode=cbr bps=8000000 ! \
#     h264parse ! \
#     rtph264pay config-interval=1 ! \
#     udpsink host=${MULTICAST_IP} port=${GSTREAMER_RGB_PORT} auto-multicast=true multicast-iface=wlan0 sync=false

### Only forward right camera, the image size is 1280x720
gst-launch-1.0 v4l2src device=/dev/video2 do-timestamp=true ! \
    image/jpeg,width=2560,height=720,framerate=15/1 ! \
    mppjpegdec ! \
    videocrop right=1280 ! \
    queue max-size-buffers=2 leaky=downstream ! \
    mpph264enc rc-mode=cbr bps=4000000 zero-copy-pkt=true gop=15 ! \
    h264parse ! \
    queue max-size-buffers=2 leaky=downstream ! \
    rtph264pay config-interval=1 ! \
    udpsink host=${MULTICAST_IP} port=${GSTREAMER_RGB_PORT} auto-multicast=true multicast-iface=wlan0 sync=false

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
# python3 -u ${SCRIPT_DIR}/d435i.py