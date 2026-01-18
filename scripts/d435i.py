import pyrealsense2 as rs
import numpy as np
import cv2, os
import gi
import time
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
Gst.init(None)

ROBOT_ID = os.getenv("ROBOT_ID", "1")
GSTREAMER_RGB_PORT = os.getenv("GSTREAMER_RGB_PORT", "1722")
GSTREAMER_DEPTH_PORT = os.getenv("GSTREAMER_DEPTH_PORT", "1723")

HOST = f"230.1.1.{ROBOT_ID}"
PUBLISH_RATE = 15
FRAME_RATE = 30

CUT_ROWS = 40

# ==== CAMERA SETTINGS ====
USE_MASTER_CAMERA_SETTINGS = True  # Use first camera's auto settings for all
AUTO_CALIBRATION_FRAMES = 30  # Number of frames to let auto-exposure settle

def gst_pipeline(port, height):
    """Creates a UDP GStreamer pipeline string for RK3588"""
    return (
        f"appsrc name=appsrc is-live=true block=true format=TIME ! "
        "videoconvert ! "
        f"video/x-raw,format=NV12,width=640,height={height},framerate={PUBLISH_RATE}/1 ! "
        f"mpph264enc bps={int(300000 * height / 480)} header-mode=1 ! "
        "rtph264pay ! "
        f"udpsink host={HOST} port={port} auto-multicast=true multicast-iface=wlan0 sync=false"
    )

def create_gst_app(port, width, height):
    pipeline_str = gst_pipeline(port, height)
    pipeline = Gst.parse_launch(pipeline_str)
    appsrc = pipeline.get_by_name("appsrc")
    appsrc.set_property("is-live", True)
    appsrc.set_property("format", Gst.Format.TIME)
    appsrc.set_property("caps",
        Gst.Caps.from_string(
            f"video/x-raw,format=BGR,width={width},height={height},framerate={PUBLISH_RATE}/1"
        )
    )
    pipeline.set_state(Gst.State.PLAYING)
    return pipeline, appsrc

def get_camera_serials():
    """Get serial numbers of connected D435i cameras"""
    ctx = rs.context()
    devices = ctx.query_devices()
    serials = []
    for dev in devices:
        serials.append(dev.get_info(rs.camera_info.serial_number))
    return serials

def get_camera_settings(pipeline):
    """Read current settings from a camera"""
    profile = pipeline.get_active_profile()
    device = profile.get_device()
    color_sensor = device.first_color_sensor()
    
    settings = {
        'exposure': color_sensor.get_option(rs.option.exposure),
        'gain': color_sensor.get_option(rs.option.gain),
        'white_balance': color_sensor.get_option(rs.option.white_balance),
    }
    
    # Try to get additional settings if available
    try:
        settings['brightness'] = color_sensor.get_option(rs.option.brightness)
        settings['contrast'] = color_sensor.get_option(rs.option.contrast)
        settings['saturation'] = color_sensor.get_option(rs.option.saturation)
        settings['sharpness'] = color_sensor.get_option(rs.option.sharpness)
    except:
        pass
    
    return settings

def apply_camera_settings(pipeline, settings):
    """Apply settings to a camera"""
    profile = pipeline.get_active_profile()
    device = profile.get_device()
    color_sensor = device.first_color_sensor()
    
    # Disable auto modes
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)
    color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
    
    # Apply main settings
    color_sensor.set_option(rs.option.exposure, settings['exposure'])
    color_sensor.set_option(rs.option.gain, settings['gain'])
    color_sensor.set_option(rs.option.white_balance, settings['white_balance'])
    
    # Apply additional settings if available
    try:
        if 'brightness' in settings:
            color_sensor.set_option(rs.option.brightness, settings['brightness'])
        if 'contrast' in settings:
            color_sensor.set_option(rs.option.contrast, settings['contrast'])
        if 'saturation' in settings:
            color_sensor.set_option(rs.option.saturation, settings['saturation'])
        if 'sharpness' in settings:
            color_sensor.set_option(rs.option.sharpness, settings['sharpness'])
    except:
        pass

def enable_auto_exposure(pipeline):
    """Enable auto-exposure and auto white balance on a camera"""
    profile = pipeline.get_active_profile()
    device = profile.get_device()
    color_sensor = device.first_color_sensor()
    
    color_sensor.set_option(rs.option.enable_auto_exposure, 1)
    color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

def disable_auto_exposure(pipeline):
    """Disable auto-exposure and auto white balance on a camera"""
    profile = pipeline.get_active_profile()
    device = profile.get_device()
    color_sensor = device.first_color_sensor()
    
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)
    color_sensor.set_option(rs.option.enable_auto_white_balance, 0)

def stream_realsense():
    # ==== Get available cameras ====
    serials = get_camera_serials()
    num_cameras = len(serials)
    
    if num_cameras == 0:
        print("❌ No RealSense cameras found!")
        return
    
    print(f"📷 Found {num_cameras} camera(s): {serials}")
    
    # Calculate output dimensions
    output_width = 640
    output_height = 480 * num_cameras - CUT_ROWS
    
    # ==== Initialize RealSense pipelines ====
    pipelines = []
    for i, serial in enumerate(serials):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, FRAME_RATE)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FRAME_RATE)
        pipeline.start(config)
        pipelines.append(pipeline)
        print(f"✓ Started camera {i+1}: {serial}")

    # ==== Synchronize camera settings ====
    if num_cameras > 1:
        FIXED_CAMERA_SETTINGS = {
            "exposure": 1000.0,        # microseconds
            "gain": 16.0,              # unitless
            "white_balance": 4600.0    # Kelvin
        }

        print(f"\n⚙️  Applying fixed settings to all cameras...")
        for i, pipeline in enumerate(pipelines):
            disable_auto_exposure(pipeline)  # make sure auto-exposure is off
            apply_camera_settings(pipeline, FIXED_CAMERA_SETTINGS)
            print(f"  ✓ Camera {i+1} configured with fixed settings")

        print(f"\n✓ All cameras configured with identical fixed settings!")

    # ==== Create GStreamer senders ====
    color_pipe, color_src = create_gst_app(port=GSTREAMER_RGB_PORT, width=output_width, height=output_height)
    depth_pipe, depth_src = create_gst_app(port=GSTREAMER_DEPTH_PORT, width=output_width, height=output_height)
    print(f"✓ GStreamer pipelines ready: {output_width}x{output_height} on ports 1722 (color) and 1723 (depth), multicast to {HOST}")

    align_to = rs.stream.color
    align = rs.align(align_to)
    hole_filling = rs.hole_filling_filter()

    max_time_diff_ms = 40  # ~1 frame at 30fps
    frame_count = 0
    target_frame_time = 1.0 / PUBLISH_RATE  # Time per frame at publish rate
    last_frame_time = time.time()

    try:
        while True:
            # Rate limiting: ensure we don't process faster than PUBLISH_RATE
            current_time = time.time()
            elapsed = current_time - last_frame_time
            if elapsed < target_frame_time:
                time.sleep(target_frame_time - elapsed)
            
            # Collect frames from all cameras using wait_for_frames (more efficient)
            best_match = []
            best_time_diff = float('inf')
            
            if num_cameras > 1:
                # For multiple cameras, try to get synchronized frames
                frames_list = []
                all_ready = True
                for i, pipeline in enumerate(pipelines):
                    try:
                        # Use wait_for_frames with short timeout to avoid blocking too long
                        frames = pipeline.wait_for_frames(timeout_ms=100)
                        if frames:
                            frames_list.append(frames)
                        else:
                            all_ready = False
                            break
                    except:
                        all_ready = False
                        break
                
                if all_ready and len(frames_list) == num_cameras:
                    # Check if frames are synchronized (simple check with first two)
                    if len(frames_list) >= 2:
                        time_diff = abs(frames_list[0].get_timestamp() - frames_list[1].get_timestamp())
                        if time_diff < max_time_diff_ms:
                            best_match = frames_list
                            best_time_diff = time_diff
                    else:
                        best_match = frames_list
                        best_time_diff = 0
            else:
                # Single camera - just wait for frames
                try:
                    frames = pipelines[0].wait_for_frames(timeout_ms=100)
                    if frames:
                        best_match = [frames]
                        best_time_diff = 0
                except:
                    pass

            if not best_match or best_time_diff >= max_time_diff_ms:
                # No valid frames available, sleep briefly to avoid busy-waiting
                time.sleep(0.001)  # 1ms sleep to prevent CPU spinning
                continue

            color_images = []
            depth_images = []
        
            # Get frames from all cameras
            all_frames_valid = True
            for i, frames in enumerate(best_match):
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                depth_frame = hole_filling.process(depth_frame)
                
                if not color_frame or not depth_frame:
                    all_frames_valid = False
                    break

                # Convert to numpy arrays (use get_data() directly for better performance)
                color_images.append(np.asanyarray(color_frame.get_data()))
                depth_images.append(np.asanyarray(depth_frame.get_data()))

            if not all_frames_valid:
                continue

            # Stack images if multiple cameras, otherwise use single image
            if num_cameras > 1:
                color_images[1] = color_images[1][CUT_ROWS:, :]
                color_combined = np.vstack(color_images)
                
                # Normalize and stack depth images
                depth_bgr_images = []
                for i, depth_image in enumerate(depth_images):
                    if i == 1:  # cut top CUT_ROWS rows for the second depth image too
                        depth_image = depth_image[CUT_ROWS:, :]

                    depth_normalized = np.clip(depth_image / 25, 0, 255).astype(np.uint8)
                    depth_gray_bgr = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)
                    depth_bgr_images.append(depth_gray_bgr)
                depth_combined = np.vstack(depth_bgr_images)
            else:
                color_combined = color_images[0]
                depth_normalized = np.clip(depth_images[0] / 25, 0, 255).astype(np.uint8)
                depth_combined = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)

            # --- Push COLOR frame ---
            color_bytes = color_combined.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(color_bytes), None)
            buf.fill(0, color_bytes)
            timestamp = Gst.util_uint64_scale(GLib.get_monotonic_time(), Gst.SECOND, 1000000)
            buf.pts = buf.dts = timestamp
            buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, PUBLISH_RATE)
            color_src.emit("push-buffer", buf)

            # --- Push DEPTH frame ---
            depth_bytes = depth_combined.tobytes()
            buf2 = Gst.Buffer.new_allocate(None, len(depth_bytes), None)
            buf2.fill(0, depth_bytes)
            buf2.pts = buf2.dts = timestamp
            buf2.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, PUBLISH_RATE)
            depth_src.emit("push-buffer", buf2)

            # Update frame time for rate limiting
            last_frame_time = time.time()
            frame_count += 1

            # Optional: preview locally
            # cv2.imshow("Color", color_combined)
            # cv2.imshow("Depth", depth_combined)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

    finally:
        for pipeline in pipelines:
            pipeline.stop()
        color_pipe.set_state(Gst.State.NULL)
        depth_pipe.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()
        print("🛑 Stopped streaming")

if __name__ == "__main__":
    stream_realsense()