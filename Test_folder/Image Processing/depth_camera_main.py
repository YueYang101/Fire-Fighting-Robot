import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    # 1. Configure depth & color streams
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 2. Start streaming
    profile = pipeline.start(config)

    # 3. Get depth sensor’s scale (to convert from raw units to meters)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale  = depth_sensor.get_depth_scale()
    print(f"[INFO] Depth scale is {depth_scale:.4f} meters per unit")

    # 4. Create an align object to align depth frames to color frames
    align_to = rs.stream.color
    align    = rs.align(align_to)

    try:
        while True:
            # 5. Wait for a coherent pair of frames: depth + color
            frames        = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            # 6. Extract aligned frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # 7. Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 8. Normalize depth to 0–255 for display and apply a colormap
            #    alpha controls contrast; tweak if it’s too dark/bright.
            depth_display = cv2.convertScaleAbs(depth_image, alpha=0.03)
            depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

            # 9. Stack both images horizontally
            combined = np.hstack((color_image, depth_colormap))

            # 10. Show in a window
            cv2.namedWindow('RealSense (RGB | Depth)', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense (RGB | Depth)', combined)

            # 11. Exit on ESC key
            if cv2.waitKey(1) & 0xFF == 27:
                break

    finally:
        # 12. Clean up
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
