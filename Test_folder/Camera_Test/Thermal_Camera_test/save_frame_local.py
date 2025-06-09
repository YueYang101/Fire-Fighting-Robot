#!/usr/bin/env python3
"""
Read an MLX-90640 thermal camera, display the 24×32 matrix in the terminal,
and save each frame to a CSV file locally.
"""

import sys, time, warnings, board, busio, numpy as np
import adafruit_mlx90640
import os
from datetime import datetime

# ─── USER SETTINGS ────────────────────────────────────────────────────────────
REFRESH_RATE      = adafruit_mlx90640.RefreshRate.REFRESH_1_HZ  # sensor at 1 Hz
DISPLAY_INTERVAL  = 2.0      # seconds between displays
SAVE_EVERY_FRAME  = True     # save every frame, not just displayed ones
SHOW_STATS        = True     # show min/max/avg temperature
SAVE_DIRECTORY    = "thermal_data"  # directory to save CSV files
BACKOFF_BASE_S    = 0.1
BACKOFF_MAX_S     = 2.0
# ───────────────────────────────────────────────────────────────────────────────

warnings.filterwarnings(
    "ignore", category=RuntimeWarning,
    message="I2C frequency is not settable in python, ignoring!"
)

# Create save directory if it doesn't exist
if not os.path.exists(SAVE_DIRECTORY):
    os.makedirs(SAVE_DIRECTORY)
    print(f"Created directory: {SAVE_DIRECTORY}")

i2c = busio.I2C(board.SCL, board.SDA)            # bus already at 100 kHz
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = REFRESH_RATE                   # 1 Hz is stable
flat = np.zeros(24 * 32, dtype=np.float32)

print(f"MLX90640 Thermal Camera - Displaying every {DISPLAY_INTERVAL} seconds")
print(f"Saving frames to: {os.path.abspath(SAVE_DIRECTORY)}/")
print("Press Ctrl+C to stop\n")

frames_seen = 0
frames_saved = 0
retries = 0
last_display_time = 0

def save_frame_to_csv(frame_2d, frame_number):
    """Save a single frame to a CSV file with timestamp"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # milliseconds
    filename = f"thermal_frame_{timestamp}_f{frame_number:06d}.csv"
    filepath = os.path.join(SAVE_DIRECTORY, filename)
    
    try:
        # Save with header showing frame info
        header = f"Frame {frame_number}, Time: {datetime.now().isoformat()}, Shape: 24x32"
        np.savetxt(filepath, frame_2d, delimiter=',', fmt='%.2f', header=header)
        return True, filepath
    except Exception as e:
        print(f"Error saving frame: {e}", file=sys.stderr)
        return False, None

try:
    while True:
        try:
            mlx.getFrame(flat)
            frames_seen += 1
            current_time = time.time()
            
            # Reshape for processing
            frame2d = flat.reshape((24, 32))
            
            # Save frame if enabled
            if SAVE_EVERY_FRAME:
                success, filepath = save_frame_to_csv(frame2d, frames_seen)
                if success:
                    frames_saved += 1
                    # Show save confirmation in terminal
                    print(f"\r[Frame {frames_seen}] Saved: {os.path.basename(filepath)} | "
                          f"Min: {flat.min():.1f}°C, Max: {flat.max():.1f}°C, "
                          f"Avg: {flat.mean():.1f}°C", end='', flush=True)

            # Display detailed view at specified interval
            if current_time - last_display_time >= DISPLAY_INTERVAL:
                # Clear previous single-line output
                print("\r" + " " * 100 + "\r", end='')
                
                # Clear screen for cleaner display (optional)
                print("\033[2J\033[H", end='')  # ANSI escape codes to clear screen
                
                np.set_printoptions(precision=1, suppress=True, linewidth=200)
                
                print(f"Frame #{frames_seen} - Thermal Data (°C):")
                print("=" * 80)
                print(frame2d)
                
                if SHOW_STATS:
                    print("\n" + "─" * 40)
                    print(f"Min Temperature:  {flat.min():.1f}°C")
                    print(f"Max Temperature:  {flat.max():.1f}°C")
                    print(f"Avg Temperature:  {flat.mean():.1f}°C")
                    print(f"Center Temp:      {frame2d[12, 16]:.1f}°C")  # center pixel
                    
                    # Find hottest spot
                    max_idx = np.argmax(flat)
                    max_row, max_col = divmod(max_idx, 32)
                    print(f"Hottest Pixel:    [{max_row}, {max_col}] = {flat[max_idx]:.1f}°C")
                
                print(f"\nFrames captured: {frames_seen} | Frames saved: {frames_saved}")
                print("Files saved to:", os.path.abspath(SAVE_DIRECTORY))
                print("=" * 80 + "\n")
                
                last_display_time = current_time

            retries = 0  # good read → reset back-off

        except ValueError:  # CRC error
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"\nCRC error – retrying in {wait:.1f} s (attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)

        except OSError as e:  # other I²C hiccup
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"\nI²C error ({e}) – retrying in {wait:.1f} s (attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)

except KeyboardInterrupt:
    print("\n\nStopped cleanly.")
    print(f"Total frames read: {frames_seen}")
    print(f"Total frames saved: {frames_saved}")
    print(f"Data saved in: {os.path.abspath(SAVE_DIRECTORY)}/")
    
    # Show last few saved files
    if frames_saved > 0:
        files = sorted(os.listdir(SAVE_DIRECTORY))[-5:]  # last 5 files
        print("\nLast saved files:")
        for f in files:
            print(f"  - {f}")