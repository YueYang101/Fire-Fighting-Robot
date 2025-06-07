#!/usr/bin/env python3
"""
Read an MLX-90640 thermal camera and display the 24×32 matrix in the terminal.
No CSV saving - just live display.
"""

import sys, time, warnings, board, busio, numpy as np
import adafruit_mlx90640

# ─── USER SETTINGS ────────────────────────────────────────────────────────────
REFRESH_RATE      = adafruit_mlx90640.RefreshRate.REFRESH_1_HZ  # sensor at 1 Hz
DISPLAY_INTERVAL  = 2.0      # seconds between displays
SHOW_STATS        = True     # show min/max/avg temperature
BACKOFF_BASE_S    = 0.1
BACKOFF_MAX_S     = 2.0
# ───────────────────────────────────────────────────────────────────────────────

warnings.filterwarnings(
    "ignore", category=RuntimeWarning,
    message="I2C frequency is not settable in python, ignoring!"
)

i2c = busio.I2C(board.SCL, board.SDA)            # bus already at 100 kHz
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = REFRESH_RATE                   # 1 Hz is stable
flat = np.zeros(24 * 32, dtype=np.float32)

print(f"MLX90640 Thermal Camera - Displaying every {DISPLAY_INTERVAL} seconds")
print("Press Ctrl+C to stop\n")

frames_seen = 0
retries = 0
last_display_time = 0

try:
    while True:
        try:
            mlx.getFrame(flat)
            frames_seen += 1
            current_time = time.time()

            # Only display at the specified interval
            if current_time - last_display_time >= DISPLAY_INTERVAL:
                # Clear screen for cleaner display (optional)
                print("\033[2J\033[H", end='')  # ANSI escape codes to clear screen
                
                # Reshape for nice console output
                frame2d = flat.reshape((24, 32))
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
                
                print("\n" + "=" * 80 + "\n")
                last_display_time = current_time

            retries = 0  # good read → reset back-off

        except ValueError:  # CRC error
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"CRC error – retrying in {wait:.1f} s (attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)

        except OSError as e:  # other I²C hiccup
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"I²C error ({e}) – retrying in {wait:.1f} s (attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)

except KeyboardInterrupt:
    print("\n\nStopped cleanly.")
    print(f"Total frames read: {frames_seen}")