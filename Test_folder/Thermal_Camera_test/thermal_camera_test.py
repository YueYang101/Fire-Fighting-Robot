#!/usr/bin/env python3
"""
Read an MLX-90640 every 2 s, print the 24×32 matrix on the console,
and append the 768 values to a CSV file.

No Excel, no pandas, no openpyxl – just Python’s csv module.
"""

import csv, sys, time, warnings, board, busio, numpy as np
import adafruit_mlx90640

# ─── USER SETTINGS ────────────────────────────────────────────────────────────
REFRESH_RATE      = adafruit_mlx90640.RefreshRate.REFRESH_1_HZ  # sensor at 1 Hz
SAVE_EVERY_NTH    = 2        # keep one frame out of N → 2 s between CSV rows
CSV_FILENAME      = "thermal_frames.csv"
BACKOFF_BASE_S    = 0.1
BACKOFF_MAX_S     = 2.0
# ───────────────────────────────────────────────────────────────────────────────

warnings.filterwarnings(
    "ignore", category=RuntimeWarning,
    message="I2C frequency is not settable in python, ignoring!"
)

i2c = busio.I2C(board.SCL, board.SDA)            # bus already at 100 k Hz
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = REFRESH_RATE                   # 1 Hz is stable
flat = np.zeros(24 * 32, dtype=np.float32)

print(f"Logging one CSV row every {SAVE_EVERY_NTH / REFRESH_RATE:.1f} s → {CSV_FILENAME}")

# open the CSV once; newline='' is required on Windows but harmless on Linux
csv_file = open(CSV_FILENAME, "a", newline="")
csv_writer = csv.writer(csv_file)

frames_seen = 0
retries     = 0
try:
    while True:
        try:
            mlx.getFrame(flat)
            frames_seen += 1

            # skip frames we don’t want to save
            if frames_seen % SAVE_EVERY_NTH:
                continue

            # reshape for nice console output, but store the 1-D array
            frame2d = flat.reshape((24, 32))
            np.set_printoptions(precision=1, suppress=True)
            print(frame2d)               # ► shows the matrix in the terminal

            csv_writer.writerow(flat)    # ► one line, 768 values
            csv_file.flush()             # ensure it’s on disk
            print(f"saved frame {frames_seen // SAVE_EVERY_NTH:5d}\n")

            retries = 0                  # good read → reset back-off

        except ValueError:               # CRC
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"CRC error – retrying in {wait:.1f} s (attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)

        except OSError as e:             # other I²C hiccup
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"I²C error ({e}) – retrying in {wait:.1f} s (attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)

except KeyboardInterrupt:
    csv_file.close()
    print("\nStopped cleanly – CSV file is up-to-date.")
