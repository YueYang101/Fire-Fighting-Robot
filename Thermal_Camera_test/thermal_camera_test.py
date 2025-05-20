#!/usr/bin/env python3
"""
Logs an MLX-90640 frame to Excel every 2 s.
Work-around: sensor refresh = 1 Hz, but we save only alternate frames.
"""

import warnings, time, pathlib, sys, board, busio
import numpy as np, pandas as pd, adafruit_mlx90640

# ─── USER SETTINGS ────────────────────────────────────────────────────────────
EXCEL_FILENAME   = "thermal_frames.xlsx"
REFRESH_RATE     = adafruit_mlx90640.RefreshRate.REFRESH_1_HZ   # 1 Hz → 1 s / frame
SAVE_EVERY_NTH   = 2      # keep 1 frame out of N → 2 s between worksheets
BACKOFF_BASE_S   = 0.1
BACKOFF_MAX_S    = 2.0
# ───────────────────────────────────────────────────────────────────────────────

warnings.filterwarnings(
    "ignore", category=RuntimeWarning,
    message="I2C frequency is not settable in python, ignoring!"
)

# Hardware I²C is now at 100 k Hz from /boot/firmware/config.txt
i2c = busio.I2C(board.SCL, board.SDA)           # don’t pass frequency=…

mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = REFRESH_RATE                 # 1 Hz → stable

xlsx_path  = pathlib.Path(__file__).with_name(EXCEL_FILENAME)
flat       = np.zeros(24*32, dtype=np.float32)
sheet_idx  = 0
retries    = 0
frames_seen = 0

print(f"Logging one worksheet every {SAVE_EVERY_NTH/REFRESH_RATE:.1f} s → {xlsx_path}")

try:
    while True:
        try:
            mlx.getFrame(flat)
            frames_seen += 1

            # Skip frames we don’t want to save (makes the period 2 s)
            if frames_seen % SAVE_EVERY_NTH:
                continue

            frame   = flat.reshape((24, 32))
            retries = 0                          # good read → reset back-off

            df      = pd.DataFrame(frame)
            stamp   = time.strftime("%Y-%m-%d_%Hh%Mm%Ss")

            mode    = "a" if xlsx_path.exists() else "w"
            with pd.ExcelWriter(xlsx_path, engine="openpyxl",
                                mode=mode, if_sheet_exists="new") as w:
                df.to_excel(w, sheet_name=stamp,
                            index=False, header=False)

            sheet_idx += 1
            print(f"saved frame {sheet_idx:5d}  @ {stamp}")

        except ValueError:                       # CRC → exponential back-off
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"CRC error – retrying in {wait:.1f} s (attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)

        except OSError as e:                     # bus hiccup → same back-off
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"I²C error ({e}) – retrying in {wait:.1f} s (attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)

except KeyboardInterrupt:
    print("\nStopped cleanly – Excel file is up-to-date.")
