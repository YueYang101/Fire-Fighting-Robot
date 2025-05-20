#!/usr/bin/env python3
"""
Log one MLX-90640 frame every 2 s into an Excel workbook.
• Retries automatically on CRC errors with exponential back-off.
• Keeps a running worksheet index and timestamped sheet names.

If you want faster or slower rates just change REFRESH_* below.
"""

import time
import pathlib
import sys

import board
import busio
import numpy as np
import pandas as pd
import adafruit_mlx90640

# ─── USER-TUNABLE PARAMETERS ──────────────────────────────────────────────────

I2C_FREQUENCY_HZ   = 400_000          # 100 k / 400 k / 800 k – match your config.txt
REFRESH_RATE       = adafruit_mlx90640.RefreshRate.REFRESH_0_5_HZ  # 2 s / frame
BACKOFF_BASE_S     = 0.1             # 1st retry waits 0.1 s, 2nd 0.2 s, etc.
BACKOFF_MAX_S      = 2.0             # cap the wait so it never sleeps forever
EXCEL_FILENAME     = "thermal_frames.xlsx"

# ─── INITIALISE BUS & SENSOR ──────────────────────────────────────────────────

try:
    i2c = busio.I2C(board.SCL, board.SDA, frequency=I2C_FREQUENCY_HZ)
except TypeError:
    # Older CircuitPython builds don’t expose 'frequency'; fall back gracefully.
    i2c = busio.I2C(board.SCL, board.SDA)

mlx   = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = REFRESH_RATE

# ─── PREP OUTPUT ──────────────────────────────────────────────────────────────

xlsx_path = pathlib.Path(__file__).with_name(EXCEL_FILENAME)
flat      = np.zeros(24 * 32, dtype=np.float32)
sheet_idx = 0

print(f"Logging one worksheet every {2/REFRESH_RATE.value:.1f} s → {xlsx_path}")

# ─── MAIN LOOP ────────────────────────────────────────────────────────────────

retries = 0
try:
    while True:
        try:
            # ── Acquire a full 32×24 frame ───────────────────────────────────
            mlx.getFrame(flat)               # blocks ≈ 2 s at 0.5 Hz
            frame   = flat.reshape((24, 32))
            retries = 0                      # success ⇒ reset back-off

            # ── Write to a new worksheet ─────────────────────────────────────
            df      = pd.DataFrame(frame)
            stamp   = time.strftime("%Y-%m-%d_%Hh%Mm%Ss")

            with pd.ExcelWriter(
                    xlsx_path,
                    engine="openpyxl",
                    mode="a" if xlsx_path.exists() else "w",
                    if_sheet_exists="new") as w:
                df.to_excel(w, sheet_name=stamp,
                            index=False, header=False)

            sheet_idx += 1
            print(f"saved frame {sheet_idx:5d}  @ {stamp}")

        # ── CRC failure: back-off and retry ──────────────────────────────────
        except ValueError:
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"CRC error – retrying in {wait:.1f} s "
                  f"(attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)
            continue

        # ── Catch any other I²C-related hiccups the same way ────────────────
        except OSError as e:
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"I²C error ({e}) – retrying in {wait:.1f} s "
                  f"(attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)
            continue

# ─── CLEAN SHUTDOWN ───────────────────────────────────────────────────────────
except KeyboardInterrupt:
    print("\nStopped cleanly – Excel file is up-to-date.")
